import crypto from 'node:crypto'
import { execFileSync } from 'node:child_process'
import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const scriptDir = path.dirname(fileURLToPath(import.meta.url))
const siteDir = path.resolve(scriptDir, '..')
const workspace = path.resolve(siteDir, '..')

const trackedInputs = [
  'docs',
  'docs-site/app',
  'docs-site/components',
  'docs-site/deploy',
  'docs-site/scripts',
  'docs-site/mdx-components.js',
  'docs-site/next.config.mjs',
  'docs-site/package.json',
  'docs-site/patches',
  'docs-site/pnpm-lock.yaml',
  'docs-site/public/favicon.svg',
  'docs-site/public/yundrone-logo.svg',
  'tools/docs-site.sh'
]

async function collectFiles(target, files) {
  let stat
  try {
    stat = await fs.lstat(target)
  } catch (error) {
    if (error.code === 'ENOENT') return
    throw error
  }

  if (stat.isDirectory()) {
    const entries = await fs.readdir(target)
    for (const entry of entries.sort()) {
      if (entry.startsWith('.') || entry === 'node_modules') continue
      await collectFiles(path.join(target, entry), files)
    }
    return
  }

  if (stat.isFile()) files.push(target)
}

async function digestPaths(inputs) {
  const files = []
  for (const input of inputs) {
    await collectFiles(path.join(workspace, input), files)
  }

  const hash = crypto.createHash('sha256')
  for (const file of files.sort()) {
    const relative = path.relative(workspace, file).split(path.sep).join('/')
    hash.update(relative)
    hash.update('\0')
    hash.update(await fs.readFile(file))
    hash.update('\0')
  }
  return hash.digest('hex')
}

function git(...args) {
  return execFileSync('git', args, {
    cwd: workspace,
    encoding: 'utf8',
    stdio: ['ignore', 'pipe', 'ignore']
  }).trim()
}

function utcReleaseId(date, sha, dirty) {
  const timestamp = date
    .toISOString()
    .replace(/[-:]/g, '')
    .replace(/\.\d{3}Z$/, 'Z')
  return `${timestamp}-${sha.slice(0, 8)}${dirty ? '-dirty' : ''}`
}

async function currentRelease() {
  const builtAt = new Date()
  const gitSha = git('rev-parse', 'HEAD')
  const dirty = Boolean(git('status', '--porcelain', '--untracked-files=all'))
  return {
    schemaVersion: 1,
    project: 'robotac',
    basePath: '/robotac',
    releaseId: utcReleaseId(builtAt, gitSha, dirty),
    gitSha,
    dirty,
    builtAt: builtAt.toISOString(),
    docsDigest: await digestPaths(['docs']),
    inputDigest: await digestPaths(trackedInputs)
  }
}

const command = process.argv[2] || 'write'
const release = await currentRelease()

if (command === '--docs-digest') {
  console.log(release.docsDigest)
  process.exit(0)
}

if (command === '--show') {
  console.log(JSON.stringify(release, null, 2))
  process.exit(0)
}

if (command === '--check') {
  try {
    const existing = JSON.parse(
      await fs.readFile(path.join(siteDir, 'out', 'release.json'), 'utf8')
    )
    process.exit(existing.inputDigest === release.inputDigest ? 0 : 1)
  } catch {
    process.exit(1)
  }
}

await fs.mkdir(path.join(siteDir, 'public'), { recursive: true })
await fs.writeFile(
  path.join(siteDir, 'public', 'release.json'),
  `${JSON.stringify(release, null, 2)}\n`,
  'utf8'
)
await fs.writeFile(
  path.join(siteDir, '.build-state.json'),
  `${JSON.stringify(release, null, 2)}\n`,
  'utf8'
)
console.log(`准备发布版本 ${release.releaseId}`)
