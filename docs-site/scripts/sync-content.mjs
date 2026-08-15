import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import remarkGfm from 'remark-gfm'
import remarkParse from 'remark-parse'
import remarkStringify from 'remark-stringify'
import { unified } from 'unified'
import { visit } from 'unist-util-visit'

const scriptDir = path.dirname(fileURLToPath(import.meta.url))
const siteDir = path.resolve(scriptDir, '..')
const workspace = path.resolve(siteDir, '..')
const sourceDir = path.join(workspace, 'docs')
const contentDir = path.join(siteDir, 'content')

function splitUrl(url) {
  const match = url.match(/^([^?#]*)(.*)$/)
  return { pathname: match?.[1] ?? url, suffix: match?.[2] ?? '' }
}

function rewriteSiteLinks(sourceRelativePath) {
  return tree => {
    visit(tree, 'link', node => {
      if (!node.url || /^(?:[a-z]+:|#|\/)/i.test(node.url)) return
      const { pathname, suffix } = splitUrl(node.url)
      if (path.posix.extname(pathname).toLowerCase() !== '.md') return

      const sourceDirectory = path.posix.dirname(
        sourceRelativePath.split(path.sep).join('/')
      )
      const resolved = path.posix.normalize(
        path.posix.join(sourceDirectory, pathname)
      )
      const route = /(?:^|\/)README\.md$/i.test(resolved)
        ? path.posix.dirname(resolved).replace(/^\.$/, '')
        : resolved.replace(/\.md$/i, '')
      node.url = `/${route}${route ? '/' : ''}${suffix}`
    })
  }
}

function createProcessor(sourceRelativePath) {
  return unified()
    .use(remarkParse)
    .use(remarkGfm)
    .use(rewriteSiteLinks, sourceRelativePath)
    .use(remarkStringify, {
    bullet: '-',
    emphasis: '*',
    fences: true,
    listItemIndent: 'one'
  })
}

function firstHeading(tree, fallback) {
  let title = ''
  visit(tree, 'heading', node => {
    if (title || node.depth !== 1) return
    title = node.children
      .filter(child => child.type === 'text' || child.type === 'inlineCode')
      .map(child => child.value)
      .join('')
      .trim()
  })
  return title || fallback
}

function outputRelativePath(relativePath) {
  if (path.basename(relativePath).toLowerCase() !== 'readme.md') {
    return relativePath
  }
  return path.join(path.dirname(relativePath), 'index.md')
}

async function walk(directory) {
  const entries = await fs.readdir(directory, { withFileTypes: true })
  const files = []
  for (const entry of entries.sort((a, b) => a.name.localeCompare(b.name))) {
    if (entry.name.startsWith('.')) continue
    const absolute = path.join(directory, entry.name)
    if (entry.isDirectory()) files.push(...(await walk(absolute)))
    else if (entry.isFile()) files.push(absolute)
  }
  return files
}

await fs.mkdir(contentDir, { recursive: true })
await fs.rm(path.join(siteDir, 'public/release.json'), { force: true })

const sourceFiles = await walk(sourceDir)
let markdownCount = 0
const sourceMap = {}
const folderTitles = {}
const expectedContentFiles = new Set()

async function writeIfChanged(filename, content) {
  const buffer = Buffer.isBuffer(content) ? content : Buffer.from(content)
  try {
    if (Buffer.compare(await fs.readFile(filename), buffer) === 0) return false
  } catch (error) {
    if (error.code !== 'ENOENT') throw error
  }
  await fs.mkdir(path.dirname(filename), { recursive: true })
  await fs.writeFile(filename, buffer)
  return true
}

async function writeContent(relativePath, content) {
  const filename = path.join(contentDir, relativePath)
  expectedContentFiles.add(filename)
  await writeIfChanged(filename, content)
}

async function pruneGenerated(directory) {
  const entries = await fs.readdir(directory, { withFileTypes: true })
  for (const entry of entries) {
    const absolute = path.join(directory, entry.name)
    if (entry.isDirectory()) {
      await pruneGenerated(absolute)
      if ((await fs.readdir(absolute)).length === 0) await fs.rmdir(absolute)
    } else if (!expectedContentFiles.has(absolute)) {
      await fs.rm(absolute)
    }
  }
}

function addSidebarEntry(destinationRelative, title) {
  if (path.basename(destinationRelative).toLowerCase() !== 'index.md') return
  const directory = path.dirname(destinationRelative)
  if (directory !== '.') folderTitles[directory.split(path.sep).join('/')] = title
}

for (const sourcePath of sourceFiles) {
  const relativePath = path.relative(sourceDir, sourcePath)
  if (relativePath.split(path.sep)[0] === 'diagrams') continue
  const destinationRelative = outputRelativePath(relativePath)

  if (path.extname(sourcePath).toLowerCase() !== '.md') {
    await writeContent(destinationRelative, await fs.readFile(sourcePath))
    continue
  }

  const source = await fs.readFile(sourcePath, 'utf8')
  const processor = createProcessor(relativePath)
  const tree = processor.parse(source)
  const title = firstHeading(tree, path.basename(sourcePath, '.md'))
  const transformed = processor.stringify(await processor.run(tree))
  const sourceFileUrl = `https://github.com/YunDrone-Team/robotac/blob/main/docs/${relativePath
    .split(path.sep)
    .map(encodeURIComponent)
    .join('/')}`
  const frontMatter = `---\ntitle: ${JSON.stringify(title)}\nsidebarTitle: ${JSON.stringify(title)}\nsourceFileUrl: ${JSON.stringify(sourceFileUrl)}\n---\n\n`
  await writeContent(destinationRelative, frontMatter + transformed)

  markdownCount += 1
  addSidebarEntry(destinationRelative, title)
  sourceMap[destinationRelative.split(path.sep).join('/')] = relativePath
    .split(path.sep)
    .join('/')
}

await writeContent(
  '_meta.js',
  `export default ${JSON.stringify(folderTitles, null, 2)}\n`,
)

await writeIfChanged(
  path.join(siteDir, '.content-map.json'),
  `${JSON.stringify({ markdownCount, sourceMap }, null, 2)}\n`
)

await pruneGenerated(contentDir)

console.log(`已生成 ${markdownCount} 篇 Nextra 文档。`)
