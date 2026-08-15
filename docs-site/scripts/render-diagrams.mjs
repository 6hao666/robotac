import crypto from 'node:crypto'
import { execFileSync } from 'node:child_process'
import fs from 'node:fs/promises'
import os from 'node:os'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const requiredVersion = '1.2026.6'
const expectedDiagramCount = 22
const requiredBackground = '#F4F6F8'
const scriptDir = path.dirname(fileURLToPath(import.meta.url))
const workspace = path.resolve(scriptDir, '../..')
const sourceDir = path.join(workspace, 'docs/diagrams')
const assetDir = path.join(workspace, 'docs/assets/plantuml')
const manifestPath = path.join(sourceDir, 'manifest.json')
const plantuml = process.env.DOCS_PLANTUML_BIN || 'plantuml'
const checkOnly = process.argv.includes('--check')
const safeEnvironment = {
  ...process.env,
  PLANTUML_SECURITY_PROFILE: 'SANDBOX'
}

function digest(content) {
  return crypto.createHash('sha256').update(content).digest('hex')
}

function fail(message) {
  throw new Error(`PlantUML 检查失败：${message}`)
}

async function readJson(filename, fallback) {
  try {
    return JSON.parse(await fs.readFile(filename, 'utf8'))
  } catch (error) {
    if (error.code === 'ENOENT' || error instanceof SyntaxError) return fallback
    throw error
  }
}

async function writeIfChanged(filename, content) {
  try {
    if (Buffer.compare(await fs.readFile(filename), content) === 0) return false
  } catch (error) {
    if (error.code !== 'ENOENT') throw error
  }
  await fs.mkdir(path.dirname(filename), { recursive: true })
  await fs.writeFile(filename, content)
  return true
}

function verifyTools() {
  let version
  try {
    version = execFileSync(plantuml, ['-version'], {
      encoding: 'utf8',
      env: safeEnvironment,
      stdio: ['ignore', 'pipe', 'pipe']
    })
  } catch {
    fail(`无法执行 ${plantuml}，请安装 PlantUML ${requiredVersion}`)
  }
  if (!version.includes(`PlantUML version ${requiredVersion}`)) {
    fail(`需要 PlantUML ${requiredVersion}，当前输出为 ${version.split('\n')[0]}`)
  }
  for (const [command, args] of [
    ['java', ['-version']],
    ['dot', ['-V']]
  ]) {
    try {
      execFileSync(command, args, { stdio: 'ignore' })
    } catch {
      fail(`缺少 ${command}`)
    }
  }
}

function verifySource(name, source) {
  if (!/^@startuml\s*$/m.test(source) || !/^@enduml\s*$/m.test(source)) {
    fail(`${name} 必须包含一组 @startuml/@enduml`)
  }
  const forbidden = [
    [/^\s*!(?:include|include_many|include_once|import)\b/im, 'include/import'],
    [/^\s*!theme\b.*\bfrom\b/im, '外部 theme'],
    [/%(?:getenv|load_json|load_yaml|load_xml|load_csv|file_exists)\s*\(/i, '外部数据函数'],
    [/(?:https?|file):\/\//i, '外部 URL']
  ]
  for (const [pattern, label] of forbidden) {
    if (pattern.test(source)) fail(`${name} 包含禁止的 ${label}`)
  }
  if (!new RegExp(`^\\s*skinparam\\s+backgroundColor\\s+${requiredBackground}\\s*$`, 'im').test(source)) {
    fail(`${name} 必须使用统一灰白背景 ${requiredBackground}`)
  }
}

function verifySvg(name, svg) {
  const text = svg.toString('utf8')
  if (svg.length < 300 || !text.includes('<svg') || !text.includes('viewBox=')) {
    fail(`${name} 不是有效的响应式 SVG`)
  }
  if (/<script\b|javascript:|<(?:foreignObject|iframe)\b/i.test(text)) {
    fail(`${name} 包含可执行内容`)
  }
  if (/(?:href|xlink:href)=["'](?:https?:|file:)/i.test(text)) {
    fail(`${name} 包含外部资源引用`)
  }
  if (/\/(?:Users|home)\/|\b(?:10|127|192\.168)\.(?:\d{1,3}\.){2}\d{1,3}\b/.test(text)) {
    fail(`${name} 包含机器相关路径或地址`)
  }
  const rootSvg = text.match(/<svg\b[^>]*>/i)?.[0]
  const width = Number(rootSvg?.match(/\bwidth="([\d.]+)px"/i)?.[1])
  const height = Number(rootSvg?.match(/\bheight="([\d.]+)px"/i)?.[1])
  if (!Number.isFinite(width) || !Number.isFinite(height)) {
    fail(`${name} 缺少可校验的 SVG 宽高`)
  }
  if (!(width > height)) fail(`${name} 必须为横向布局，当前为 ${width} x ${height}`)
}

function runPlantuml(args) {
  execFileSync(plantuml, args, {
    env: safeEnvironment,
    stdio: 'inherit'
  })
}

verifyTools()
await fs.mkdir(sourceDir, { recursive: true })
await fs.mkdir(assetDir, { recursive: true })

const sourceNames = (await fs.readdir(sourceDir))
  .filter(name => name.endsWith('.puml'))
  .sort()
if (sourceNames.length !== expectedDiagramCount) {
  fail(`预期 ${expectedDiagramCount} 个 .puml，实际 ${sourceNames.length} 个`)
}

const sources = new Map()
for (const name of sourceNames) {
  const content = await fs.readFile(path.join(sourceDir, name), 'utf8')
  verifySource(name, content)
  sources.set(name, { content, sourceSha256: digest(content) })
}

const previous = await readJson(manifestPath, { files: {} })
const changed = []
for (const [name, source] of sources) {
  const stem = path.basename(name, '.puml')
  const svgName = `${stem}.svg`
  let svg
  try {
    svg = await fs.readFile(path.join(assetDir, svgName))
  } catch (error) {
    if (error.code !== 'ENOENT') throw error
  }
  const entry = previous.files?.[name]
  if (
    !svg ||
    entry?.sourceSha256 !== source.sourceSha256 ||
    entry?.svgSha256 !== digest(svg) ||
    previous.renderer !== `PlantUML ${requiredVersion}`
  ) {
    changed.push(name)
  } else {
    verifySvg(svgName, svg)
  }
}

const svgNames = (await fs.readdir(assetDir))
  .filter(name => name.endsWith('.svg'))
  .sort()
const expectedSvgNames = sourceNames.map(name => name.replace(/\.puml$/, '.svg'))
const staleSvgNames = svgNames.filter(name => !expectedSvgNames.includes(name))

if (checkOnly) {
  runPlantuml(['--check-syntax', ...sourceNames.map(name => path.join(sourceDir, name))])
  if (changed.length || staleSvgNames.length) {
    fail('图表源文件与 SVG 不同步，请运行 ./tools/docs-site.sh diagrams')
  }
  console.log(`PlantUML 检查通过：${sourceNames.length} 个图表。`)
  process.exit(0)
}

if (changed.length) {
  const temporary = await fs.mkdtemp(path.join(os.tmpdir(), 'robotac-plantuml-'))
  try {
    runPlantuml([
      '--format',
      'svg',
      '--charset',
      'UTF-8',
      '--no-metadata',
      '--check-before-run',
      '--output-dir',
      temporary,
      ...changed.map(name => path.join(sourceDir, name))
    ])
    for (const name of changed) {
      const svgName = name.replace(/\.puml$/, '.svg')
      const svg = await fs.readFile(path.join(temporary, svgName))
      verifySvg(svgName, svg)
      await writeIfChanged(path.join(assetDir, svgName), svg)
    }
  } finally {
    await fs.rm(temporary, { force: true, recursive: true })
  }
}

for (const name of staleSvgNames) {
  await fs.rm(path.join(assetDir, name))
}

const files = {}
for (const [name, source] of sources) {
  const svgName = name.replace(/\.puml$/, '.svg')
  const svg = await fs.readFile(path.join(assetDir, svgName))
  verifySvg(svgName, svg)
  files[name] = {
    sourceSha256: source.sourceSha256,
    svg: `../assets/plantuml/${svgName}`,
    svgSha256: digest(svg)
  }
}
const manifest = Buffer.from(
  `${JSON.stringify({ schemaVersion: 1, renderer: `PlantUML ${requiredVersion}`, files }, null, 2)}\n`
)
await writeIfChanged(manifestPath, manifest)
console.log(`PlantUML 已同步：${changed.length} 个更新，${sourceNames.length} 个总计。`)
