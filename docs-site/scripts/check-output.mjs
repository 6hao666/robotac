import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { load } from 'cheerio'

const scriptDir = path.dirname(fileURLToPath(import.meta.url))
const siteDir = path.resolve(scriptDir, '..')
const workspace = path.resolve(siteDir, '..')
const docsDir = path.join(workspace, 'docs')
const diagramsDir = path.join(docsDir, 'diagrams')
const diagramAssetsDir = path.join(docsDir, 'assets/plantuml')
const outDir = path.join(siteDir, 'out')
const basePath = '/robotac'

async function walk(directory, extension) {
  const entries = await fs.readdir(directory, { withFileTypes: true })
  const files = []
  for (const entry of entries) {
    const absolute = path.join(directory, entry.name)
    if (entry.isDirectory()) files.push(...(await walk(absolute, extension)))
    else if (!extension || absolute.endsWith(extension)) files.push(absolute)
  }
  return files.sort()
}

function sourceRoute(sourcePath) {
  const relative = path.relative(docsDir, sourcePath).split(path.sep).join('/')
  if (relative.toLowerCase() === 'readme.md') return ''
  if (relative.toLowerCase().endsWith('/readme.md')) {
    return relative.slice(0, -'/README.md'.length)
  }
  return relative.replace(/\.md$/i, '')
}

function outputForPathname(pathname) {
  const clean = decodeURIComponent(pathname.split(/[?#]/, 1)[0])
  if (!clean.startsWith(`${basePath}/`) && clean !== basePath) return null
  const relative = clean.slice(basePath.length).replace(/^\//, '')
  if (!relative) return path.join(outDir, 'index.html')
  if (relative.endsWith('/')) return path.join(outDir, relative, 'index.html')
  if (path.extname(relative)) return path.join(outDir, relative)
  return path.join(outDir, relative, 'index.html')
}

async function exists(filename) {
  try {
    await fs.access(filename)
    return true
  } catch {
    return false
  }
}

function fail(message) {
  console.error(`文档站检查失败：${message}`)
  process.exitCode = 1
}

const markdownFiles = await walk(docsDir, '.md')
if (markdownFiles.length !== 23) {
  fail(`预期 23 篇 Markdown，实际 ${markdownFiles.length} 篇`)
}

const sourceDiagramAssets = new Set()
for (const source of markdownFiles) {
  const markdown = await fs.readFile(source, 'utf8')
  if (/```mermaid\b/i.test(markdown)) fail(`${path.relative(workspace, source)} 仍包含 Mermaid`)
  const images = [...markdown.matchAll(/!\[[^\]]+\]\(([^\s)]+)\s+"PlantUML[^"]*"\)/g)]
  if (images.length !== 1) {
    fail(`${path.relative(workspace, source)} 的 PlantUML 图片引用数量为 ${images.length}`)
  } else {
    const asset = path.resolve(path.dirname(source), images[0][1])
    sourceDiagramAssets.add(asset)
    if (!(await exists(asset))) fail(`${path.relative(workspace, asset)} 不存在`)
  }
  const route = sourceRoute(source)
  const htmlPath = path.join(outDir, route, 'index.html')
  if (!(await exists(htmlPath))) {
    fail(`${path.relative(workspace, source)} 未生成 ${path.relative(siteDir, htmlPath)}`)
  }
}

const pumlFiles = await walk(diagramsDir, '.puml')
const svgFiles = await walk(diagramAssetsDir, '.svg')
if (pumlFiles.length !== 23 || svgFiles.length !== 23 || sourceDiagramAssets.size !== 23) {
  fail(`预期 23 组独立图表，实际 puml=${pumlFiles.length} svg=${svgFiles.length} 引用=${sourceDiagramAssets.size}`)
}
if (await exists(path.join(outDir, '11-migration', 'index.html'))) {
  fail('旧版迁移页面仍存在于静态导出')
}

for (const required of [
  '_next/static',
  '_pagefind/pagefind.js',
  'release.json',
  'index.html',
  'tutorials/index.html'
]) {
  if (!(await exists(path.join(outDir, required)))) fail(`缺少 ${required}`)
}

const htmlFiles = await walk(outDir, '.html')
const outputDiagramAssets = new Set()
for (const htmlFile of htmlFiles) {
  const html = await fs.readFile(htmlFile, 'utf8')
  const $ = load(html)
  const relativeHtml = path.relative(outDir, htmlFile).split(path.sep).join('/')
  const pagePath =
    relativeHtml === 'index.html'
      ? `${basePath}/`
      : `${basePath}/${relativeHtml.replace(/index\.html$/, '')}`
  if (html.includes('旧版迁移说明')) fail(`${relativeHtml} 仍包含旧版迁移说明`)
  for (const element of $('a[href]').toArray()) {
    const href = $(element).attr('href')
    if (!href || /^(?:#|mailto:|tel:|https?:\/\/)/i.test(href)) continue
    let pathname
    try {
      pathname = new URL(href, `https://docs.yundrone.cn${pagePath}`).pathname
    } catch {
      fail(`${path.relative(outDir, htmlFile)} 包含无法解析的链接 ${href}`)
      continue
    }
    const target = outputForPathname(pathname)
    if (target && !(await exists(target))) {
      fail(`${path.relative(outDir, htmlFile)} -> ${href} 没有对应静态文件`)
    }
  }
  if (contentMapRoute(relativeHtml)) {
    const diagrams = $('img[alt^="PlantUML："]')
    if (diagrams.length !== 1) fail(`${relativeHtml} 的 PlantUML 图数量为 ${diagrams.length}`)
    const source = diagrams.first().attr('src')
    if (source) {
      const pathname = new URL(source, `https://docs.yundrone.cn${pagePath}`).pathname
      const target = outputForPathname(pathname)
      if (!target || !(await exists(target))) fail(`${relativeHtml} 的 SVG ${source} 不存在`)
      else outputDiagramAssets.add(target)
    }
  }
}

if (outputDiagramAssets.size !== 23) {
  fail(`静态导出只引用了 ${outputDiagramAssets.size} 个独立 PlantUML SVG`)
}

for (const [outputPage, sourceFile] of [
  ['index.html', 'README.md'],
  ['tutorials/index.html', 'tutorials/README.md'],
  ['01-project-overview/index.html', '01-project-overview.md']
]) {
  const html = await fs.readFile(path.join(outDir, outputPage), 'utf8')
  const expected = `https://github.com/YunDrone-Team/robotac/blob/main/docs/${sourceFile}`
  if (!html.includes(expected)) fail(`${outputPage} 的 GitHub 源文件链接错误`)
}

const home = await fs.readFile(path.join(outDir, 'index.html'), 'utf8')
if (!home.includes('Robotac 文档索引')) fail('首页缺少中文文档标题')

if (!process.exitCode) {
  console.log(`文档站检查通过：${markdownFiles.length} 篇文档，${htmlFiles.length} 个 HTML 文件。`)
}

function contentMapRoute(relativeHtml) {
  if (relativeHtml === 'index.html') return true
  const route = relativeHtml.replace(/index\.html$/, '').replace(/\/$/, '')
  return markdownFiles.some(source => sourceRoute(source) === route)
}
