import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { load } from 'cheerio'

const scriptDir = path.dirname(fileURLToPath(import.meta.url))
const siteDir = path.resolve(scriptDir, '..')
const workspace = path.resolve(siteDir, '..')
const docsDir = path.join(workspace, 'docs')
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
if (markdownFiles.length !== 24) {
  fail(`预期 24 篇 Markdown，实际 ${markdownFiles.length} 篇`)
}

for (const source of markdownFiles) {
  const route = sourceRoute(source)
  const htmlPath = path.join(outDir, route, 'index.html')
  if (!(await exists(htmlPath))) {
    fail(`${path.relative(workspace, source)} 未生成 ${path.relative(siteDir, htmlPath)}`)
  }
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
for (const htmlFile of htmlFiles) {
  const html = await fs.readFile(htmlFile, 'utf8')
  const $ = load(html)
  const relativeHtml = path.relative(outDir, htmlFile).split(path.sep).join('/')
  const pagePath =
    relativeHtml === 'index.html'
      ? `${basePath}/`
      : `${basePath}/${relativeHtml.replace(/index\.html$/, '')}`
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
}

for (const mermaidPage of [
  '01-project-overview/index.html',
  '05-competition-examples/index.html'
]) {
  const html = await fs.readFile(path.join(outDir, mermaidPage), 'utf8')
  if (!html.includes('Mermaid') || !html.includes('chart')) {
    fail(`${mermaidPage} 缺少 Mermaid 客户端组件`)
  }
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
