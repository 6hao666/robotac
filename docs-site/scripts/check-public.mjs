import fs from 'node:fs/promises'
import path from 'node:path'
import { fileURLToPath } from 'node:url'
import { load } from 'cheerio'

const scriptDir = path.dirname(fileURLToPath(import.meta.url))
const siteDir = path.resolve(scriptDir, '..')
const timeout = 20_000

function fail(message) {
  throw new Error(`公网检查失败：${message}`)
}

async function request(url, options = {}) {
  return fetch(url, {
    redirect: options.redirect || 'follow',
    signal: AbortSignal.timeout(timeout)
  })
}

async function json(url) {
  const response = await request(url)
  if (!response.ok) fail(`${url} 返回 ${response.status}`)
  return response.json()
}

if (process.argv[2] === '--matches-local') {
  const publicBase = process.argv[3]?.replace(/\/$/, '')
  const localFile = process.argv[4]
  if (!publicBase || !localFile) process.exit(2)
  try {
    const [online, local] = await Promise.all([
      json(`${publicBase}/robotac/release.json`),
      fs.readFile(localFile, 'utf8').then(JSON.parse)
    ])
    const same =
      online.inputDigest === local.inputDigest &&
      online.gitSha === local.gitSha &&
      online.dirty === local.dirty
    process.exit(same ? 0 : 1)
  } catch {
    process.exit(1)
  }
}

const basicCheck = process.argv[2] === '--basic'
const argumentOffset = basicCheck ? 1 : 0
const publicBase = process.argv[2 + argumentOffset]?.replace(/\/$/, '')
const expectedRelease = process.argv[3 + argumentOffset] || ''
if (!publicBase) fail('缺少公网站点地址')
const siteBase = `${publicBase}/robotac/`

for (const [url, status, destination] of [
  [`${publicBase.replace(/^https:/, 'http:')}/robotac/`, 301, siteBase],
  [`${publicBase}/`, 302, siteBase]
]) {
  const response = await request(url, { redirect: 'manual' })
  if (response.status !== status) fail(`${url} 应返回 ${status}，实际 ${response.status}`)
  const location = response.headers.get('location')
  if (!location || new URL(location, url).href !== destination) {
    fail(`${url} 跳转目标错误：${location || '无 Location'}`)
  }
}

const release = await json(`${siteBase}release.json`)
if (expectedRelease && release.releaseId !== expectedRelease) {
  fail(`releaseId 为 ${release.releaseId}，预期 ${expectedRelease}`)
}

if (basicCheck) {
  for (const route of ['', 'tutorials/', '_pagefind/pagefind.js']) {
    const response = await request(new URL(route, siteBase))
    if (!response.ok) fail(`${route || '首页'} 返回 ${response.status}`)
  }
  console.log(`公网基础检查通过：版本 ${release.releaseId}。`)
  process.exit(0)
}

const contentMap = JSON.parse(
  await fs.readFile(path.join(siteDir, '.content-map.json'), 'utf8')
)
if (contentMap.markdownCount !== 23) fail(`内容映射页数为 ${contentMap.markdownCount}`)

const diagramUrls = new Set()
for (const outputPath of Object.keys(contentMap.sourceMap).sort()) {
  const route = outputPath === 'index.md' ? '' : outputPath.replace(/index\.md$/, '').replace(/\.md$/, '/')
  const pageUrl = new URL(route, siteBase)
  const response = await request(pageUrl)
  if (!response.ok) fail(`${pageUrl.href} 返回 ${response.status}`)
  const html = await response.text()
  if (html.includes('旧版迁移说明')) fail(`${pageUrl.href} 仍包含旧版迁移说明`)
  const $ = load(html)
  const images = $('img[alt^="PlantUML："]')
  if (images.length !== 1) fail(`${pageUrl.href} 的 PlantUML 图数量为 ${images.length}`)
  const source = images.first().attr('src')
  if (!source) fail(`${pageUrl.href} 的 PlantUML 图没有 src`)
  diagramUrls.add(new URL(source, pageUrl).href)
}

if (diagramUrls.size !== 23) fail(`公网页面只引用了 ${diagramUrls.size} 个独立 SVG`)
for (const url of diagramUrls) {
  const response = await request(url)
  if (!response.ok) fail(`${url} 返回 ${response.status}`)
  const svg = await response.text()
  if (!svg.includes('<svg') || !svg.includes('viewBox=')) fail(`${url} 不是有效 SVG`)
}

for (const required of ['_pagefind/pagefind.js']) {
  const response = await request(new URL(required, siteBase))
  if (!response.ok) fail(`${required} 返回 ${response.status}`)
}

const removed = await request(new URL('11-migration/', siteBase), { redirect: 'manual' })
if (removed.status !== 404) fail(`旧迁移路由应返回 404，实际 ${removed.status}`)

console.log(`公网检查通过：23 页、23 个 PlantUML SVG，版本 ${release.releaseId}。`)
