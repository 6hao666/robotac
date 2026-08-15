import fs from 'node:fs/promises'
import http from 'node:http'
import path from 'node:path'
import { fileURLToPath } from 'node:url'

const scriptDir = path.dirname(fileURLToPath(import.meta.url))
const outDir = path.resolve(scriptDir, '..', 'out')
const basePath = '/robotac'
const host = process.env.DOCS_HOST || '127.0.0.1'
const port = Number(process.env.DOCS_PORT || 3000)

const mimeTypes = {
  '.css': 'text/css; charset=utf-8',
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.wasm': 'application/wasm'
}

async function resolveFile(pathname) {
  let relative = decodeURIComponent(pathname.slice(basePath.length))
  relative = relative.replace(/^\/+/, '')
  let filename = path.resolve(outDir, relative || 'index.html')
  if (filename !== outDir && !filename.startsWith(`${outDir}${path.sep}`)) return null

  try {
    const stat = await fs.stat(filename)
    if (stat.isDirectory()) filename = path.join(filename, 'index.html')
  } catch (error) {
    if (error.code !== 'ENOENT' || path.extname(filename)) throw error
    filename = path.join(filename, 'index.html')
  }
  return filename
}

const server = http.createServer(async (request, response) => {
  try {
    const { pathname } = new URL(request.url || '/', `http://${host}:${port}`)
    if (pathname === basePath) {
      response.writeHead(308, { Location: `${basePath}/` })
      response.end()
      return
    }
    if (!pathname.startsWith(`${basePath}/`)) {
      response.writeHead(404)
      response.end('Not found')
      return
    }

    const filename = await resolveFile(pathname)
    const body = filename && (await fs.readFile(filename))
    response.writeHead(200, {
      'Cache-Control': 'no-cache',
      'Content-Type': mimeTypes[path.extname(filename)] || 'application/octet-stream'
    })
    response.end(request.method === 'HEAD' ? undefined : body)
  } catch (error) {
    if (error.code !== 'ENOENT') console.error(error)
    response.writeHead(404)
    response.end('Not found')
  }
})

server.listen(port, host, () => {
  console.log(`本地静态预览：http://${host}:${port}${basePath}/`)
})
