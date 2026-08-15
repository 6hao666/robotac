import nextra from 'nextra'
import { fileURLToPath } from 'node:url'

const withNextra = nextra({
  contentDirBasePath: '/',
  defaultShowCopyCode: true,
  search: {
    codeblocks: false
  }
})

export default withNextra({
  output: 'export',
  basePath: '/robotac',
  assetPrefix: '/robotac',
  trailingSlash: true,
  images: {
    unoptimized: true
  },
  outputFileTracingRoot: fileURLToPath(new URL('.', import.meta.url)),
  poweredByHeader: false,
  reactStrictMode: true
})
