import nextra from 'nextra'
import { fileURLToPath } from 'node:url'
import { rehypeMermaidPre } from './scripts/rehype-mermaid-pre.mjs'

const withNextra = nextra({
  contentDirBasePath: '/',
  defaultShowCopyCode: true,
  search: {
    codeblocks: false
  },
  mdxOptions: {
    rehypePlugins: [rehypeMermaidPre]
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
