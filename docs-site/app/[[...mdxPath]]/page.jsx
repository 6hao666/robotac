import { generateStaticParamsFor, importPage } from 'nextra/pages'
import { useMDXComponents as getMDXComponents } from '../../mdx-components'

export const dynamic = 'force-static'
export const generateStaticParams = generateStaticParamsFor('mdxPath')

export async function generateMetadata({ params: paramsPromise }) {
  const params = await paramsPromise
  const { metadata } = await importPage(params.mdxPath)
  return metadata
}

const Wrapper = getMDXComponents().wrapper

export default async function Page(props) {
  const params = await props.params
  const { default: MDXContent, toc, metadata, sourceCode } = await importPage(
    params.mdxPath
  )

  return (
    <Wrapper
      toc={toc}
      metadata={{
        ...metadata,
        filePath: metadata.sourceFileUrl || metadata.filePath
      }}
      sourceCode={sourceCode}
    >
      <MDXContent {...props} params={params} />
    </Wrapper>
  )
}
