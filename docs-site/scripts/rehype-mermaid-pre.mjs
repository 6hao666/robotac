import { visit } from 'unist-util-visit'

export function rehypeMermaidPre() {
  return tree => {
    visit(tree, 'element', node => {
      if (node.tagName !== 'pre' || node.children?.length !== 1) return
      const code = node.children[0]
      if (code.type !== 'element' || code.tagName !== 'code') return
      const classes = code.properties?.className || []
      if (!classes.includes('language-mermaid')) return

      node.properties = {
        ...node.properties,
        className: ['mermaid']
      }
      node.children = code.children
    })
  }
}
