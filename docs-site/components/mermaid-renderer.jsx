'use client'

import { usePathname } from 'next/navigation'
import { useEffect } from 'react'

export function MermaidRenderer() {
  const pathname = usePathname()

  useEffect(() => {
    let cancelled = false

    async function renderDiagrams() {
      const nodes = Array.from(document.querySelectorAll('pre.mermaid')).filter(
        node => !node.dataset.processed
      )
      if (nodes.length === 0) return

      const { default: mermaid } = await import('mermaid')
      if (cancelled) return

      mermaid.initialize({
        startOnLoad: false,
        securityLevel: 'strict',
        theme: 'neutral',
        fontFamily: 'inherit'
      })
      await mermaid.run({ nodes, suppressErrors: false })
    }

    void renderDiagrams().catch(error => {
      console.error('Mermaid rendering failed', error)
    })

    return () => {
      cancelled = true
    }
  }, [pathname])

  return null
}
