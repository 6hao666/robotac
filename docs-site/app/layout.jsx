import { Footer, LastUpdated, Layout, Navbar } from 'nextra-theme-docs'
import { Head, Search } from 'nextra/components'
import { getPageMap } from 'nextra/page-map'
import 'nextra-theme-docs/style.css'
import './site.css'

export const metadata = {
  metadataBase: new URL('https://docs.yundrone.cn/robotac/'),
  title: {
    default: 'Robotac 文档',
    template: '%s - Robotac 文档'
  },
  description: 'Robotac ROS Noetic 中文技术资料与编号教程',
  applicationName: 'Robotac 文档',
  generator: 'Nextra',
  icons: {
    icon: '/robotac/favicon.svg'
  }
}

const navbar = (
  <Navbar
    logo={
      <span className="robotac-brand">
        <img src="/robotac/yundrone-logo.svg" alt="YunDrone" />
        <span className="robotac-brand-divider" aria-hidden="true" />
        <strong>Robotac 文档</strong>
      </span>
    }
    projectLink="https://github.com/YunDrone-Team/robotac"
  />
)

const footer = (
  <Footer>YunDrone Robotac · 中文技术资料</Footer>
)

export default async function RootLayout({ children }) {
  return (
    <html lang="zh-CN" dir="ltr" suppressHydrationWarning>
      <Head faviconGlyph="R" />
      <body>
        <Layout
          navbar={navbar}
          footer={footer}
          pageMap={await getPageMap()}
          docsRepositoryBase="https://github.com/YunDrone-Team/robotac/tree/main/docs"
          editLink="在 GitHub 查看本页"
          feedback={{ content: '问题反馈', labels: 'feedback' }}
          copyPageButton={false}
          lastUpdated={<LastUpdated locale="zh-CN">最后更新于</LastUpdated>}
          sidebar={{ defaultMenuCollapseLevel: 1, toggleButton: true }}
          toc={{ title: '本页目录', backToTop: '返回顶部' }}
          search={
            <Search
              placeholder="搜索文档..."
              emptyResult="没有找到相关内容。"
              errorText="搜索索引加载失败。"
              loading="正在加载..."
            />
          }
          navigation={{ prev: true, next: true }}
          themeSwitch={{ dark: '深色', light: '浅色', system: '跟随系统' }}
        >
          {children}
        </Layout>
      </body>
    </html>
  )
}
