import { Button } from "@/components/ui/button";
import { Link } from "wouter";
import { BookOpen, Code, Rocket, HelpCircle, Download, Zap } from "lucide-react";
import { APP_LOGO, APP_TITLE } from "@/const";

export default function Home() {
  return (
    <div className="min-h-screen flex flex-col">
      {/* Header */}
      <header className="sticky top-0 z-50 w-full border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60">
        <div className="container flex h-16 items-center justify-between">
          <div className="flex items-center gap-2">
            <img src={APP_LOGO} alt={APP_TITLE} className="h-8" />
            <span className="font-bold text-lg">{APP_TITLE}</span>
          </div>
          <nav className="flex items-center gap-4">
            <Link href="/docs/about/overview">
              <Button variant="ghost">文档</Button>
            </Link>
            <Link href="/docs/quickstart/unboxing">
              <Button>快速开始</Button>
            </Link>
          </nav>
        </div>
      </header>

      {/* Hero Section */}
      <section className="container py-20 md:py-32">
        <div className="text-center max-w-3xl mx-auto">
          <h1 className="text-4xl md:text-6xl font-bold mb-6 bg-gradient-to-r from-primary to-primary/60 bg-clip-text text-transparent">
            XHAND1 全驱灵巧手
          </h1>
          <p className="text-xl md:text-2xl text-muted-foreground mb-8">
            12自由度 · 270度触觉感知 · 80N抓握力
          </p>
          <p className="text-lg text-muted-foreground mb-10">
            星动量科技推出的高仿生灵巧手，为人形机器人和工业自动化提供精密操作能力
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link href="/docs/quickstart/unboxing">
              <Button size="lg" className="w-full sm:w-auto">
                <Rocket className="mr-2 h-5 w-5" />
                快速开始
              </Button>
            </Link>
            <Link href="/docs/about/overview">
              <Button size="lg" variant="outline" className="w-full sm:w-auto">
                <BookOpen className="mr-2 h-5 w-5" />
                查看文档
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="container py-16 bg-accent/30">
        <h2 className="text-3xl font-bold text-center mb-12">核心特性</h2>
        <div className="grid md:grid-cols-3 gap-8">
          <div className="bg-background p-6 rounded-lg border">
            <div className="h-12 w-12 rounded-lg bg-primary/10 flex items-center justify-center mb-4">
              <Zap className="h-6 w-6 text-primary" />
            </div>
            <h3 className="text-xl font-semibold mb-2">全驱设计</h3>
            <p className="text-muted-foreground">
              12个主动自由度，每个关节独立驱动，实现精确控制和复杂动作
            </p>
          </div>
          <div className="bg-background p-6 rounded-lg border">
            <div className="h-12 w-12 rounded-lg bg-primary/10 flex items-center justify-center mb-4">
              <BookOpen className="h-6 w-6 text-primary" />
            </div>
            <h3 className="text-xl font-semibold mb-2">高分辨率触觉</h3>
            <p className="text-muted-foreground">
              16个触觉传感器，270度环绕式覆盖，提供实时接触力反馈
            </p>
          </div>
          <div className="bg-background p-6 rounded-lg border">
            <div className="h-12 w-12 rounded-lg bg-primary/10 flex items-center justify-center mb-4">
              <Code className="h-6 w-6 text-primary" />
            </div>
            <h3 className="text-xl font-semibold mb-2">强大抓握力</h3>
            <p className="text-muted-foreground">
              最大抓握力80N，额定负载1.5kg，满足各种操作任务需求
            </p>
          </div>
        </div>
      </section>

      {/* Quick Links Section */}
      <section className="container py-16">
        <h2 className="text-3xl font-bold text-center mb-12">快速导航</h2>
        <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-6">
          <Link href="/docs/about/overview">
            <div className="border rounded-lg p-6 hover:border-primary transition-colors cursor-pointer h-full">
              <BookOpen className="h-8 w-8 text-primary mb-4" />
              <h3 className="text-lg font-semibold mb-2">产品介绍</h3>
              <p className="text-sm text-muted-foreground">
                了解XHAND1的核心特性、技术规格和应用场景
              </p>
            </div>
          </Link>
          <Link href="/docs/quickstart/unboxing">
            <div className="border rounded-lg p-6 hover:border-primary transition-colors cursor-pointer h-full">
              <Rocket className="h-8 w-8 text-primary mb-4" />
              <h3 className="text-lg font-semibold mb-2">快速入门</h3>
              <p className="text-sm text-muted-foreground">
                从开箱到运行第一个程序，快速上手XHAND1
              </p>
            </div>
          </Link>
          <Link href="/docs/sdk/overview">
            <div className="border rounded-lg p-6 hover:border-primary transition-colors cursor-pointer h-full">
              <Code className="h-8 w-8 text-primary mb-4" />
              <h3 className="text-lg font-semibold mb-2">SDK开发</h3>
              <p className="text-sm text-muted-foreground">
                Python、C++、ROS多语言SDK，完整的API参考
              </p>
            </div>
          </Link>
          <Link href="/docs/faq">
            <div className="border rounded-lg p-6 hover:border-primary transition-colors cursor-pointer h-full">
              <HelpCircle className="h-8 w-8 text-primary mb-4" />
              <h3 className="text-lg font-semibold mb-2">常见问题</h3>
              <p className="text-sm text-muted-foreground">
                FAQ、故障排查、技术支持和下载中心
              </p>
            </div>
          </Link>
        </div>
      </section>

      {/* CTA Section */}
      <section className="container py-16 bg-primary/5">
        <div className="text-center max-w-2xl mx-auto">
          <h2 className="text-3xl font-bold mb-4">准备开始了吗？</h2>
          <p className="text-lg text-muted-foreground mb-8">
            查看快速入门指南，几分钟内让XHAND1运行起来
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link href="/docs/quickstart/hello-world">
              <Button size="lg">
                运行第一个程序
              </Button>
            </Link>
            <Link href="/docs/support/downloads">
              <Button size="lg" variant="outline">
                <Download className="mr-2 h-5 w-5" />
                下载SDK
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t py-8 mt-auto">
        <div className="container text-center text-sm text-muted-foreground">
          <p>© 2024 星动量科技. All rights reserved.</p>
          <p className="mt-2">
            <a href="mailto:support@xingdong.tech" className="hover:text-primary">
              support@xingdong.tech
            </a>
          </p>
        </div>
      </footer>
    </div>
  );
}
