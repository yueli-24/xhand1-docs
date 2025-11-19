import { Button } from "@/components/ui/button";
import { Link } from "wouter";
import { BookOpen, Code, Rocket, HelpCircle, Download, Zap, Gamepad2, ChevronRight, Lightbulb, LifeBuoy } from "lucide-react";
import { APP_LOGO, APP_TITLE } from "@/const";

export default function Home() {
  return (
    <div className="min-h-screen flex flex-col bg-background relative overflow-hidden">
      {/* Grid Background */}
      <div className="absolute inset-0 bg-[linear-gradient(to_right,oklch(0.25_0.03_240_/_0.1)_1px,transparent_1px),linear-gradient(to_bottom,oklch(0.25_0.03_240_/_0.1)_1px,transparent_1px)] bg-[size:4rem_4rem] [mask-image:radial-gradient(ellipse_80%_50%_at_50%_0%,#000_70%,transparent_110%)]" />
      
      {/* Header */}
      <header className="sticky top-0 z-50 w-full border-b border-border/40 bg-background/80 backdrop-blur-xl">
        <div className="container flex h-20 items-center justify-between">
          <div className="flex items-center gap-3">
            <img src={APP_LOGO} alt={APP_TITLE} className="h-10" style={{width: '150px'}} />
            <div className="h-8 w-px bg-border/60" />
            <span className="font-semibold text-lg tracking-tight">{APP_TITLE}</span>
          </div>
          <nav className="flex items-center gap-6">
            <Link href="/docs/about/overview">
              <Button className="bg-primary hover:bg-primary/90 shadow-[0_0_20px_rgba(59,130,246,0.3)]">
                文档中心
                <ChevronRight className="ml-1 h-4 w-4" />
              </Button>
            </Link>
          </nav>
        </div>
      </header>

      {/* Hero Section */}
      <section className="container relative z-10 py-24 md:py-32 lg:py-40">
        <div className="text-center max-w-5xl mx-auto space-y-8">
          {/* Badge */}
          <div className="inline-flex items-center gap-2 px-4 py-2 rounded-full bg-accent/10 border border-accent/20 text-accent text-sm font-medium">
            <Zap className="h-4 w-4" />
            <span>高自由度 · 高灵巧度 · 高感知</span>
          </div>
          
          {/* Title */}
          <h1 className="text-5xl md:text-7xl lg:text-8xl font-bold tracking-tight">
            <span className="bg-gradient-to-br from-foreground via-foreground to-foreground/60 bg-clip-text text-transparent">
              XHAND1
            </span>
            <br />
            <span className="bg-gradient-to-r from-primary via-accent to-primary bg-clip-text text-transparent">
              全直驱灵巧手
            </span>
          </h1>
          
          {/* Specs */}
          <div className="flex flex-wrap items-center justify-center gap-6 text-lg md:text-xl text-muted-foreground">
            <div className="flex items-center gap-2">
              <div className="h-1.5 w-1.5 rounded-full bg-primary animate-pulse" />
              <span>12个独立自由度</span>
            </div>
            <div className="h-4 w-px bg-border" />
            <div className="flex items-center gap-2">
              <div className="h-1.5 w-1.5 rounded-full bg-accent animate-pulse" />
              <span>高分辨率触觉感知</span>
            </div>
            <div className="h-4 w-px bg-border" />
            <div className="flex items-center gap-2">
              <div className="h-1.5 w-1.5 rounded-full bg-primary animate-pulse" />
              <span>支持反驱，可靠性高</span>
            </div>
          </div>
          
          {/* Description */}
          <p className="text-lg md:text-xl text-muted-foreground max-w-3xl mx-auto leading-relaxed">
            专为 强化学习、模仿学习、控制研究 打造的旗舰级机器人灵巧手。
          </p>
          
          {/* CTA Buttons */}
          <div className="flex flex-col sm:flex-row gap-4 justify-center pt-4">
            <Link href="/docs/quickstart/unboxing">
              <Button size="lg" className="w-full sm:w-auto bg-primary hover:bg-primary/90 shadow-[0_0_30px_rgba(59,130,246,0.4)] hover:shadow-[0_0_40px_rgba(59,130,246,0.5)] transition-all">
                <Rocket className="mr-2 h-5 w-5" />
                快速开始
              </Button>
            </Link>
            <Link href="/docs/about/overview">
              <Button size="lg" variant="outline" className="w-full sm:w-auto border-border/60 hover:border-primary/50 hover:bg-primary/5">
                <BookOpen className="mr-2 h-5 w-5" />
                查看文档中心
              </Button>
            </Link>
          </div>
        </div>
      </section>

      {/* Features Section */}
      <section className="container relative z-10 py-20">
        <h2 className="text-3xl md:text-4xl font-bold text-center mb-16">
          <span className="bg-gradient-to-r from-foreground to-foreground/60 bg-clip-text text-transparent">
            核心特性
          </span>
        </h2>
        <div className="grid md:grid-cols-3 gap-8 max-w-6xl mx-auto">
          {/* Feature 1 */}
          <div className="group relative p-8 rounded-xl bg-card/50 border border-border/40 hover:border-primary/50 transition-all hover:shadow-[0_0_30px_rgba(59,130,246,0.15)]">
            <div className="absolute inset-0 bg-gradient-to-br from-primary/5 to-transparent rounded-xl opacity-0 group-hover:opacity-100 transition-opacity" />
            <div className="relative space-y-4">
              <div className="h-12 w-12 rounded-lg bg-primary/10 flex items-center justify-center">
                <Zap className="h-6 w-6 text-primary" />
              </div>
              <h3 className="text-xl font-semibold">全驱设计</h3>
              <p className="text-muted-foreground leading-relaxed">
                12 个主动自由度，全关节独立驱动。每个关节由独立执行机构精准控制，支持拇指与四指的灵活对指及高精度操作。
              </p>
            </div>
          </div>

          {/* Feature 2 */}
          <div className="group relative p-8 rounded-xl bg-card/50 border border-border/40 hover:border-accent/50 transition-all hover:shadow-[0_0_30px_rgba(16,185,129,0.15)]">
            <div className="absolute inset-0 bg-gradient-to-br from-accent/5 to-transparent rounded-xl opacity-0 group-hover:opacity-100 transition-opacity" />
            <div className="relative space-y-4">
              <div className="h-12 w-12 rounded-lg bg-accent/10 flex items-center justify-center">
                <BookOpen className="h-6 w-6 text-accent" />
              </div>
              <h3 className="text-xl font-semibold">高密度触觉阵列</h3>
              <p className="text-muted-foreground leading-relaxed">
                每指 120 点高分辨率传感点，270° 环绕覆盖，支持 三维力感知（含切向 XY） 与 温度检测，实现类人级触摸与接触理解。
              </p>
            </div>
          </div>

          {/* Feature 3 */}
          <div className="group relative p-8 rounded-xl bg-card/50 border border-border/40 hover:border-primary/50 transition-all hover:shadow-[0_0_30px_rgba(59,130,246,0.15)]">
            <div className="absolute inset-0 bg-gradient-to-br from-primary/5 to-transparent rounded-xl opacity-0 group-hover:opacity-100 transition-opacity" />
            <div className="relative space-y-4">
              <div className="h-12 w-12 rounded-lg bg-primary/10 flex items-center justify-center">
                <Code className="h-6 w-6 text-primary" />
              </div>
              <h3 className="text-xl font-semibold">高保真遥操作</h3>
              <p className="text-muted-foreground leading-relaxed">
                仿人手设计，搭配自研 Apple Vision Pro / Meta Quest / Manus 手套遥操作软件，实现自然的人手 → 机器人手映射，轻松采集高质量示教数据。
              </p>
            </div>
          </div>
        </div>
      </section>

      {/* Quick Navigation */}
      <section className="container relative z-10 py-20">
        <h2 className="text-3xl md:text-4xl font-bold text-center mb-16">
          <span className="bg-gradient-to-r from-foreground to-foreground/60 bg-clip-text text-transparent">
            快速导航
          </span>
        </h2>
        <div className="grid sm:grid-cols-2 lg:grid-cols-3 gap-6 max-w-6xl mx-auto">
          {/* Nav Card 1 */}
          <Link href="/docs/about/overview">
            <div className="group p-6 rounded-xl bg-card/30 border border-border/40 hover:border-primary/50 transition-all hover:shadow-[0_0_20px_rgba(59,130,246,0.15)] cursor-pointer h-full">
              <div className="space-y-3">
                <div className="h-10 w-10 rounded-lg bg-primary/10 flex items-center justify-center group-hover:bg-primary/20 transition-colors">
                  <BookOpen className="h-5 w-5 text-primary" />
                </div>
                <h3 className="text-lg font-semibold group-hover:text-primary transition-colors">产品介绍</h3>
                <p className="text-sm text-muted-foreground">了解XHAND1的核心特性、技术规格和应用场景</p>
              </div>
            </div>
          </Link>

          {/* Nav Card 2 */}
          <Link href="/docs/quickstart/unboxing">
            <div className="group p-6 rounded-xl bg-card/30 border border-border/40 hover:border-accent/50 transition-all hover:shadow-[0_0_20px_rgba(16,185,129,0.15)] cursor-pointer h-full">
              <div className="space-y-3">
                <div className="h-10 w-10 rounded-lg bg-accent/10 flex items-center justify-center group-hover:bg-accent/20 transition-colors">
                  <Rocket className="h-5 w-5 text-accent" />
                </div>
                <h3 className="text-lg font-semibold group-hover:text-accent transition-colors">快速入门</h3>
                <p className="text-sm text-muted-foreground">上位机软件使用，从开箱到第一次测试运行</p>
              </div>
            </div>
          </Link>

          {/* Nav Card 3 */}
          <Link href="/docs/sdk/overview">
            <div className="group p-6 rounded-xl bg-card/30 border border-border/40 hover:border-primary/50 transition-all hover:shadow-[0_0_20px_rgba(59,130,246,0.15)] cursor-pointer h-full">
              <div className="space-y-3">
                <div className="h-10 w-10 rounded-lg bg-primary/10 flex items-center justify-center group-hover:bg-primary/20 transition-colors">
                  <Code className="h-5 w-5 text-primary" />
                </div>
                <h3 className="text-lg font-semibold group-hover:text-primary transition-colors">SDK开发</h3>
                <p className="text-sm text-muted-foreground">Python、C++、ROS1、ROS2多语言SDK支持</p>
              </div>
            </div>
          </Link>

          {/* Nav Card 4 */}
          <Link href="/docs/teleoperation/overview">
            <div className="group p-6 rounded-xl bg-card/30 border border-border/40 hover:border-accent/50 transition-all hover:shadow-[0_0_20px_rgba(16,185,129,0.15)] cursor-pointer h-full">
              <div className="space-y-3">
                <div className="h-10 w-10 rounded-lg bg-accent/10 flex items-center justify-center group-hover:bg-accent/20 transition-colors">
                  <Gamepad2 className="h-5 w-5 text-accent" />
                </div>
                <h3 className="text-lg font-semibold group-hover:text-accent transition-colors">数据采集</h3>
                <p className="text-sm text-muted-foreground">VR头显、动捕手套、外骨骼手套多种方案</p>
              </div>
            </div>
          </Link>

          {/* Nav Card 5 */}
          <Link href="/docs/applications">
            <div className="group p-6 rounded-xl bg-card/30 border border-border/40 hover:border-primary/50 transition-all hover:shadow-[0_0_20px_rgba(59,130,246,0.15)] cursor-pointer h-full">
              <div className="space-y-3">
                <div className="h-10 w-10 rounded-lg bg-primary/10 flex items-center justify-center group-hover:bg-primary/20 transition-colors">
                  <Lightbulb className="h-5 w-5 text-primary" />
                </div>
                <h3 className="text-lg font-semibold group-hover:text-primary transition-colors">应用实践</h3>
                <p className="text-sm text-muted-foreground">控制模式、灵巧抓取、触觉感知应用</p>
              </div>
            </div>
          </Link>

          {/* Nav Card 6 */}
          <Link href="/docs/faq">
            <div className="group p-6 rounded-xl bg-card/30 border border-border/40 hover:border-accent/50 transition-all hover:shadow-[0_0_20px_rgba(16,185,129,0.15)] cursor-pointer h-full">
              <div className="space-y-3">
                <div className="h-10 w-10 rounded-lg bg-accent/10 flex items-center justify-center group-hover:bg-accent/20 transition-colors">
                  <HelpCircle className="h-5 w-5 text-accent" />
                </div>
                <h3 className="text-lg font-semibold group-hover:text-accent transition-colors">常见问题</h3>
                <p className="text-sm text-muted-foreground">FAQ、故障排查、问题解决方案</p>
              </div>
            </div>
          </Link>

          {/* Nav Card 7 */}
          <Link href="/docs/support/troubleshooting">
            <div className="group p-6 rounded-xl bg-card/30 border border-border/40 hover:border-primary/50 transition-all hover:shadow-[0_0_20px_rgba(59,130,246,0.15)] cursor-pointer h-full">
              <div className="space-y-3">
                <div className="h-10 w-10 rounded-lg bg-primary/10 flex items-center justify-center group-hover:bg-primary/20 transition-colors">
                  <LifeBuoy className="h-5 w-5 text-primary" />
                </div>
                <h3 className="text-lg font-semibold group-hover:text-primary transition-colors">支持与资源</h3>
                <p className="text-sm text-muted-foreground">技术支持、下载中心、术语词汇表</p>
              </div>
            </div>
          </Link>
        </div>
      </section>

      {/* CTA Section */}
      <section className="container relative z-10 py-24">
        <div className="max-w-4xl mx-auto text-center space-y-8 p-12 rounded-2xl bg-gradient-to-br from-primary/10 via-accent/5 to-transparent border border-primary/20">
          <h2 className="text-3xl md:text-4xl font-bold">
            <span className="bg-gradient-to-r from-primary via-accent to-primary bg-clip-text text-transparent">
              准备开始了吗？
            </span>
          </h2>
          <p className="text-lg text-muted-foreground max-w-2xl mx-auto">
            查看快速入门指南，几分钟内让XHAND1运行起来
          </p>
          <div className="flex flex-col sm:flex-row gap-4 justify-center">
            <Link href="/docs/quickstart/unboxing">
              <Button size="lg" className="w-full sm:w-auto bg-primary hover:bg-primary/90 shadow-[0_0_30px_rgba(59,130,246,0.3)]">
                运行第一个程序
                <ChevronRight className="ml-2 h-5 w-5" />
              </Button>
            </Link>
            <Button size="lg" variant="outline" className="w-full sm:w-auto border-border/60">
              <Download className="mr-2 h-5 w-5" />
              下载SDK
            </Button>
          </div>
        </div>
      </section>

      {/* Footer */}
      <footer className="border-t border-border/40 py-12 mt-auto relative z-10">
        <div className="container text-center space-y-4">
          <p className="text-sm text-muted-foreground">
            © 2024 北京星动纪元科技有限公司 版权所有京ICP备2023021939号
          </p>
          <p className="text-sm">
            <a href="mailto:support@robotera.com" className="text-muted-foreground hover:text-primary transition-colors">
              support@robotera.com
            </a>
          </p>
        </div>
      </footer>
    </div>
  );
}
