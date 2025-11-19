import { useState } from "react";
import { Link, useLocation } from "wouter";
import { ChevronDown, ChevronRight, Menu, X, Search } from "lucide-react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { docsData, type DocSection } from "@/lib/docs";
import { APP_LOGO, APP_TITLE } from "@/const";

interface DocLayoutProps {
  children: React.ReactNode;
}

function NavItem({ section, level = 0 }: { section: DocSection; level?: number }) {
  const [location] = useLocation();
  const [isOpen, setIsOpen] = useState(true);
  const hasChildren = section.children && section.children.length > 0;
  const isActive = location === section.path;
  
  return (
    <div className="mb-1">
      <div
        className={`flex items-center gap-2 px-3 py-2 rounded-md text-sm cursor-pointer transition-all ${
          isActive
            ? "bg-primary/20 text-primary font-medium border-l-2 border-primary shadow-[0_0_10px_rgba(59,130,246,0.2)]"
            : "hover:bg-accent/10 hover:text-accent-foreground hover:border-l-2 hover:border-accent/50"
        }`}
        style={{ paddingLeft: `${level * 12 + 12}px` }}
      >
        {hasChildren && (
          <button
            onClick={(e) => {
              e.stopPropagation();
              setIsOpen(!isOpen);
            }}
            className="p-0 h-4 w-4"
          >
            {isOpen ? <ChevronDown className="h-4 w-4" /> : <ChevronRight className="h-4 w-4" />}
          </button>
        )}
        {!hasChildren && <div className="w-4" />}
        <Link href={section.path} className="flex-1">
          <span>{section.title}</span>
          {hasChildren && (
            <span className="ml-2 text-xs opacity-60">
              {section.children?.length}
            </span>
          )}
        </Link>
      </div>
      {hasChildren && isOpen && (
        <div className="mt-1">
          {section.children?.map((child) => (
            <NavItem key={child.id} section={child} level={level + 1} />
          ))}
        </div>
      )}
    </div>
  );
}

export default function DocLayout({ children }: DocLayoutProps) {
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [searchQuery, setSearchQuery] = useState("");

  return (
    <div className="min-h-screen flex flex-col">
      {/* Header */}
      <header className="sticky top-0 z-50 w-full border-b border-border/40 bg-background/80 backdrop-blur-xl shadow-[0_1px_20px_rgba(0,0,0,0.3)]">
        <div className="container flex h-16 items-center justify-between">
          <div className="flex items-center gap-4">
            <Button
              variant="ghost"
              size="icon"
              className="lg:hidden"
              onClick={() => setSidebarOpen(!sidebarOpen)}
            >
              {sidebarOpen ? <X className="h-5 w-5" /> : <Menu className="h-5 w-5" />}
            </Button>
            <Link href="/" className="flex items-center gap-3">
              <img src={APP_LOGO} alt={APP_TITLE} className="h-10" style={{width: '150px'}} />
              <div className="h-8 w-px bg-border/60" />
              <span className="font-semibold text-lg tracking-tight">{APP_TITLE}</span>
            </Link>
          </div>
          
          <div className="flex items-center gap-4">
            <div className="relative hidden md:block">
              <Search className="absolute left-2 top-1/2 h-4 w-4 -translate-y-1/2 text-muted-foreground" />
              <Input
                type="search"
                placeholder="搜索文档..."
                className="pl-8 w-64 bg-muted/50 border-border/60 focus:border-primary/50 focus:ring-1 focus:ring-primary/20"
                value={searchQuery}
                onChange={(e) => setSearchQuery(e.target.value)}
              />
            </div>
          </div>
        </div>
      </header>

      <div className="flex-1 flex">
        {/* Sidebar */}
        <aside
          className={`fixed lg:sticky top-16 left-0 z-40 h-[calc(100vh-4rem)] w-64 border-r border-border/40 bg-background/50 backdrop-blur-xl transition-transform lg:translate-x-0 ${
            sidebarOpen ? "translate-x-0" : "-translate-x-full"
          }`}
        >
          <div className="h-full overflow-y-auto p-4">
            <nav className="space-y-1">
              {docsData.map((section) => (
                <NavItem key={section.id} section={section} />
              ))}
            </nav>
          </div>
        </aside>

        {/* Overlay for mobile */}
        {sidebarOpen && (
          <div
            className="fixed inset-0 z-30 bg-background/80 backdrop-blur-sm lg:hidden"
            onClick={() => setSidebarOpen(false)}
          />
        )}

        {/* Main content */}
        <main className="flex-1 overflow-y-auto">
          <div className="container max-w-4xl py-8">
            {children}
          </div>
        </main>
      </div>
    </div>
  );
}
