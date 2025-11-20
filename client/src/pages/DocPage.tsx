import { useRoute, useLocation } from "wouter";
import { Streamdown } from "streamdown";
import DocLayout from "@/components/DocLayout";
import { findDocByPath } from "@/lib/docs";
import { Button } from "@/components/ui/button";
import { Link } from "wouter";
import { ArrowLeft } from "lucide-react";
import { trpc } from "@/lib/trpc";

export default function DocPage() {
  const [location] = useLocation();
  const path = location;
  
  const doc = findDocByPath(path);
  
  // Try to load custom content from database
  const { data: savedDoc } = trpc.docs.getDoc.useQuery(
    { docId: doc?.id || "" },
    { enabled: !!doc?.id, retry: false }
  );
  
  // Use saved content if available, otherwise use default
  const content = savedDoc?.content || doc?.content;

  if (!doc) {
    return (
      <DocLayout>
        <div className="text-center py-12">
          <h1 className="text-4xl font-bold mb-4">页面未找到</h1>
          <p className="text-muted-foreground mb-6">
            抱歉，您访问的文档页面不存在。
          </p>

          <Link href="/docs/about/overview">
            <Button>
              <ArrowLeft className="mr-2 h-4 w-4" />
              返回文档首页
            </Button>
          </Link>
        </div>
      </DocLayout>
    );
  }

  // 如果是目录页（有children但没有content），显示子页面列表
  if (doc.children && doc.children.length > 0 && !doc.content) {
    return (
      <DocLayout>
        <article className="prose prose-slate dark:prose-invert max-w-none">
          <h1>{doc.title}</h1>
          <p className="lead">
            请从左侧导航栏选择具体的文档页面，或从下方列表中选择：
          </p>
        </article>
        <section className="grid gap-4 mt-8">
            {doc.children.map((child) => (
              <Link key={child.id} href={child.path}>
                <div className="border rounded-lg p-4 hover:border-primary transition-colors cursor-pointer">
                  <h3 className="text-lg font-semibold mb-2">{child.title}</h3>
                  {child.children && child.children.length > 0 && (
                    <p className="text-sm text-muted-foreground">
                      包含 {child.children.length} 个子章节
                    </p>
                  )}
                </div>
              </Link>
            ))}
        </section>
      </DocLayout>
    );
  }

  return (
    <DocLayout>
      <article className="prose prose-slate dark:prose-invert max-w-none">
        <Streamdown>{content || `# ${doc.title}\n\n内容正在完善中...`}</Streamdown>
      </article>
    </DocLayout>
  );
}
