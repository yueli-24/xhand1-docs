import { useState, useEffect } from "react";;
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { trpc } from "@/lib/trpc";
import { useLocation } from "wouter";
import { toast } from "sonner";
import { LogOut, FileText, Plus } from "lucide-react";
import { docsData } from "@/lib/docs";

export default function AdminDashboard() {
  const [, setLocation] = useLocation();
  
  const { data: adminSession, isLoading: sessionLoading } = trpc.admin.me.useQuery();
  const { data: savedDocs } = trpc.admin.listDocs.useQuery();
  const utils = trpc.useUtils();
  
  const logoutMutation = trpc.auth.logout.useMutation({
    onSuccess: () => {
      // Clear auth cache
      utils.auth.me.setData(undefined, null);
      toast.success("已退出登录");
      // Redirect to home page
      setLocation("/");
    },
  });

  useEffect(() => {
    if (!sessionLoading && !adminSession) {
      // Redirect to home if not admin
      setLocation("/");
    }
  }, [adminSession, sessionLoading, setLocation]);

  if (sessionLoading) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-background">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mx-auto mb-4"></div>
          <p className="text-muted-foreground">加载中...</p>
        </div>
      </div>
    );
  }

  if (!adminSession) {
    return null;
  }

  // Flatten all documents from docsData
  const allDocs: Array<{ id: string; title: string; path: string }> = [];
  
  function collectDocs(items: any[]) {
    items.forEach(item => {
      allDocs.push({
        id: item.id,
        title: item.title,
        path: item.path,
      });
      if (item.children) {
        collectDocs(item.children);
      }
    });
  }
  
  collectDocs(docsData);

  // Create a map of saved docs for quick lookup
  const savedDocsMap = new Map(savedDocs?.map(doc => [doc.docId, doc]) || []);

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <header className="border-b border-border/40 bg-background/80 backdrop-blur-xl sticky top-0 z-50">
        <div className="container flex h-16 items-center justify-between">
          <div className="flex items-center gap-4">
            <FileText className="h-6 w-6 text-primary" />
            <div>
              <h1 className="text-lg font-semibold">文档管理系统</h1>
              <p className="text-sm text-muted-foreground">管理员: {adminSession.username || 'Admin'}</p>
            </div>
          </div>
          <Button
            variant="outline"
            size="sm"
            onClick={() => logoutMutation.mutate()}
            disabled={logoutMutation.isPending}
          >
            <LogOut className="mr-2 h-4 w-4" />
            退出登录
          </Button>
        </div>
      </header>

      {/* Main Content */}
      <main className="container py-8">
        <div className="mb-8">
          <h2 className="text-2xl font-bold mb-2">文档列表</h2>
          <p className="text-muted-foreground">
            点击文档进行编辑。共 {allDocs.length} 个文档，已编辑 {savedDocs?.length || 0} 个。
          </p>
        </div>

        <div className="grid gap-4 md:grid-cols-2 lg:grid-cols-3">
          {allDocs.map(doc => {
            const savedDoc = savedDocsMap.get(doc.id);
            const isEdited = !!savedDoc;
            
            return (
              <Card
                key={doc.id}
                className="cursor-pointer hover:border-primary/50 transition-all"
                onClick={() => setLocation(`/admin/edit/${doc.id}`)}
              >
                <CardHeader>
                  <div className="flex items-start justify-between">
                    <CardTitle className="text-lg">{doc.title}</CardTitle>
                    {isEdited && (
                      <span className="text-xs bg-primary/10 text-primary px-2 py-1 rounded">
                        已编辑
                      </span>
                    )}
                  </div>
                  <CardDescription className="text-sm">
                    {doc.path}
                  </CardDescription>
                </CardHeader>
                <CardContent>
                  <div className="flex items-center gap-2 text-sm text-muted-foreground">
                    <FileText className="h-4 w-4" />
                    <span>ID: {doc.id}</span>
                  </div>
                  {isEdited && savedDoc && (
                    <div className="mt-2 text-xs text-muted-foreground">
                      最后更新: {new Date(savedDoc.updatedAt).toLocaleString('zh-CN')}
                    </div>
                  )}
                </CardContent>
              </Card>
            );
          })}
        </div>
      </main>
    </div>
  );
}
