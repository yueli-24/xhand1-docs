import { useEffect, useState, useRef } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Textarea } from "@/components/ui/textarea";
import { trpc } from "@/lib/trpc";
import { useLocation, useRoute } from "wouter";
import { toast } from "sonner";
import { ArrowLeft, Save, Upload, Eye } from "lucide-react";
import { docsData } from "@/lib/docs";
import { Streamdown } from "streamdown";

export default function AdminEditDoc() {
  const [, params] = useRoute("/admin/edit/:docId");
  const [, setLocation] = useLocation();
  const docId = params?.docId || "";
  
  const [content, setContent] = useState("");
  const [showPreview, setShowPreview] = useState(false);
  const fileInputRef = useRef<HTMLInputElement>(null);
  
  const { data: adminSession, isLoading: sessionLoading } = trpc.admin.me.useQuery(undefined, {
    retry: false,
    refetchOnWindowFocus: false,
    refetchOnMount: false,
  });
  const { data: savedDoc, isLoading: docLoading } = trpc.admin.getDoc.useQuery(
    { docId },
    { enabled: !!docId }
  );
  
  const updateMutation = trpc.admin.updateDoc.useMutation({
    onSuccess: () => {
      toast.success("文档保存成功");
    },
    onError: (error: any) => {
      toast.error(error.message || "保存失败");
    },
  });

  useEffect(() => {
    if (!sessionLoading && !adminSession) {
      // Redirect to home if not admin
      setLocation("/");
    }
  }, [adminSession, sessionLoading, setLocation]);

  // Show loading while checking session
  if (sessionLoading) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-background">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mx-auto mb-4"></div>
          <p className="text-muted-foreground">验证权限中...</p>
        </div>
      </div>
    );
  }

  // If no admin session, show redirect message
  if (!adminSession) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-background">
        <div className="text-center">
          <p className="text-muted-foreground">正在跳转...</p>
        </div>
      </div>
    );
  }

  useEffect(() => {
    if (savedDoc) {
      setContent(savedDoc.content);
    } else {
      // Find document in docsData to get default content
      const findDocInData = (items: any[]): any => {
        for (const item of items) {
          if (item.id === docId) return item;
          if (item.children) {
            const found = findDocInData(item.children);
            if (found) return found;
          }
        }
        return null;
      };
      
      const doc = findDocInData(docsData);
      if (doc && doc.content) {
        setContent(doc.content);
      }
    }
  }, [savedDoc, docId]);

  if (sessionLoading || docLoading) {
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

  // Find document info
  const findDocInfo = (items: any[]): any => {
    for (const item of items) {
      if (item.id === docId) return item;
      if (item.children) {
        const found = findDocInfo(item.children);
        if (found) return found;
      }
    }
    return null;
  };
  
  const doc = findDocInfo(docsData);
  
  if (!doc) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-background">
        <div className="text-center">
          <p className="text-lg text-muted-foreground">文档未找到</p>
          <Button className="mt-4" onClick={() => setLocation("/admin/dashboard")}>
            返回Dashboard
          </Button>
        </div>
      </div>
    );
  }

  const handleSave = () => {
    updateMutation.mutate({ docId, content });
  };

  const handleFileUpload = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    
    if (!file.name.endsWith('.md') && !file.name.endsWith('.markdown')) {
      toast.error("请上传Markdown文件（.md或.markdown）");
      return;
    }
    
    const reader = new FileReader();
    reader.onload = (event) => {
      const text = event.target?.result as string;
      setContent(text);
      toast.success("文件上传成功");
    };
    reader.onerror = () => {
      toast.error("文件读取失败");
    };
    reader.readAsText(file);
  };

  return (
    <div className="min-h-screen bg-background">
      {/* Header */}
      <header className="border-b border-border/40 bg-background/80 backdrop-blur-xl sticky top-0 z-50">
        <div className="container flex h-16 items-center justify-between">
          <div className="flex items-center gap-4">
            <Button
              variant="ghost"
              size="sm"
              onClick={() => setLocation("/admin/dashboard")}
            >
              <ArrowLeft className="mr-2 h-4 w-4" />
              返回
            </Button>
            <div>
              <h1 className="text-lg font-semibold">{doc.title}</h1>
              <p className="text-sm text-muted-foreground">{doc.path}</p>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <Button
              variant="outline"
              size="sm"
              onClick={() => setShowPreview(!showPreview)}
            >
              <Eye className="mr-2 h-4 w-4" />
              {showPreview ? "编辑" : "预览"}
            </Button>
            <Button
              variant="outline"
              size="sm"
              onClick={() => fileInputRef.current?.click()}
            >
              <Upload className="mr-2 h-4 w-4" />
              上传MD
            </Button>
            <Button
              size="sm"
              onClick={handleSave}
              disabled={updateMutation.isPending}
            >
              <Save className="mr-2 h-4 w-4" />
              {updateMutation.isPending ? "保存中..." : "保存"}
            </Button>
          </div>
        </div>
      </header>

      {/* Hidden file input */}
      <input
        ref={fileInputRef}
        type="file"
        accept=".md,.markdown"
        onChange={handleFileUpload}
        className="hidden"
      />

      {/* Main Content */}
      <main className="container py-8">
        {showPreview ? (
          <Card>
            <CardHeader>
              <CardTitle>预览</CardTitle>
              <CardDescription>Markdown渲染预览</CardDescription>
            </CardHeader>
            <CardContent>
              <div className="prose prose-invert max-w-none">
                <Streamdown>{content}</Streamdown>
              </div>
            </CardContent>
          </Card>
        ) : (
          <Card>
            <CardHeader>
              <CardTitle>编辑内容</CardTitle>
              <CardDescription>
                使用Markdown格式编辑文档内容，或上传Markdown文件
              </CardDescription>
            </CardHeader>
            <CardContent>
              <Textarea
                value={content}
                onChange={(e) => setContent(e.target.value)}
                className="min-h-[600px] font-mono text-sm"
                placeholder="在此输入Markdown内容..."
              />
              <div className="mt-4 text-sm text-muted-foreground">
                字符数: {content.length}
              </div>
            </CardContent>
          </Card>
        )}
      </main>
    </div>
  );
}
