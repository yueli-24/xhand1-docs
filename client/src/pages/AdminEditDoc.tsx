import { useState, useEffect, useRef } from "react";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { Textarea } from "@/components/ui/textarea";
import { trpc } from "@/lib/trpc";
import { useLocation, useRoute } from "wouter";
import { toast } from "sonner";
import { ArrowLeft, Save, Upload, Eye, Image, Paperclip } from "lucide-react";
import { docsData } from "@/lib/docs";
import { Streamdown } from "streamdown";
import { useAuth } from "@/_core/hooks/useAuth";

export default function AdminEditDoc() {
  const [, params] = useRoute("/admin/edit/:docId");
  const [, setLocation] = useLocation();
  const docId = params?.docId || "";
  
  const [content, setContent] = useState("");
  const [showPreview, setShowPreview] = useState(false);
  const [isDragging, setIsDragging] = useState(false);
  const [checkComplete, setCheckComplete] = useState(false);
  const fileInputRef = useRef<HTMLInputElement>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);
  const { user, loading: authLoading } = useAuth();
  const { data: savedDoc, isLoading: docLoading } = trpc.admin.getDoc.useQuery(
    { docId },
    { 
      enabled: !!docId && !!user && user.role === 'admin',
      retry: false,
      refetchOnWindowFocus: false,
      refetchOnMount: false,
    }
  );
  
  const updateMutation = trpc.admin.updateDoc.useMutation({
    onSuccess: () => {
      toast.success("文档保存成功");
    },
    onError: (error: any) => {
      toast.error(error.message || "保存失败");
    },
  });
  
  const uploadFileMutation = trpc.admin.uploadFile.useMutation({
    onSuccess: (data) => {
      // Insert markdown syntax at cursor position
      const textarea = textareaRef.current;
      if (!textarea) return;
      
      const start = textarea.selectionStart;
      const end = textarea.selectionEnd;
      const isImage = data.fileName.match(/\.(jpg|jpeg|png|gif|webp|svg)$/i);
      const markdown = isImage 
        ? `![${data.fileName}](${data.url})`
        : `[${data.fileName}](${data.url})`;
      
      const newContent = content.substring(0, start) + markdown + content.substring(end);
      setContent(newContent);
      
      // Move cursor after inserted text
      setTimeout(() => {
        textarea.focus();
        const newPos = start + markdown.length;
        textarea.setSelectionRange(newPos, newPos);
      }, 0);
      
      toast.success(`${isImage ? '图片' : '附件'}上传成功`);
    },
    onError: (error: any) => {
      toast.error(error.message || "上传失败");
    },
  });

  // All useEffect hooks must be at the top level, before any conditional returns
  // Wait for auth to complete before checking admin status
  useEffect(() => {
    if (!authLoading) {
      const timer = setTimeout(() => {
        setCheckComplete(true);
      }, 500);
      return () => clearTimeout(timer);
    }
  }, [authLoading]);

  // Redirect non-admin users after check is complete
  useEffect(() => {
    if (checkComplete && (!user || user.role !== 'admin')) {
      setLocation("/");
    }
  }, [checkComplete, user, setLocation]);

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

  // Show loading while checking auth
  if (authLoading || !checkComplete) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-background">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mx-auto mb-4"></div>
          <p className="text-muted-foreground">验证权限中...</p>
        </div>
      </div>
    );
  }

  // If no admin user, show redirect message
  if (!user || user.role !== 'admin') {
    return (
      <div className="min-h-screen flex items-center justify-center bg-background">
        <div className="text-center">
          <p className="text-muted-foreground">正在跳转...</p>
        </div>
      </div>
    );
  }

  if (docLoading) {
    return (
      <div className="min-h-screen flex items-center justify-center bg-background">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-primary mx-auto mb-4"></div>
          <p className="text-muted-foreground">加载中...</p>
        </div>
      </div>
    );
  }

  // Already checked above, this is redundant
  // if (!user || user.role !== 'admin') {
  //   return null;
  // }

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
  
  // Handle image/attachment upload
  const handleImageUpload = async (file: File) => {
    // Check file size (max 10MB)
    if (file.size > 10 * 1024 * 1024) {
      toast.error("文件大小不能超过10MB");
      return;
    }
    
    // Read file as base64
    const reader = new FileReader();
    reader.onload = async (e) => {
      const base64 = e.target?.result as string;
      const base64Data = base64.split(',')[1]; // Remove data:image/...;base64, prefix
      
      uploadFileMutation.mutate({
        fileName: file.name,
        fileData: base64Data,
        fileType: file.type,
      });
    };
    reader.onerror = () => {
      toast.error("文件读取失败");
    };
    reader.readAsDataURL(file);
  };
  
  // Drag and drop handlers
  const handleDragOver = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(true);
  };
  
  const handleDragLeave = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(false);
  };
  
  const handleDrop = (e: React.DragEvent) => {
    e.preventDefault();
    e.stopPropagation();
    setIsDragging(false);
    
    const files = Array.from(e.dataTransfer.files);
    if (files.length === 0) return;
    
    // Upload each file
    files.forEach(file => {
      handleImageUpload(file);
    });
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
              <article className="prose prose-slate dark:prose-invert max-w-none">
                <Streamdown>{content}</Streamdown>
              </article>
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
              <div 
                className="relative"
                onDragOver={handleDragOver}
                onDragLeave={handleDragLeave}
                onDrop={handleDrop}
              >
                {isDragging && (
                  <div className="absolute inset-0 z-10 flex items-center justify-center bg-primary/10 border-2 border-dashed border-primary rounded-lg">
                    <div className="text-center">
                      <Image className="mx-auto h-12 w-12 text-primary mb-2" />
                      <p className="text-lg font-medium text-primary">拖放图片或附件到这里</p>
                      <p className="text-sm text-muted-foreground mt-1">支持 JPG, PNG, GIF, PDF 等格式</p>
                    </div>
                  </div>
                )}
                <Textarea
                  ref={textareaRef}
                  value={content}
                  onChange={(e) => setContent(e.target.value)}
                  placeholder="输入Markdown内容...或拖放图片/附件到这里"
                  className="min-h-[600px] font-mono"
                />
              </div>
              <div className="mt-4 flex items-center justify-between text-sm text-muted-foreground">
                <span>字符数: {content.length}</span>
                <span className="flex items-center gap-2">
                  <Image className="h-4 w-4" />
                  <span>支持拖放图片和附件</span>
                </span>
              </div>
            </CardContent>
          </Card>
        )}
      </main>
    </div>
  );
}
