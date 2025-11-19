# 管理员模式使用指南

## 概述

XHAND1文档中心已集成管理员模式，允许授权用户直接在网页上编辑文档内容。管理员功能基于OAuth用户认证系统，只有role为"admin"的用户才能访问。

## 功能特性

- **OAuth集成认证**：使用Google/GitHub等OAuth登录，role为admin的用户自动获得管理权限
- **隐藏访问入口**：通过特殊路径 `/admin` 访问，不在网页上公开展示
- **文档管理**：列出所有文档章节和子页面
- **在线编辑**：支持Markdown格式的在线编辑
- **文件上传**：支持上传.md文件批量更新文档
- **动态加载**：编辑后的内容实时保存到数据库，前端自动加载显示

## 访问管理后台

### 方式一：通过OAuth登录（推荐）

1. 访问网站首页，点击右上角"登录"按钮
2. 选择Google、GitHub或其他OAuth提供商登录
3. 确保您的账户role字段为"admin"（需要管理员在数据库中设置）
4. 登录成功后，直接访问 `/admin` 路径即可进入管理后台

### 方式二：直接访问（已登录用户）

如果您已经通过OAuth登录，并且role为admin，可以直接访问：
- `https://your-domain.com/admin` - 管理员Dashboard
- `https://your-domain.com/admin/edit/{docId}` - 编辑指定文档

## 使用流程

### 1. 查看文档列表

访问 `/admin` 进入Dashboard，显示所有文档章节：
- 关于XHAND1（5个子页面）
- 快速入门（4个子页面）
- SDK开发指南（6个子页面）
- 应用实践（4个子页面）
- 数据采集（5个子页面）
- 常见问题
- 支持与资源（4个子页面）

Dashboard显示：
- 文档总数
- 已编辑文档数量
- 每个文档的标题、路径和ID

### 2. 编辑文档

点击任意文档卡片进入编辑页面，提供以下功能：

**在线编辑**
- Markdown编辑器支持完整的Markdown语法
- 实时编辑，支持代码高亮
- 点击"保存"按钮保存修改

**上传文件**
1. 点击"上传MD"按钮
2. 选择本地的.md文件
3. 文件内容会自动填充到编辑器
4. 点击"保存"保存上传的内容

**预览功能**
- 点击"预览"按钮在新标签页中查看文档效果
- 预览页面使用与前端相同的渲染方式

**返回列表**
- 点击"返回"按钮回到Dashboard

### 3. 查看效果

保存后，访问对应的文档页面即可看到更新后的内容。系统会优先从数据库加载已编辑的内容，如果数据库中没有，则使用docs.ts中的默认内容。

## 权限管理

### 设置管理员权限

管理员权限通过users表的role字段控制。要授予某个用户管理员权限：

**方式一：通过数据库UI**
1. 登录管理后台（Management UI）
2. 进入Database面板
3. 找到users表
4. 找到目标用户的记录
5. 将role字段修改为"admin"

**方式二：通过SQL**
```sql
UPDATE users 
SET role = 'admin' 
WHERE email = 'user@example.com';
```

### 当前管理员

系统会在Dashboard顶部显示当前登录的管理员信息（用户名或邮箱）。

## 数据库结构

### users表（OAuth用户）
- `id`: 主键
- `openId`: OAuth唯一标识
- `name`: 用户名
- `email`: 邮箱
- `role`: 用户角色（"user" 或 "admin"）
- `loginMethod`: 登录方式（google、github等）
- `createdAt`: 创建时间
- `lastSignedIn`: 最后登录时间

### docContents表（文档内容）
- `id`: 主键
- `docId`: 文档ID（对应docs.ts中的id）
- `content`: Markdown内容
- `updatedAt`: 更新时间
- `updatedBy`: 更新者ID（对应users.id）

## 技术实现

- **后端框架**：tRPC + Express
- **数据库**：PostgreSQL（通过Drizzle ORM）
- **认证方式**：OAuth 2.0（支持Google、GitHub等）
- **权限控制**：基于users.role字段
- **前端框架**：React + Wouter
- **Markdown渲染**：Streamdown

## API端点

### admin.me
- **类型**：query
- **权限**：需要OAuth登录且role为admin
- **输出**：`{ id: number, username: string } | null`
- **功能**：获取当前登录的管理员信息

### admin.listDocs
- **类型**：query
- **权限**：需要OAuth登录且role为admin
- **输出**：`Array<{ id: number, docId: string, content: string, updatedAt: Date }>`
- **功能**：获取所有已编辑的文档列表

### admin.getDoc
- **类型**：query
- **权限**：需要OAuth登录且role为admin
- **输入**：`{ docId: string }`
- **输出**：`{ id: number, docId: string, content: string, updatedAt: Date } | null`
- **功能**：获取指定文档的内容

### admin.updateDoc
- **类型**：mutation
- **权限**：需要OAuth登录且role为admin
- **输入**：`{ docId: string, content: string }`
- **输出**：`{ success: boolean }`
- **功能**：保存或更新文档内容

## 注意事项

1. **权限安全**：只有role为"admin"的OAuth用户才能访问管理功能，请谨慎授予管理员权限
2. **数据备份**：建议在大量修改前备份数据库（通过Management UI的Database面板导出）
3. **Markdown语法**：编辑时请遵循标准Markdown语法，避免使用HTML标签
4. **代码块转义**：如果内容包含代码块（\`\`\`），系统会自动处理转义
5. **并发编辑**：当前不支持多人同时编辑同一文档，后保存的内容会覆盖先保存的内容

## 故障排除

### 无法访问管理后台（显示"未授权"）
- 确认已通过OAuth登录
- 检查用户的role字段是否为"admin"
- 清除浏览器缓存并重新登录

### 内容保存失败
- 检查数据库连接是否正常
- 确认Markdown内容格式正确
- 查看浏览器控制台和服务器日志获取详细错误信息

### 页面显示异常
- 清除浏览器缓存
- 检查Markdown语法是否正确
- 确认数据库中的内容没有被破坏

### 编辑后内容未更新
- 刷新文档页面（Ctrl+F5强制刷新）
- 检查数据库中的docContents表是否有对应记录
- 确认updatedAt时间戳是否为最新

## 开发测试

已通过vitest测试验证：
- ✅ 管理员账户创建
- ✅ 密码验证
- ✅ 文档内容CRUD操作

测试文件位置：`server/admin.test.ts`

运行测试：
```bash
pnpm test server/admin.test.ts
```

## 未来改进

- [ ] 添加版本历史和回滚功能
- [ ] 实现Markdown实时预览（分屏模式）
- [ ] 支持图片上传和管理
- [ ] 添加操作日志记录
- [ ] 实现批量导入/导出功能
- [ ] 支持多人协作编辑（冲突检测）
- [ ] 添加文档搜索和过滤功能

## 更新日志

### v2.0 (2025-11-19)
- 🔄 **重大变更**：从独立管理员系统迁移到OAuth认证
- ✅ 使用users表的role字段进行权限控制
- ✅ 移除了admins表和独立登录页面
- ✅ 简化了认证流程，与主应用的用户系统统一

### v1.0 (2025-11-19)
- ✅ 初始版本
- ✅ 实现基本的文档管理功能
- ✅ 支持Markdown编辑和文件上传
- ✅ 实现文档内容动态加载
