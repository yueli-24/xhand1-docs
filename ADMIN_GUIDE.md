# 管理员模式使用指南

## 概述

XHAND1文档中心已集成管理员模式，允许授权用户直接在网页上编辑文档内容。

## 功能特性

- **隐藏访问入口**：通过特殊路径 `/admin` 访问，不在网页上公开展示
- **登录认证**：需要用户名和密码才能访问管理功能
- **文档管理**：列出所有文档章节和子页面
- **在线编辑**：支持Markdown格式的在线编辑
- **文件上传**：支持上传.md文件批量更新文档
- **动态加载**：编辑后的内容实时保存到数据库，前端自动加载显示

## 登录凭据

- **用户名**：`tsuki`
- **密码**：`1Q2w3E4r`
- **访问地址**：`https://your-domain.com/admin`

## 使用流程

### 1. 登录管理后台

访问 `/admin` 路径，输入用户名和密码登录。

### 2. 查看文档列表

登录成功后进入Dashboard，显示所有文档章节：
- 关于XHAND1
- 快速入门
- SDK开发指南
- 应用实践
- 数据采集
- 常见问题
- 支持与资源

### 3. 编辑文档

点击任意文档进入编辑页面，提供两种编辑方式：

**方式一：在线编辑**
1. 在Markdown编辑器中直接编辑内容
2. 支持完整的Markdown语法
3. 点击"保存"按钮保存修改

**方式二：上传文件**
1. 点击"选择文件"按钮
2. 选择本地的.md文件
3. 文件内容会自动填充到编辑器
4. 点击"保存"保存上传的内容

### 4. 查看效果

保存后，返回对应的文档页面即可看到更新后的内容。

## 数据库结构

### admins表
存储管理员账户信息：
- `id`: 主键
- `username`: 用户名（唯一）
- `passwordHash`: 密码哈希值
- `createdAt`: 创建时间

### docContents表
存储文档内容：
- `id`: 主键
- `docId`: 文档ID（对应docs.ts中的id）
- `content`: Markdown内容
- `updatedAt`: 更新时间
- `updatedBy`: 更新者ID

## 技术实现

- **后端框架**：tRPC + Express
- **数据库**：PostgreSQL（通过Drizzle ORM）
- **认证方式**：Session-based（cookie）
- **前端框架**：React + Wouter
- **Markdown渲染**：Streamdown

## API端点

### admin.login
- **类型**：mutation
- **输入**：`{ username: string, password: string }`
- **输出**：`{ success: boolean }`
- **功能**：管理员登录

### admin.me
- **类型**：query
- **输出**：`{ id: number, username: string } | null`
- **功能**：获取当前登录的管理员信息

### admin.getDocContent
- **类型**：query
- **输入**：`{ docId: string }`
- **输出**：`{ id: number, docId: string, content: string, updatedAt: Date } | null`
- **功能**：获取指定文档的内容

### admin.saveDocContent
- **类型**：mutation
- **输入**：`{ docId: string, content: string }`
- **输出**：`{ success: boolean }`
- **功能**：保存文档内容

## 注意事项

1. **安全性**：管理员密码使用bcrypt加密存储，建议定期更换密码
2. **备份**：建议在大量修改前备份数据库
3. **权限控制**：当前只有一个管理员账户，如需添加更多管理员，需要手动在数据库中添加
4. **Markdown语法**：编辑时请遵循标准Markdown语法，避免使用HTML标签
5. **代码块转义**：如果内容包含代码块（\`\`\`），在保存到数据库前会自动转义为`\\\`\\\`\\\``

## 故障排除

### 登录失败
- 检查用户名和密码是否正确
- 检查数据库连接是否正常
- 查看服务器日志获取详细错误信息

### 内容保存失败
- 检查数据库连接
- 确认Markdown内容格式正确
- 查看浏览器控制台和服务器日志

### 页面显示异常
- 清除浏览器缓存
- 检查Markdown语法是否正确
- 确认数据库中的内容没有被破坏

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

- [ ] 添加多管理员支持
- [ ] 实现版本历史和回滚功能
- [ ] 添加Markdown实时预览
- [ ] 支持图片上传和管理
- [ ] 添加操作日志记录
- [ ] 实现批量导入/导出功能
