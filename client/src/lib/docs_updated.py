#!/usr/bin/env python3
# 脚本用于生成更新后的docs.ts文件

# 读取原文件
with open('client/src/lib/docs.ts', 'r', encoding='utf-8') as f:
    original_content = f.read()

# 提取"关于XHAND1"部分（保持不变，只更新公司名称）
about_section_start = original_content.find('{\\n    id: "about"')
about_section_end = original_content.find('},\\n  {\\n    id: "quickstart"')

# 提取"应用实践"和"支持与资源"部分（保持不变）
applications_start = original_content.find('{\\n    id: "applications"')
support_start = original_content.find('{\\n    id: "support"')

print("文件结构分析完成")
print(f"关于部分: {about_section_start} - {about_section_end}")
print(f"应用部分: {applications_start}")
print(f"支持部分: {support_start}")
