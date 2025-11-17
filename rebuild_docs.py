#!/usr/bin/env python3
import re

# 读取原文件
with open('client/src/lib/docs.ts', 'r', encoding='utf-8') as f:
    content = f.read()

# 读取新的快速入门、SDK和遥操作章节
with open('client/src/lib/docs_new_structure.ts', 'r', encoding='utf-8') as f:
    new_content = f.read()

# 提取新章节的内容
# 提取quickstart部分
quickstart_match = re.search(r'export const quickstartSectionNew = ({.*?});', new_content, re.DOTALL)
# 提取sdk部分  
sdk_match = re.search(r'export const sdkSectionNew = ({.*?});', new_content, re.DOTALL)
# 提取teleoperation部分
tele_match = re.search(r'export const teleoperationSectionNew = ({.*?});', new_content, re.DOTALL)

print("提取新章节...")
print(f"Quickstart: {'找到' if quickstart_match else '未找到'}")
print(f"SDK: {'找到' if sdk_match else '未找到'}")
print(f"Teleoperation: {'找到' if tele_match else '未找到'}")

# 找到原文件中需要替换的部分
# 找到quickstart章节的开始和结束
quickstart_start = content.find('{\n    id: "quickstart"')
quickstart_end = content.find('},\n  {\n    id: "sdk"')

# 找到sdk章节
sdk_start = content.find('{\n    id: "sdk"')
sdk_end = content.find('},\n  {\n    id: "applications"')

print(f"\n原文件结构:")
print(f"Quickstart: {quickstart_start} - {quickstart_end}")
print(f"SDK: {sdk_start} - {sdk_end}")

if quickstart_start > 0 and sdk_start > 0:
    # 构建新文件
    # 保留开头部分（类型定义和about章节）
    new_file = content[:quickstart_start]
    
    # 添加新的quickstart章节
    if quickstart_match:
        new_file += quickstart_match.group(1) + ',\n  '
    
    # 添加新的sdk章节
    if sdk_match:
        new_file += sdk_match.group(1) + ',\n  '
    
    # 添加新的teleoperation章节
    if tele_match:
        new_file += tele_match.group(1) + ',\n  '
    
    # 添加原有的applications和support章节
    new_file += content[sdk_end+4:]  # +4 跳过 '},\n  '
    
    # 写入新文件
    with open('client/src/lib/docs.ts', 'w', encoding='utf-8') as f:
        f.write(new_file)
    
    print("\n文档更新完成！")
else:
    print("\n错误：无法找到章节位置")
