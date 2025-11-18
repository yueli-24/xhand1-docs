import re

# 读取文件
with open('client/src/lib/docs.ts', 'r', encoding='utf-8') as f:
    content = f.read()

# 定义一级tab的总括内容
section_intros = {
    '"about"': '''content: \`# 关于 XHAND1

本章节全面介绍XHAND1全直驱灵巧手的产品信息，包括核心技术特性、详细技术规格和产品对比分析。通过本章节，您将深入了解XHAND1的技术优势、应用场景和产品定位。

## 章节导航

- **产品概述**：了解XHAND1的核心定位、设计理念和应用场景
- **核心特性与优势**：深入了解全直驱设计、触觉感知、抓握能力等核心技术
- **技术规格**：查看完整的技术参数和性能指标
- **产品对比**：与其他灵巧手产品的对比分析

\`,''',
    
    '"quickstart"': '''content: \`# 快速入门

本章节帮助您快速上手XHAND1灵巧手，从开箱准备到完成第一次测试运行。我们提供了详细的上位机软件使用指南，让您能够在最短时间内熟悉产品操作。

## 章节导航

- **开箱与准备**：了解包装清单、硬件连接和安全注意事项
- **上位机软件使用**：掌握上位机软件的安装、配置和基本操作
- **第一次测试运行**：完成您的第一个测试程序，验证系统正常工作

\`,''',
    
    '"sdk"': '''content: \`# SDK 开发指南

本章节提供XHAND1 SDK的完整开发指南，涵盖Python、C++、ROS1和ROS2多种开发环境。通过本章节，您将掌握如何使用SDK进行灵巧手的编程控制和算法开发。

## 章节导航

- **SDK架构概述**：了解SDK的整体架构、API设计和开发工具
- **Python SDK**：使用Python进行快速原型开发和算法验证
- **C++ SDK**：使用C++进行高性能实时控制开发
- **ROS1集成**：在ROS1环境中集成和使用XHAND1
- **ROS2集成**：在ROS2环境中集成和使用XHAND1

\`,''',
    
    '"teleoperation"': '''content: \`# 遥操作

本章节介绍XHAND1的遥操作解决方案，支持VR头显、动捕手套、外骨骼手套等多种遥操作方式。通过自研的高精度遥操作软件，实现人手与XHAND1的精准动作重定向（Retargeting），助力远程操作测试和高质量数据采集。

## 章节导航

- **遥操作概述**：了解遥操作技术原理、Retargeting算法和应用场景
- **Meta Quest遥操作**：使用Meta Quest VR头显进行遥操作
- **Apple Vision Pro遥操作**：使用Apple Vision Pro进行高精度遥操作
- **MANUS手套遥操作**：使用MANUS动捕手套进行精细操作控制

\`,''',
    
    '"applications"': '''content: \`# 应用实践

本章节提供XHAND1在实际应用中的详细指导，涵盖控制模式、灵巧抓取和触觉感知等核心应用场景。通过实践案例和技术解析，帮助您充分发挥XHAND1的性能潜力。

## 章节导航

- **控制模式详解**：深入了解位置控制、力控、混合控制等多种控制模式
- **灵巧抓取与操作**：掌握抓取规划、力控抓取和精细操作技巧
- **触觉感知应用**：利用高分辨率触觉传感器实现智能感知和自适应控制

\`,''',
    
    '"support"': '''content: \`# 支持与资源

本章节提供XHAND1的技术支持资源，包括故障排查指南、术语词汇表和软件下载中心。无论您遇到任何技术问题，都可以在这里找到解决方案和相关资源。

## 章节导航

- **故障排查与错误码**：常见问题的诊断和解决方法
- **术语词汇表**：灵巧手相关的专业术语解释
- **下载中心**：SDK、固件、驱动程序和技术文档下载

\`,'''
}

# 为每个一级tab添加content
for section_id, intro_content in section_intros.items():
    # 查找一级tab的位置（有children但没有content的）
    pattern = rf'(id: {section_id},\s*title: "[^"]+",\s*path: "[^"]+",)\s*(children: \[)'
    replacement = rf'\1\n    {intro_content}\n    \2'
    content = re.sub(pattern, replacement, content)

# 写回文件
with open('client/src/lib/docs.ts', 'w', encoding='utf-8') as f:
    f.write(content)

print("已为一级tab添加总括内容")
