#!/bin/bash

# 创建关于XHAND1章节的文件
touch about/features.md about/specs.md about/comparison.md

# 创建快速入门章节的文件
touch quickstart/unboxing.md quickstart/host-software.md quickstart/first-test.md

# 创建SDK开发指南章节的文件
touch sdk/overview.md sdk/python.md sdk/cpp.md sdk/ros1.md sdk/ros2.md

# 创建遥操作章节的文件
touch teleoperation/overview.md teleoperation/meta-quest.md teleoperation/apple-vision-pro.md teleoperation/manus-glove.md

# 创建应用实践章节的文件
touch applications/grasping.md applications/tactile.md applications/advanced.md

# 创建支持与资源章节的文件
touch support/troubleshooting.md support/glossary.md support/downloads.md

# 创建FAQ文件
touch ../faq.md

echo "所有Markdown文件已创建"
