export interface DocSection {
  id: string;
  title: string;
  path: string;
  children?: DocSection[];
  content?: string;
}

export const docsData: DocSection[] = [
  {
    id: "about",
    title: "关于 XHAND1",
    path: "/docs/about",
    children: [
      {
        id: "overview",
        title: "产品概述",
        path: "/docs/about/overview",
        content: `# 产品概述

**XHAND1** 是星动量科技推出的全驱高仿生灵巧手，代表了当前机器人灵巧操作领域的前沿技术水平。

## 核心亮点

XHAND1 采用创新的全驱（Fully-actuated）设计理念，每个关节都配备独立的驱动单元，实现了对手部动作的精确控制。相比传统的欠驱（Under-actuated）方案，XHAND1 能够完成更加复杂和精细的操作任务。

**12个主动自由度**：覆盖拇指、食指、中指、无名指和小指的所有关键关节，实现了接近人手的灵活性。

**270度环绕式触觉传感**：创新的触觉传感器布局，提供高分辨率的接触力反馈，让机器人能够"感知"物体的形状、材质和滑动状态。

**80N强大抓握力**：在保持轻量化设计的同时，提供足够的力量完成各种操作任务，从精密装配到重物搬运。

## 应用场景

XHAND1 广泛应用于人形机器人、工业自动化、科研教学等领域，是实现机器人灵巧操作的理想选择。`
      },
      {
        id: "features",
        title: "核心特性与优势",
        path: "/docs/about/features",
        content: `# 核心特性与优势

## 全驱设计 (Fully-Actuated)

XHAND1 采用全驱设计，每个关节都配备独立的驱动单元。这种设计带来了以下优势：

- **精确控制**：每个关节的位置、速度和力矩都可以独立控制
- **复杂动作**：能够完成传统欠驱手无法实现的复杂操作
- **更好的稳定性**：在抓取和操作过程中提供更好的稳定性和可预测性

## 高分辨率触觉感知

**270度环绕式触觉传感器**覆盖手指的关键接触区域，提供：

- 高精度的接触力测量
- 物体形状和材质的感知能力
- 滑动检测和防滑控制
- 实时触觉反馈

## 强大的抓握能力

- **最大抓握力**：80N
- **额定负载**：1.5kg
- **重复定位精度**：±0.5mm

## 准直驱电机技术

采用准直驱（Quasi-Direct Drive）电机技术，结合了直驱和减速器的优点：

- 低惯量，快速响应
- 高扭矩密度
- 良好的力控性能
- 低噪音运行

## 多种控制模式

支持多种控制模式，适应不同的应用场景：

- **位置控制模式**：精确的位置控制
- **力控模式**：基于力反馈的柔顺控制
- **混合控制模式**：位置和力的混合控制`
      },
      {
        id: "specs",
        title: "技术规格",
        path: "/docs/about/specs",
        content: `# 技术规格

## 基本参数

| 参数 | 数值 |
|------|------|
| **自由度** | 12 DoF (主动) |
| **重量** | 约 1.2 kg |
| **尺寸** | 220mm × 110mm × 60mm |
| **额定负载** | 1.5 kg |
| **最大抓握力** | 80 N |
| **重复定位精度** | ±0.5 mm |

## 电气参数

| 参数 | 数值 |
|------|------|
| **供电电压** | 24V DC |
| **额定功率** | 120W |
| **峰值功率** | 200W |
| **通信接口** | EtherCAT / RS485 |
| **通信速率** | 100 Mbps (EtherCAT) |

## 关节参数

| 关节 | 活动范围 | 最大扭矩 |
|------|----------|----------|
| **拇指CMC** | 0° ~ 90° | 2.5 N·m |
| **拇指MCP** | 0° ~ 90° | 2.0 N·m |
| **拇指IP** | 0° ~ 90° | 1.5 N·m |
| **食指MCP** | 0° ~ 90° | 2.0 N·m |
| **食指PIP** | 0° ~ 100° | 1.5 N·m |
| **中指MCP** | 0° ~ 90° | 2.0 N·m |
| **中指PIP** | 0° ~ 100° | 1.5 N·m |
| **无名指MCP** | 0° ~ 90° | 2.0 N·m |
| **无名指PIP** | 0° ~ 100° | 1.5 N·m |
| **小指MCP** | 0° ~ 90° | 1.8 N·m |
| **小指PIP** | 0° ~ 100° | 1.3 N·m |

## 触觉传感器

| 参数 | 数值 |
|------|------|
| **传感器数量** | 16个 |
| **覆盖范围** | 270度环绕 |
| **采样频率** | 1000 Hz |
| **力测量范围** | 0 ~ 20 N |
| **分辨率** | 0.1 N |

## 环境参数

| 参数 | 数值 |
|------|------|
| **工作温度** | -10°C ~ 50°C |
| **存储温度** | -20°C ~ 60°C |
| **防护等级** | IP40 |`
      },
      {
        id: "comparison",
        title: "XHAND1 vs XHAND1 Lite",
        path: "/docs/about/comparison",
        content: `# XHAND1 vs XHAND1 Lite 产品对比

## 产品定位

**XHAND1** 是旗舰版本，提供完整的功能和最高的性能，适用于科研、高端应用和复杂操作场景。

**XHAND1 Lite** 是轻量版本，在保持核心功能的同时，简化了部分配置，更适合教育、入门级应用和成本敏感的项目。

## 详细对比

| 特性 | XHAND1 | XHAND1 Lite |
|------|--------|-------------|
| **自由度** | 12 DoF | 6 DoF |
| **触觉传感器** | 16个（270度环绕） | 8个（关键位置） |
| **最大抓握力** | 80 N | 50 N |
| **控制模式** | 位控/力控/混合 | 位控 |
| **通信接口** | EtherCAT + RS485 | RS485 |
| **重量** | 1.2 kg | 0.8 kg |
| **价格** | 标准 | 经济型 |
| **适用场景** | 科研、高端应用 | 教育、入门应用 |

## 选型建议

**选择 XHAND1 如果您需要：**
- 完整的12自由度灵巧操作
- 高精度的力控能力
- 全方位的触觉感知
- 高性能的EtherCAT通信
- 复杂的操作任务

**选择 XHAND1 Lite 如果您需要：**
- 基础的抓取和操作功能
- 更轻量化的设计
- 更经济的解决方案
- 教育和入门级应用
- 快速原型开发`
      }
    ]
  },
  {
    id: "quickstart",
    title: "快速入门",
    path: "/docs/quickstart",
    children: [
      {
        id: "unboxing",
        title: "开箱与安全须知",
        path: "/docs/quickstart/unboxing",
        content: `# 开箱与安全须知

## 装箱清单

请确认包装内包含以下物品：

- XHAND1 灵巧手主体 × 1
- 电源适配器（24V/5A） × 1
- 通信线缆（EtherCAT/RS485） × 1
- 安装支架 × 1
- 快速入门指南 × 1
- 保修卡 × 1

## 安全须知

⚠️ **重要提示**：在使用XHAND1之前，请仔细阅读以下安全须知。

### 电气安全

- 使用官方提供的24V电源适配器，切勿使用其他电压的电源
- 确保电源线连接牢固，避免松动导致接触不良
- 在潮湿环境中使用时，注意防水防潮
- 发现异常发热、异味或冒烟时，立即断电并联系技术支持

### 机械安全

- XHAND1 具有强大的抓握力（最大80N），操作时注意避免夹伤
- 首次使用时，建议在安全区域进行测试
- 避免在手指运动范围内放置易碎物品
- 定期检查机械部件，确保螺丝紧固

### 操作安全

- 在调试和开发过程中，建议使用较低的速度和力矩限制
- 避免让手指超出其机械限位范围
- 在无人看管的情况下，请关闭电源
- 儿童使用时需要成人监护

## 存储与维护

- 存储温度：-20°C ~ 60°C
- 避免阳光直射和高温环境
- 定期清洁手指表面，避免灰尘积累
- 长期不使用时，建议每月通电运行一次`
      },
      {
        id: "connection",
        title: "硬件连接指南",
        path: "/docs/quickstart/connection",
        content: `# 硬件连接指南

## 接口位置

XHAND1 的主要接口位于手腕底部，包括：

1. **电源接口**（DC 24V）
2. **EtherCAT 接口**（RJ45）
3. **RS485 接口**（4针端子）
4. **调试接口**（USB Type-C）

## 连接步骤

### 1. 电源连接

1. 将24V电源适配器的DC插头插入XHAND1的电源接口
2. 确保连接牢固，听到"咔哒"声
3. 将电源适配器插入220V交流电源插座

### 2. 通信连接

#### 选项A：EtherCAT连接（推荐）

1. 使用标准的CAT5e或CAT6网线
2. 一端连接XHAND1的EtherCAT接口
3. 另一端连接到您的EtherCAT主站设备（如工控机、实时控制器）
4. 确保网线连接指示灯亮起

#### 选项B：RS485连接

1. 使用4针端子线缆
2. 按照以下引脚定义连接：
   - Pin 1: RS485-A+
   - Pin 2: RS485-B-
   - Pin 3: GND
   - Pin 4: NC（不连接）
3. 连接到您的RS485主站设备

### 3. 调试接口连接（可选）

1. 使用USB Type-C线缆
2. 连接XHAND1的调试接口和计算机
3. 用于固件更新和底层调试

## 接口引脚定义

### 电源接口（DC插座）

| 引脚 | 定义 | 说明 |
|------|------|------|
| 中心针 | +24V | 正极 |
| 外壳 | GND | 负极/地 |

### EtherCAT接口（RJ45）

标准以太网接口，支持100Mbps通信速率。

### RS485接口（4针端子）

| 引脚 | 定义 | 说明 |
|------|------|------|
| 1 | A+ | RS485差分信号正 |
| 2 | B- | RS485差分信号负 |
| 3 | GND | 信号地 |
| 4 | NC | 不连接 |

## 连接验证

连接完成后，进行以下验证：

1. **电源指示**：手腕处的LED应该亮起绿色
2. **通信指示**：EtherCAT接口的指示灯应该闪烁
3. **初始化**：手指会进行一次自检动作（轻微摆动）

如果出现红色LED或无反应，请检查连接并参考故障排查章节。`
      },
      {
        id: "environment",
        title: "开发环境配置",
        path: "/docs/quickstart/environment",
        content: `# 开发环境配置

## 系统要求

### 硬件要求

- **处理器**：Intel Core i5 或更高
- **内存**：8GB RAM 或更高
- **存储**：至少10GB可用空间
- **网络接口**：支持EtherCAT的网卡（推荐Intel I210/I350系列）

### 软件要求

- **操作系统**：
  - Ubuntu 20.04 / 22.04 LTS（推荐）
  - Windows 10/11（部分功能支持）
  - macOS 12+（仅Python SDK）

- **开发工具**：
  - Python 3.8+
  - GCC 9.0+ 或 Clang 10.0+（C++开发）
  - ROS Noetic / ROS2 Humble（机器人集成）

## Python 环境配置

### 1. 安装 Conda（推荐）

\`\`\`bash
# 下载 Miniconda
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# 安装
bash Miniconda3-latest-Linux-x86_64.sh

# 创建虚拟环境
conda create -n xhand python=3.10
conda activate xhand
\`\`\`

### 2. 安装 XHAND SDK

\`\`\`bash
# 从PyPI安装（推荐）
pip install xhand-control-sdk

# 或从源码安装
git clone https://github.com/your-org/xhand_control_sdk_py.git
cd xhand_control_sdk_py
pip install -e .
\`\`\`

### 3. 验证安装

\`\`\`python
import xhand_control_sdk as xhand

print(f"XHAND SDK版本: {xhand.__version__}")
\`\`\`

## C++ 环境配置

### 1. 安装依赖

\`\`\`bash
# Ubuntu
sudo apt-get update
sudo apt-get install build-essential cmake git

# 安装EtherCAT库
sudo apt-get install libethercat-dev
\`\`\`

### 2. 编译 XHAND SDK

\`\`\`bash
git clone https://github.com/your-org/xhand_control_sdk_cpp.git
cd xhand_control_sdk_cpp
mkdir build && cd build
cmake ..
make -j4
sudo make install
\`\`\`

## ROS 环境配置

### ROS Noetic (Ubuntu 20.04)

\`\`\`bash
# 安装ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update
sudo apt-get install ros-noetic-desktop-full

# 创建工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 克隆XHAND ROS包
git clone https://github.com/your-org/xhand_control_ros.git

# 编译
cd ~/catkin_ws
catkin_make

# 配置环境变量
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
\`\`\`

## 网络配置（EtherCAT）

### 1. 识别网卡

\`\`\`bash
ifconfig
# 或
ip addr show
\`\`\`

### 2. 配置静态IP（可选）

\`\`\`bash
sudo nano /etc/netplan/01-netcfg.yaml
\`\`\`

添加以下配置：

\`\`\`yaml
network:
  version: 2
  ethernets:
    eth0:  # 替换为您的网卡名称
      addresses: [192.168.1.100/24]
      dhcp4: no
\`\`\`

应用配置：

\`\`\`bash
sudo netplan apply
\`\`\`

## 故障排查

### Python SDK 导入失败

\`\`\`bash
# 检查Python版本
python --version

# 检查安装路径
pip show xhand-control-sdk

# 重新安装
pip uninstall xhand-control-sdk
pip install --no-cache-dir xhand-control-sdk
\`\`\`

### EtherCAT 连接失败

\`\`\`bash
# 检查网卡驱动
lsmod | grep ec_

# 检查权限
sudo chmod 666 /dev/EtherCAT0

# 查看EtherCAT状态
ethercat slaves
\`\`\``
      },
      {
        id: "hello-world",
        title: '运行第一个程序 ("Hello, XHAND!")',
        path: "/docs/quickstart/hello-world",
        content: `# 运行第一个程序 ("Hello, XHAND!")

## 目标

在本教程中，您将学习如何编写并运行第一个XHAND1控制程序，让灵巧手完成一个简单的"握拳"动作。

## Python 示例

### 1. 创建脚本文件

创建一个名为 \`hello_xhand.py\` 的文件：

\`\`\`python
#!/usr/bin/env python3
"""
XHAND1 Hello World 示例
让灵巧手完成一个握拳动作
"""

import time
import xhand_control_sdk as xhand

def main():
    # 1. 初始化XHAND控制器
    print("正在连接XHAND1...")
    hand = xhand.XHandController(
        interface="eth0",  # 替换为您的网卡名称
        hand_id=0          # 手部ID，默认为0
    )
    
    # 2. 等待连接建立
    if not hand.connect(timeout=5.0):
        print("连接失败！请检查硬件连接。")
        return
    
    print("连接成功！")
    
    # 3. 切换到位置控制模式
    hand.set_control_mode(xhand.ControlMode.POSITION)
    
    # 4. 设置目标位置（握拳）
    # 所有手指弯曲到90度
    target_positions = {
        "thumb_cmc": 45,      # 拇指根部
        "thumb_mcp": 45,      # 拇指掌指关节
        "thumb_ip": 45,       # 拇指指间关节
        "index_mcp": 90,      # 食指掌指关节
        "index_pip": 90,      # 食指近端指间关节
        "middle_mcp": 90,     # 中指掌指关节
        "middle_pip": 90,     # 中指近端指间关节
        "ring_mcp": 90,       # 无名指掌指关节
        "ring_pip": 90,       # 无名指近端指间关节
        "pinky_mcp": 90,      # 小指掌指关节
        "pinky_pip": 90,      # 小指近端指间关节
    }
    
    print("开始握拳...")
    hand.set_joint_positions(target_positions)
    
    # 5. 等待动作完成
    time.sleep(2.0)
    
    print("握拳完成！")
    
    # 6. 返回初始位置（张开手）
    print("张开手...")
    hand.set_joint_positions_all(0)  # 所有关节回到0度
    time.sleep(2.0)
    
    print("动作完成！")
    
    # 7. 断开连接
    hand.disconnect()
    print("已断开连接。")

if __name__ == "__main__":
    main()
\`\`\`

### 2. 运行程序

\`\`\`bash
# 确保已激活虚拟环境
conda activate xhand

# 运行脚本
python hello_xhand.py
\`\`\`

### 3. 预期输出

\`\`\`
正在连接XHAND1...
连接成功！
开始握拳...
握拳完成！
张开手...
动作完成！
已断开连接。
\`\`\`

## C++ 示例

### 1. 创建源文件

创建 \`hello_xhand.cpp\`：

\`\`\`cpp
#include <iostream>
#include <thread>
#include <chrono>
#include <xhand_control_sdk/xhand_controller.h>

int main() {
    // 1. 初始化控制器
    std::cout << "正在连接XHAND1..." << std::endl;
    xhand::XHandController hand("eth0", 0);
    
    // 2. 连接
    if (!hand.connect(5.0)) {
        std::cerr << "连接失败！" << std::endl;
        return 1;
    }
    std::cout << "连接成功！" << std::endl;
    
    // 3. 设置控制模式
    hand.setControlMode(xhand::ControlMode::POSITION);
    
    // 4. 握拳
    std::cout << "开始握拳..." << std::endl;
    std::map<std::string, double> positions = {
        {"thumb_cmc", 45}, {"thumb_mcp", 45}, {"thumb_ip", 45},
        {"index_mcp", 90}, {"index_pip", 90},
        {"middle_mcp", 90}, {"middle_pip", 90},
        {"ring_mcp", 90}, {"ring_pip", 90},
        {"pinky_mcp", 90}, {"pinky_pip", 90}
    };
    hand.setJointPositions(positions);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "握拳完成！" << std::endl;
    
    // 5. 张开手
    std::cout << "张开手..." << std::endl;
    hand.setJointPositionsAll(0.0);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    std::cout << "动作完成！" << std::endl;
    
    // 6. 断开连接
    hand.disconnect();
    return 0;
}
\`\`\`

### 2. 编译运行

\`\`\`bash
g++ -o hello_xhand hello_xhand.cpp -lxhand_control_sdk -std=c++17
./hello_xhand
\`\`\`

## 常见问题

### Q: 连接失败怎么办？

A: 请检查：
1. 硬件连接是否正确
2. 网卡名称是否正确（使用 \`ifconfig\` 查看）
3. 是否有足够的权限（可能需要 \`sudo\`）
4. EtherCAT驱动是否正常加载

### Q: 手指不动或动作不正确？

A: 请检查：
1. 控制模式是否正确设置
2. 关节角度是否在有效范围内（0-90度）
3. 是否有机械卡死或异常

### Q: 如何调整动作速度？

A: 可以通过设置速度限制参数：

\`\`\`python
hand.set_velocity_limit(50)  # 设置最大速度为50度/秒
\`\`\`

## 下一步

恭喜您完成了第一个XHAND1程序！接下来您可以：

- 学习更多的控制模式（力控、混合控制）
- 探索触觉传感器的使用
- 尝试更复杂的抓取动作
- 集成到您的机器人系统中`
      }
    ]
  },
  {
    id: "sdk",
    title: "SDK 开发指南",
    path: "/docs/sdk",
    children: [
      {
        id: "overview",
        title: "SDK 架构概述",
        path: "/docs/sdk/overview",
        content: `# SDK 架构概述

## 整体架构

XHAND1 SDK 采用分层架构设计，从底层硬件驱动到高层应用接口，提供了完整的开发支持。

\`\`\`
┌─────────────────────────────────────────┐
│        应用层 (Application Layer)        │
│   Python/C++/ROS/ROS2 应用程序           │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│         API层 (API Layer)               │
│   Python SDK / C++ SDK / ROS Wrapper    │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│      中间件层 (Middleware Layer)         │
│   通信管理 / 数据处理 / 状态机           │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│      驱动层 (Driver Layer)              │
│   EtherCAT Master / RS485 Driver        │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│      硬件层 (Hardware Layer)            │
│   XHAND1 灵巧手硬件                      │
└─────────────────────────────────────────┘
\`\`\`

## 核心组件

### 1. 通信管理器 (Communication Manager)

负责与XHAND1硬件的底层通信，支持：

- **EtherCAT通信**：高速实时通信，适用于高性能应用
- **RS485通信**：简单可靠，适用于一般应用
- **自动重连**：网络中断后自动恢复连接
- **数据校验**：CRC校验确保数据完整性

### 2. 控制器 (Controller)

提供多种控制模式：

- **位置控制**：精确的关节位置控制
- **速度控制**：关节速度控制
- **力矩控制**：基于力矩的控制
- **混合控制**：位置和力的混合控制

### 3. 传感器管理器 (Sensor Manager)

管理所有传感器数据：

- **关节位置传感器**：实时关节角度
- **关节速度传感器**：关节运动速度
- **触觉传感器**：16个触觉传感器的力数据
- **温度传感器**：关节电机温度监控

### 4. 安全管理器 (Safety Manager)

确保系统安全运行：

- **限位保护**：防止关节超出机械限位
- **过流保护**：电机过流自动保护
- **过温保护**：温度过高自动降速或停止
- **急停功能**：紧急情况下快速停止

## 数据流

### 命令数据流（下行）

\`\`\`
应用程序
    ↓ 发送控制命令
API层（格式化命令）
    ↓ 转换为底层协议
中间件层（添加校验、优先级）
    ↓ 打包数据
驱动层（EtherCAT/RS485发送）
    ↓ 物理传输
XHAND1硬件
\`\`\`

### 状态数据流（上行）

\`\`\`
XHAND1硬件
    ↓ 采集传感器数据
驱动层（接收原始数据）
    ↓ 解析数据包
中间件层（数据校验、滤波）
    ↓ 转换为标准格式
API层（封装为对象）
    ↓ 回调或查询
应用程序
\`\`\`

## 多语言支持

### Python SDK

- 简单易用，适合快速原型开发
- 完整的API文档和类型提示
- 丰富的示例代码

### C++ SDK

- 高性能，适合实时控制
- 零拷贝数据传输
- 支持多线程

### ROS/ROS2

- 标准的ROS消息和服务
- 与其他ROS节点无缝集成
- 支持可视化和调试工具

## 开发模式

### 同步模式

适用于简单的控制任务，代码逻辑清晰：

\`\`\`python
hand.set_position(90)  # 阻塞直到完成
hand.set_position(0)
\`\`\`

### 异步模式

适用于复杂的并发任务，性能更高：

\`\`\`python
future = hand.set_position_async(90)
# 做其他事情...
future.wait()  # 等待完成
\`\`\`

### 回调模式

适用于事件驱动的应用：

\`\`\`python
def on_position_reached(joint_id):
    print(f"关节 {joint_id} 到达目标位置")

hand.register_callback("position_reached", on_position_reached)
hand.set_position(90)
\`\`\``
      },
      {
        id: "python",
        title: "Python SDK",
        path: "/docs/sdk/python",
        content: `# Python SDK

## 安装

### 从 PyPI 安装（推荐）

\`\`\`bash
pip install xhand-control-sdk
\`\`\`

### 从源码安装

\`\`\`bash
git clone https://github.com/your-org/xhand_control_sdk_py.git
cd xhand_control_sdk_py
pip install -e .
\`\`\`

## 快速开始

### 基本使用

\`\`\`python
import xhand_control_sdk as xhand

# 创建控制器
hand = xhand.XHandController(interface="eth0")

# 连接
hand.connect()

# 设置控制模式
hand.set_control_mode(xhand.ControlMode.POSITION)

# 控制手指
hand.set_joint_position("index_mcp", 90)

# 读取状态
position = hand.get_joint_position("index_mcp")
print(f"当前位置: {position}度")

# 断开连接
hand.disconnect()
\`\`\`

## 核心类和方法

### XHandController

主控制器类，提供所有控制功能。

#### 构造函数

\`\`\`python
XHandController(
    interface: str = "eth0",
    hand_id: int = 0,
    protocol: str = "ethercat"
)
\`\`\`

**参数：**
- \`interface\`: 网络接口名称
- \`hand_id\`: 手部ID（支持多手控制）
- \`protocol\`: 通信协议（"ethercat" 或 "rs485"）

#### 连接管理

\`\`\`python
# 连接到硬件
connect(timeout: float = 5.0) -> bool

# 断开连接
disconnect() -> None

# 检查连接状态
is_connected() -> bool
\`\`\`

#### 控制模式

\`\`\`python
# 设置控制模式
set_control_mode(mode: ControlMode) -> None

# 获取当前控制模式
get_control_mode() -> ControlMode
\`\`\`

**控制模式枚举：**
- \`ControlMode.POSITION\`: 位置控制
- \`ControlMode.VELOCITY\`: 速度控制
- \`ControlMode.TORQUE\`: 力矩控制
- \`ControlMode.FORCE\`: 力控制

#### 位置控制

\`\`\`python
# 设置单个关节位置
set_joint_position(
    joint_name: str,
    position: float,
    velocity: float = None,
    blocking: bool = False
) -> None

# 设置多个关节位置
set_joint_positions(
    positions: Dict[str, float],
    blocking: bool = False
) -> None

# 设置所有关节到同一位置
set_joint_positions_all(
    position: float,
    blocking: bool = False
) -> None

# 获取关节位置
get_joint_position(joint_name: str) -> float

# 获取所有关节位置
get_joint_positions() -> Dict[str, float]
\`\`\`

#### 力控制

\`\`\`python
# 设置关节力矩
set_joint_torque(joint_name: str, torque: float) -> None

# 设置抓握力
set_grasp_force(force: float) -> None

# 获取关节力矩
get_joint_torque(joint_name: str) -> float
\`\`\`

#### 触觉传感器

\`\`\`python
# 获取单个传感器数据
get_tactile_sensor(sensor_id: int) -> float

# 获取所有传感器数据
get_tactile_sensors() -> List[float]

# 获取接触状态
is_in_contact() -> bool
\`\`\`

#### 预设动作

\`\`\`python
# 执行预设动作
execute_preset(preset_name: str) -> None
\`\`\`

**预设动作列表：**
- \`"open"\`: 张开手
- \`"close"\`: 握拳
- \`"pinch"\`: 捏取姿态
- \`"ok"\`: OK手势
- \`"peace"\`: V字手势
- \`"thumbs_up"\`: 点赞手势

## 代码示例

### 示例1：基本抓取

\`\`\`python
import xhand_control_sdk as xhand
import time

hand = xhand.XHandController()
hand.connect()

# 张开手
hand.execute_preset("open")
time.sleep(1)

# 握拳
hand.execute_preset("close")
time.sleep(1)

# 张开手
hand.execute_preset("open")

hand.disconnect()
\`\`\`

### 示例2：力控抓取

\`\`\`python
import xhand_control_sdk as xhand

hand = xhand.XHandController()
hand.connect()

# 切换到力控模式
hand.set_control_mode(xhand.ControlMode.FORCE)

# 设置抓握力为10N
hand.set_grasp_force(10.0)

# 等待抓取稳定
time.sleep(2)

# 检查是否接触到物体
if hand.is_in_contact():
    print("成功抓取物体")
else:
    print("未检测到物体")

hand.disconnect()
\`\`\`

### 示例3：触觉反馈

\`\`\`python
import xhand_control_sdk as xhand
import time

hand = xhand.XHandController()
hand.connect()

# 持续读取触觉数据
try:
    while True:
        sensors = hand.get_tactile_sensors()
        print(f"触觉传感器数据: {sensors}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("停止读取")

hand.disconnect()
\`\`\`

## API 参考

完整的API文档请访问：[Python SDK API Reference](/docs/api/python)`
      },
      {
        id: "cpp",
        title: "C++ SDK",
        path: "/docs/sdk/cpp",
        content: `# C++ SDK

## 安装

### 系统要求

- C++17 或更高版本
- CMake 3.10+
- EtherCAT库（libethercat）

### 编译安装

\`\`\`bash
# 克隆仓库
git clone https://github.com/your-org/xhand_control_sdk_cpp.git
cd xhand_control_sdk_cpp

# 创建构建目录
mkdir build && cd build

# 配置
cmake ..

# 编译
make -j4

# 安装
sudo make install
\`\`\`

## 快速开始

### 基本使用

\`\`\`cpp
#include <xhand_control_sdk/xhand_controller.h>
#include <iostream>

int main() {
    // 创建控制器
    xhand::XHandController hand("eth0");
    
    // 连接
    if (!hand.connect()) {
        std::cerr << "连接失败" << std::endl;
        return 1;
    }
    
    // 设置控制模式
    hand.setControlMode(xhand::ControlMode::POSITION);
    
    // 控制手指
    hand.setJointPosition("index_mcp", 90.0);
    
    // 读取状态
    double position = hand.getJointPosition("index_mcp");
    std::cout << "当前位置: " << position << "度" << std::endl;
    
    // 断开连接
    hand.disconnect();
    
    return 0;
}
\`\`\`

### 编译应用程序

\`\`\`bash
g++ -o my_app my_app.cpp -lxhand_control_sdk -std=c++17
\`\`\`

或使用CMake：

\`\`\`cmake
cmake_minimum_required(VERSION 3.10)
project(my_xhand_app)

set(CMAKE_CXX_STANDARD 17)

find_package(xhand_control_sdk REQUIRED)

add_executable(my_app my_app.cpp)
target_link_libraries(my_app xhand_control_sdk::xhand_control_sdk)
\`\`\`

## 核心类和方法

### XHandController

\`\`\`cpp
namespace xhand {

class XHandController {
public:
    // 构造函数
    XHandController(
        const std::string& interface = "eth0",
        int hand_id = 0,
        Protocol protocol = Protocol::ETHERCAT
    );
    
    // 连接管理
    bool connect(double timeout = 5.0);
    void disconnect();
    bool isConnected() const;
    
    // 控制模式
    void setControlMode(ControlMode mode);
    ControlMode getControlMode() const;
    
    // 位置控制
    void setJointPosition(
        const std::string& joint_name,
        double position,
        double velocity = -1.0,
        bool blocking = false
    );
    
    void setJointPositions(
        const std::map<std::string, double>& positions,
        bool blocking = false
    );
    
    void setJointPositionsAll(double position, bool blocking = false);
    
    double getJointPosition(const std::string& joint_name) const;
    std::map<std::string, double> getJointPositions() const;
    
    // 力控制
    void setJointTorque(const std::string& joint_name, double torque);
    void setGraspForce(double force);
    double getJointTorque(const std::string& joint_name) const;
    
    // 触觉传感器
    double getTactileSensor(int sensor_id) const;
    std::vector<double> getTactileSensors() const;
    bool isInContact() const;
    
    // 预设动作
    void executePreset(const std::string& preset_name);
};

} // namespace xhand
\`\`\`

## 代码示例

### 示例1：实时控制循环

\`\`\`cpp
#include <xhand_control_sdk/xhand_controller.h>
#include <thread>
#include <chrono>

int main() {
    xhand::XHandController hand;
    hand.connect();
    hand.setControlMode(xhand::ControlMode::POSITION);
    
    // 实时控制循环（1kHz）
    auto start_time = std::chrono::steady_clock::now();
    const double frequency = 1000.0;  // Hz
    const double period = 1.0 / frequency;
    
    for (int i = 0; i < 10000; ++i) {
        // 计算目标位置（正弦波）
        double t = i * period;
        double target = 45.0 * std::sin(2 * M_PI * 0.5 * t) + 45.0;
        
        // 发送命令
        hand.setJointPosition("index_mcp", target);
        
        // 精确延时
        auto next_time = start_time + 
            std::chrono::microseconds(static_cast<long>(i * period * 1e6));
        std::this_thread::sleep_until(next_time);
    }
    
    hand.disconnect();
    return 0;
}
\`\`\`

### 示例2：多线程数据采集

\`\`\`cpp
#include <xhand_control_sdk/xhand_controller.h>
#include <thread>
#include <atomic>
#include <fstream>

std::atomic<bool> running(true);

void data_acquisition_thread(xhand::XHandController& hand) {
    std::ofstream log_file("sensor_data.csv");
    log_file << "timestamp,sensor0,sensor1,...,sensor15\\n";
    
    while (running) {
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()
        ).count();
        
        auto sensors = hand.getTactileSensors();
        
        log_file << timestamp;
        for (double value : sensors) {
            log_file << "," << value;
        }
        log_file << "\\n";
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    log_file.close();
}

int main() {
    xhand::XHandController hand;
    hand.connect();
    
    // 启动数据采集线程
    std::thread acq_thread(data_acquisition_thread, std::ref(hand));
    
    // 主线程执行控制任务
    hand.setControlMode(xhand::ControlMode::POSITION);
    hand.executePreset("close");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    hand.executePreset("open");
    
    // 停止数据采集
    running = false;
    acq_thread.join();
    
    hand.disconnect();
    return 0;
}
\`\`\`

### 示例3：异常处理

\`\`\`cpp
#include <xhand_control_sdk/xhand_controller.h>
#include <xhand_control_sdk/exceptions.h>
#include <iostream>

int main() {
    try {
        xhand::XHandController hand;
        
        if (!hand.connect(5.0)) {
            throw xhand::ConnectionException("无法连接到XHAND1");
        }
        
        hand.setControlMode(xhand::ControlMode::POSITION);
        hand.setJointPosition("index_mcp", 90.0);
        
        // 检查是否到达目标位置
        double current = hand.getJointPosition("index_mcp");
        if (std::abs(current - 90.0) > 1.0) {
            throw xhand::ControlException("未能到达目标位置");
        }
        
        hand.disconnect();
        
    } catch (const xhand::ConnectionException& e) {
        std::cerr << "连接错误: " << e.what() << std::endl;
        return 1;
        
    } catch (const xhand::ControlException& e) {
        std::cerr << "控制错误: " << e.what() << std::endl;
        return 2;
        
    } catch (const std::exception& e) {
        std::cerr << "未知错误: " << e.what() << std::endl;
        return 3;
    }
    
    return 0;
}
\`\`\`

## 性能优化

### 零拷贝数据传输

\`\`\`cpp
// 使用引用避免拷贝
const std::vector<double>& sensors = hand.getTactileSensorsRef();

// 直接写入缓冲区
std::vector<double> buffer(16);
hand.getTactileSensors(buffer.data(), buffer.size());
\`\`\`

### 批量操作

\`\`\`cpp
// 批量设置关节位置（更高效）
std::map<std::string, double> positions = {
    {"index_mcp", 90}, {"index_pip", 90},
    {"middle_mcp", 90}, {"middle_pip", 90}
};
hand.setJointPositions(positions);

// 而不是逐个设置
// hand.setJointPosition("index_mcp", 90);
// hand.setJointPosition("index_pip", 90);
// ...
\`\`\`

## API 参考

完整的API文档请访问：[C++ SDK API Reference](/docs/api/cpp)`
      }
    ]
  },
  {
    id: "applications",
    title: "应用实践",
    path: "/docs/applications",
    children: [
      {
        id: "control-modes",
        title: "控制模式详解",
        path: "/docs/applications/control-modes",
        content: `# 控制模式详解

XHAND1 支持多种控制模式，每种模式适用于不同的应用场景。

## 位置控制模式 (Position Control)

### 原理

位置控制模式通过PID控制器，使关节精确地跟踪目标位置。这是最常用的控制模式，适用于大多数场景。

### 使用场景

- 精确的抓取和放置
- 预定义的手势和动作
- 轨迹跟踪
- 遥操作

### 代码示例

\`\`\`python
hand.set_control_mode(xhand.ControlMode.POSITION)

# 设置目标位置
hand.set_joint_position("index_mcp", 90)

# 设置速度限制（可选）
hand.set_velocity_limit(50)  # 度/秒
\`\`\`

### 参数调节

- **位置增益 (Kp)**：控制响应速度，过大会震荡
- **速度限制**：限制最大运动速度
- **加速度限制**：限制加速度，使运动更平滑

## 力控模式 (Force Control)

### 原理

力控模式通过控制关节力矩，实现基于力反馈的柔顺控制。适用于需要与环境交互的场景。

### 使用场景

- 柔顺抓取
- 力觉装配
- 接触力控制
- 物体操作

### 代码示例

\`\`\`python
hand.set_control_mode(xhand.ControlMode.FORCE)

# 设置抓握力
hand.set_grasp_force(10.0)  # 10N

# 监控接触力
while True:
    force = hand.get_contact_force()
    if force > 15.0:
        print("警告：接触力过大！")
        break
\`\`\`

### 力控策略

1. **恒力控制**：保持恒定的接触力
2. **阻抗控制**：模拟弹簧-阻尼系统
3. **导纳控制**：根据力调整位置

## 混合控制模式 (Hybrid Control)

### 原理

混合控制模式结合了位置控制和力控制，在不同的自由度上使用不同的控制策略。

### 使用场景

- 插拔任务（某些方向位置控制，某些方向力控制）
- 表面跟踪
- 复杂的操作任务

### 代码示例

\`\`\`python
hand.set_control_mode(xhand.ControlMode.HYBRID)

# 设置混合控制参数
# 手指弯曲方向：力控制
# 手指侧向：位置控制
hand.set_hybrid_control(
    position_axes=["lateral"],
    force_axes=["flexion"],
    target_position={"lateral": 0},
    target_force={"flexion": 10.0}
)
\`\`\``
      },
      {
        id: "grasping",
        title: "灵巧抓取与操作",
        path: "/docs/applications/grasping",
        content: `# 灵巧抓取与操作

## 预设抓取姿态

XHAND1 提供了多种预设的抓取姿态，可以直接调用。

### 可用姿态

| 姿态名称 | 描述 | 适用物体 |
|---------|------|---------|
| \`power_grasp\` | 力量抓取 | 大型、重物体 |
| \`precision_grasp\` | 精密抓取 | 小型、精密物体 |
| \`pinch\` | 捏取 | 薄片、小物体 |
| \`tripod\` | 三指抓取 | 球形、圆柱形物体 |
| \`lateral\` | 侧向抓取 | 卡片、钥匙 |

### 使用示例

\`\`\`python
# 执行预设姿态
hand.execute_preset("precision_grasp")

# 等待完成
hand.wait_for_completion()

# 检查是否成功抓取
if hand.is_in_contact():
    print("成功抓取物体")
\`\`\`

## 自适应抓取

自适应抓取能够根据物体的形状自动调整手指姿态。

\`\`\`python
# 启用自适应抓取
hand.enable_adaptive_grasping()

# 开始抓取
hand.adaptive_grasp(
    approach_distance=50,  # 接近距离（mm）
    max_force=15.0,        # 最大抓握力（N）
    timeout=5.0            # 超时时间（秒）
)
\`\`\`

## 基于触觉的抓取

利用触觉传感器实现更智能的抓取。

\`\`\`python
def tactile_guided_grasp(hand):
    # 慢速闭合手指
    hand.set_control_mode(xhand.ControlMode.FORCE)
    hand.set_grasp_force(5.0)
    
    # 监控触觉传感器
    while True:
        sensors = hand.get_tactile_sensors()
        
        # 检查是否所有手指都接触到物体
        if all(s > 0.5 for s in sensors):
            print("检测到物体，停止闭合")
            break
        
        time.sleep(0.01)
    
    # 增加抓握力
    hand.set_grasp_force(10.0)
    print("抓取完成")

tactile_guided_grasp(hand)
\`\`\``
      },
      {
        id: "tactile",
        title: "触觉感知应用",
        path: "/docs/applications/tactile",
        content: `# 触觉感知应用

## 触觉传感器布局

XHAND1 配备了16个高精度触觉传感器，覆盖手指的关键接触区域。

### 传感器位置

| 传感器ID | 位置 | 覆盖区域 |
|---------|------|---------|
| 0-2 | 拇指 | 指尖、指腹、侧面 |
| 3-5 | 食指 | 指尖、指腹、侧面 |
| 6-8 | 中指 | 指尖、指腹、侧面 |
| 9-11 | 无名指 | 指尖、指腹、侧面 |
| 12-14 | 小指 | 指尖、指腹、侧面 |
| 15 | 手掌 | 手掌中心 |

## 读取触觉数据

\`\`\`python
# 读取单个传感器
force = hand.get_tactile_sensor(0)  # 拇指指尖
print(f"拇指指尖接触力: {force} N")

# 读取所有传感器
sensors = hand.get_tactile_sensors()
for i, force in enumerate(sensors):
    print(f"传感器 {i}: {force} N")
\`\`\`

## 物体识别

基于触觉数据识别物体的形状和材质。

\`\`\`python
def identify_object(hand):
    # 执行探索性抓取
    hand.adaptive_grasp(max_force=10.0)
    
    # 收集触觉数据
    tactile_pattern = hand.get_tactile_sensors()
    
    # 简单的形状识别
    if tactile_pattern[0] > 5 and tactile_pattern[3] > 5:
        return "球形物体"
    elif max(tactile_pattern) < 3:
        return "柔软物体"
    else:
        return "不规则物体"

object_type = identify_object(hand)
print(f"识别结果: {object_type}")
\`\`\`

## 滑动检测

检测物体是否在手中滑动，并自动调整抓握力。

\`\`\`python
import numpy as np

def slip_detection(hand, threshold=0.5):
    # 记录初始触觉数据
    initial_tactile = np.array(hand.get_tactile_sensors())
    
    while True:
        current_tactile = np.array(hand.get_tactile_sensors())
        
        # 计算触觉变化率
        tactile_change = np.abs(current_tactile - initial_tactile)
        
        # 检测滑动
        if np.max(tactile_change) > threshold:
            print("检测到滑动！增加抓握力")
            current_force = hand.get_grasp_force()
            hand.set_grasp_force(current_force + 2.0)
        
        initial_tactile = current_tactile
        time.sleep(0.01)

# 启动滑动检测
slip_detection(hand)
\`\`\``
      }
    ]
  },
  {
    id: "faq",
    title: "常见问题 (FAQ)",
    path: "/docs/faq",
    content: `# 常见问题 (FAQ)

## 硬件相关

### Q: XHAND1 的最大抓握力是多少？

A: XHAND1 的最大抓握力为 **80N**，足以应对大多数操作任务。在实际使用中，建议根据物体的脆弱程度调整抓握力，避免损坏物体。

### Q: XHAND1 支持哪些通信协议？

A: XHAND1 支持两种通信协议：
- **EtherCAT**：高速实时通信，适用于高性能应用（推荐）
- **RS485**：简单可靠，适用于一般应用

### Q: 如何更换损坏的手指？

A: XHAND1 采用模块化设计，手指可以单独更换。请联系技术支持获取备件和更换指南。

## 软件相关

### Q: SDK 支持哪些操作系统？

A: 
- **Python SDK**：支持 Linux、Windows、macOS
- **C++ SDK**：支持 Linux、Windows
- **ROS/ROS2**：支持 Linux（Ubuntu 20.04/22.04）

### Q: 如何解决 "连接失败" 的问题？

A: 请按以下步骤排查：

1. 检查硬件连接是否正确
2. 确认网卡名称是否正确（使用 \`ifconfig\` 或 \`ip addr\` 查看）
3. 检查是否有足够的权限（可能需要 \`sudo\`）
4. 确认 EtherCAT 驱动是否正常加载
5. 尝试重启 XHAND1 和计算机

### Q: 为什么手指不动或动作不正确？

A: 可能的原因：

1. **控制模式不正确**：确保已设置正确的控制模式
2. **关节角度超出范围**：检查目标角度是否在有效范围内（0-90度）
3. **机械卡死**：检查是否有异物或机械故障
4. **电源不足**：确认电源适配器输出正常

## 控制相关

### Q: 如何调整手指运动速度？

A: 可以通过设置速度限制参数：

\`\`\`python
hand.set_velocity_limit(50)  # 设置最大速度为50度/秒
\`\`\`

### Q: 位置控制和力控制有什么区别？

A: 
- **位置控制**：精确控制关节位置，适用于需要精确定位的场景
- **力控制**：控制接触力，适用于需要与环境交互的场景

详细说明请参考 [控制模式详解](/docs/applications/control-modes)。

### Q: 如何实现柔顺抓取？

A: 使用力控模式，并设置合适的抓握力：

\`\`\`python
hand.set_control_mode(xhand.ControlMode.FORCE)
hand.set_grasp_force(10.0)  # 设置较小的力，实现柔顺抓取
\`\`\`

## 触觉传感器

### Q: 触觉传感器的采样频率是多少？

A: 触觉传感器的采样频率为 **1000 Hz**，能够实时捕捉快速的接触变化。

### Q: 如何校准触觉传感器？

A: 在无接触状态下运行校准程序：

\`\`\`python
hand.calibrate_tactile_sensors()
\`\`\`

### Q: 触觉传感器数据有噪声怎么办？

A: 可以使用滤波器平滑数据：

\`\`\`python
# 启用低通滤波器
hand.enable_tactile_filter(cutoff_freq=50)  # 50Hz截止频率
\`\`\`

## 集成相关

### Q: 如何将 XHAND1 集成到 ROS 系统？

A: 请参考 [ROS 集成指南](/docs/sdk/ros)。

### Q: 支持哪些仿真环境？

A: XHAND1 支持以下仿真环境：
- **Isaac Gym / Isaac Lab**
- **MuJoCo**
- **PyBullet**
- **Gazebo**（通过ROS）

### Q: 如何与遥操作设备集成？

A: XHAND1 支持与主流遥操作设备集成，如 MANUS 手套。详细说明请参考 [遥操作集成](/docs/applications/teleoperation)。

## 维护相关

### Q: 多久需要进行一次维护？

A: 建议：
- **日常使用**：每周检查一次螺丝紧固情况
- **重度使用**：每月进行一次全面检查
- **长期不使用**：每月通电运行一次

### Q: 如何清洁 XHAND1？

A: 
1. 断开电源
2. 使用干净的软布擦拭表面
3. 避免使用水或化学溶剂
4. 清洁触觉传感器表面时要特别小心

### Q: 固件如何更新？

A: 请参考 [固件更新指南](/docs/applications/firmware-update)。

## 技术支持

### Q: 如何获取技术支持？

A: 您可以通过以下方式获取技术支持：

- **邮箱**：support@xingdong.tech
- **电话**：+86-xxx-xxxx-xxxx
- **在线文档**：https://docs.xingdong.tech
- **GitHub Issues**：https://github.com/your-org/xhand-sdk/issues

### Q: 保修期是多久？

A: XHAND1 提供 **1年** 的质保服务。质保期内，非人为损坏的故障可以免费维修或更换。`
  },
  {
    id: "support",
    title: "支持与资源",
    path: "/docs/support",
    children: [
      {
        id: "troubleshooting",
        title: "故障排查与错误码",
        path: "/docs/support/troubleshooting",
        content: `# 故障排查与错误码

## 常见故障排查

### 连接问题

#### 症状：无法连接到 XHAND1

**可能原因和解决方案：**

1. **硬件连接问题**
   - 检查电源线和通信线是否连接牢固
   - 确认电源指示灯是否亮起
   - 尝试重新插拔连接线

2. **网卡配置问题**
   - 使用 \`ifconfig\` 或 \`ip addr\` 确认网卡名称
   - 检查网卡是否启用
   - 确认IP地址配置正确

3. **驱动问题**
   - 检查 EtherCAT 驱动是否加载：\`lsmod | grep ec_\`
   - 重新加载驱动：\`sudo modprobe ec_master\`

4. **权限问题**
   - 尝试使用 \`sudo\` 运行程序
   - 或添加用户到 \`ethercat\` 组

### 运动问题

#### 症状：手指不动或运动异常

**可能原因和解决方案：**

1. **控制模式错误**
   - 确认已设置正确的控制模式
   - 尝试切换到位置控制模式

2. **目标位置超出范围**
   - 检查目标角度是否在 0-90度 范围内
   - 查看关节限位表

3. **机械卡死**
   - 手动检查手指是否能自由活动
   - 检查是否有异物卡住

4. **电机过热保护**
   - 等待电机冷却
   - 降低运动速度和频率

### 传感器问题

#### 症状：触觉传感器数据异常

**可能原因和解决方案：**

1. **传感器未校准**
   - 运行校准程序：\`hand.calibrate_tactile_sensors()\`

2. **传感器损坏**
   - 检查传感器表面是否有损伤
   - 联系技术支持更换传感器

3. **数据噪声**
   - 启用滤波器：\`hand.enable_tactile_filter()\`
   - 检查接地是否良好

## 错误码参考

### 系统错误码 (0x0xxx)

| 错误码 | 名称 | 描述 | 解决方案 |
|-------|------|------|---------|
| 0x0001 | COMM_TIMEOUT | 通信超时 | 检查网络连接，重启设备 |
| 0x0002 | COMM_CRC_ERROR | CRC校验错误 | 检查通信线缆，降低通信速率 |
| 0x0003 | INVALID_COMMAND | 无效命令 | 检查命令格式，更新SDK |
| 0x0004 | DEVICE_NOT_READY | 设备未就绪 | 等待设备初始化完成 |

### 控制错误码 (0x1xxx)

| 错误码 | 名称 | 描述 | 解决方案 |
|-------|------|------|---------|
| 0x1001 | POSITION_LIMIT | 位置超限 | 调整目标位置到有效范围 |
| 0x1002 | VELOCITY_LIMIT | 速度超限 | 降低速度限制 |
| 0x1003 | TORQUE_LIMIT | 力矩超限 | 降低力矩限制 |
| 0x1004 | CONTROL_MODE_ERROR | 控制模式错误 | 切换到正确的控制模式 |

### 硬件错误码 (0x2xxx)

| 错误码 | 名称 | 描述 | 解决方案 |
|-------|------|------|---------|
| 0x2001 | MOTOR_OVERCURRENT | 电机过流 | 降低负载，检查机械卡死 |
| 0x2002 | MOTOR_OVERTEMP | 电机过温 | 等待冷却，降低使用强度 |
| 0x2003 | ENCODER_ERROR | 编码器错误 | 重启设备，联系技术支持 |
| 0x2004 | POWER_ERROR | 电源错误 | 检查电源适配器和电压 |

### 传感器错误码 (0x3xxx)

| 错误码 | 名称 | 描述 | 解决方案 |
|-------|------|------|---------|
| 0x3001 | TACTILE_SENSOR_ERROR | 触觉传感器错误 | 重新校准，检查传感器 |
| 0x3002 | SENSOR_TIMEOUT | 传感器超时 | 检查传感器连接 |
| 0x3003 | SENSOR_CALIBRATION_FAILED | 校准失败 | 确保无接触状态下校准 |

## 诊断工具

### 系统诊断

\`\`\`python
# 运行完整的系统诊断
hand.run_diagnostics()

# 输出诊断报告
report = hand.get_diagnostic_report()
print(report)
\`\`\`

### 日志记录

\`\`\`python
# 启用详细日志
hand.set_log_level(xhand.LogLevel.DEBUG)

# 保存日志到文件
hand.enable_log_file("xhand_debug.log")
\`\`\`

## 联系技术支持

如果以上方法无法解决问题，请联系技术支持：

- **邮箱**：support@xingdong.tech
- **电话**：+86-xxx-xxxx-xxxx
- **在线工单**：https://support.xingdong.tech

提交问题时，请提供：
1. 错误码和错误信息
2. 系统诊断报告
3. 日志文件
4. 问题复现步骤`
      },
      {
        id: "glossary",
        title: "词汇表",
        path: "/docs/support/glossary",
        content: `# 词汇表

## A

**API (Application Programming Interface)**
应用程序编程接口，软件组件之间交互的规范。

**Adaptive Grasping**
自适应抓取，根据物体形状自动调整手指姿态的抓取方法。

## C

**CMC (Carpometacarpal Joint)**
腕掌关节，拇指的根部关节。

**Control Mode**
控制模式，指定如何控制关节的方式（位置、速度、力矩等）。

## D

**DoF (Degree of Freedom)**
自由度，机械系统可以独立运动的方向数量。XHAND1 具有 12 个主动自由度。

**DIP (Distal Interphalangeal Joint)**
远端指间关节，手指最远端的关节。

## E

**EtherCAT**
以太网控制自动化技术，一种高速实时工业以太网协议。

## F

**Force Control**
力控制，基于力反馈的控制方法，适用于需要与环境交互的场景。

**Fully-Actuated**
全驱，每个关节都有独立驱动单元的设计方式。

## I

**Impedance Control**
阻抗控制，一种力控制策略，模拟弹簧-阻尼系统的行为。

**IP (Interphalangeal Joint)**
指间关节，手指中间的关节。

## M

**MCP (Metacarpophalangeal Joint)**
掌指关节，手指与手掌连接的关节。

## P

**PID Controller**
比例-积分-微分控制器，一种常用的反馈控制算法。

**PIP (Proximal Interphalangeal Joint)**
近端指间关节，手指靠近手掌的指间关节。

**Position Control**
位置控制，精确控制关节位置的控制方法。

**Precision Grasp**
精密抓取，使用指尖进行的精细抓取。

**Power Grasp**
力量抓取，使用整个手掌进行的强力抓取。

## Q

**Quasi-Direct Drive**
准直驱，结合直驱和减速器优点的电机驱动技术。

## R

**RS485**
一种串行通信标准，支持多点通信。

**ROS (Robot Operating System)**
机器人操作系统，一个用于机器人软件开发的框架。

## S

**SDK (Software Development Kit)**
软件开发工具包，提供开发特定软件所需的工具和库。

**Sim2Real**
从仿真到现实，将在仿真环境中训练的策略迁移到真实硬件的过程。

## T

**Tactile Sensor**
触觉传感器，用于测量接触力的传感器。

**Teleoperation**
遥操作，通过远程设备控制机器人的方法。

**Torque Control**
力矩控制，直接控制关节力矩的控制方法。

## U

**Under-Actuated**
欠驱，驱动单元数量少于自由度数量的设计方式。

**URDF (Unified Robot Description Format)**
统一机器人描述格式，用于描述机器人结构的XML格式。

## 中文术语

**灵巧手 (Dexterous Hand)**
具有多自由度和精细操作能力的机器人手。

**抓握力 (Grasp Force)**
抓取物体时施加的力。

**关节限位 (Joint Limit)**
关节可以运动的最大和最小角度。

**触觉反馈 (Tactile Feedback)**
通过触觉传感器获得的接触信息。

**柔顺控制 (Compliant Control)**
能够适应环境变化的柔性控制方法。`
      },
      {
        id: "downloads",
        title: "下载中心",
        path: "/docs/support/downloads",
        content: `# 下载中心

## SDK 下载

### Python SDK

| 版本 | 发布日期 | 下载链接 | 说明 |
|------|---------|---------|------|
| v1.2.0 | 2024-11-15 | [下载](https://github.com/your-org/xhand-sdk-python/releases/v1.2.0) | 最新稳定版 |
| v1.1.0 | 2024-10-01 | [下载](https://github.com/your-org/xhand-sdk-python/releases/v1.1.0) | 旧版本 |

**安装方式：**
\`\`\`bash
pip install xhand-control-sdk==1.2.0
\`\`\`

### C++ SDK

| 版本 | 发布日期 | 下载链接 | 说明 |
|------|---------|---------|------|
| v1.2.0 | 2024-11-15 | [下载](https://github.com/your-org/xhand-sdk-cpp/releases/v1.2.0) | 最新稳定版 |
| v1.1.0 | 2024-10-01 | [下载](https://github.com/your-org/xhand-sdk-cpp/releases/v1.1.0) | 旧版本 |

### ROS Package

| 版本 | ROS版本 | 发布日期 | 下载链接 |
|------|---------|---------|---------|
| v1.2.0 | Noetic | 2024-11-15 | [下载](https://github.com/your-org/xhand-ros/releases/v1.2.0) |
| v1.2.0 | Humble | 2024-11-15 | [下载](https://github.com/your-org/xhand-ros2/releases/v1.2.0) |

## 3D 模型

### URDF 模型

| 文件名 | 描述 | 下载链接 |
|-------|------|---------|
| xhand1.urdf | XHAND1 完整URDF模型 | [下载](/downloads/xhand1.urdf) |
| xhand1_lite.urdf | XHAND1 Lite URDF模型 | [下载](/downloads/xhand1_lite.urdf) |

### STEP 模型

| 文件名 | 描述 | 下载链接 |
|-------|------|---------|
| xhand1.step | XHAND1 STEP格式3D模型 | [下载](/downloads/xhand1.step) |
| xhand1_lite.step | XHAND1 Lite STEP格式3D模型 | [下载](/downloads/xhand1_lite.step) |

## 文档

### 产品手册

| 文档名称 | 版本 | 下载链接 |
|---------|------|---------|
| XHAND1 产品使用手册 | v2.0 | [下载](/downloads/xhand1_manual_v2.0.pdf) |
| XHAND1 技术白皮书 | v1.0 | [下载](/downloads/xhand1_whitepaper_v1.0.pdf) |

### SDK 文档

| 文档名称 | 版本 | 下载链接 |
|---------|------|---------|
| Python SDK 使用文档 | v1.2 | [下载](/downloads/python_sdk_docs_v1.2.pdf) |
| C++ SDK 使用文档 | v1.2 | [下载](/downloads/cpp_sdk_docs_v1.2.pdf) |
| ROS 集成指南 | v1.0 | [下载](/downloads/ros_integration_guide_v1.0.pdf) |

## 固件

### XHAND1 固件

| 版本 | 发布日期 | 下载链接 | 更新说明 |
|------|---------|---------|---------|
| v2.1.0 | 2024-11-15 | [下载](/downloads/xhand1_firmware_v2.1.0.bin) | 修复触觉传感器bug |
| v2.0.0 | 2024-10-01 | [下载](/downloads/xhand1_firmware_v2.0.0.bin) | 主要版本更新 |

**固件更新方法：**
\`\`\`bash
xhand-update-firmware xhand1_firmware_v2.1.0.bin
\`\`\`

## 示例代码

### Python 示例

| 示例名称 | 描述 | 下载链接 |
|---------|------|---------|
| 基本控制示例 | 位置控制、力控制等基本功能 | [下载](/downloads/examples/python_basic.zip) |
| 触觉感知示例 | 触觉传感器应用示例 | [下载](/downloads/examples/python_tactile.zip) |
| 遥操作示例 | MANUS手套集成示例 | [下载](/downloads/examples/python_teleoperation.zip) |

### C++ 示例

| 示例名称 | 描述 | 下载链接 |
|---------|------|---------|
| 实时控制示例 | 高频实时控制示例 | [下载](/downloads/examples/cpp_realtime.zip) |
| 多线程示例 | 多线程数据采集和控制 | [下载](/downloads/examples/cpp_multithread.zip) |

## 仿真环境

### Isaac Gym

| 文件名 | 描述 | 下载链接 |
|-------|------|---------|
| xhand1_isaac_gym.zip | Isaac Gym 仿真环境 | [下载](/downloads/sim/xhand1_isaac_gym.zip) |

### MuJoCo

| 文件名 | 描述 | 下载链接 |
|-------|------|---------|
| xhand1_mujoco.zip | MuJoCo 仿真环境 | [下载](/downloads/sim/xhand1_mujoco.zip) |

## 工具软件

| 软件名称 | 版本 | 平台 | 下载链接 |
|---------|------|------|---------|
| XHAND1 上位机 | v1.5.0 | Windows | [下载](/downloads/tools/xhand1_gui_v1.5.0_win.exe) |
| XHAND1 上位机 | v1.5.0 | Linux | [下载](/downloads/tools/xhand1_gui_v1.5.0_linux.AppImage) |
| 固件更新工具 | v1.0.0 | 跨平台 | [下载](/downloads/tools/firmware_updater_v1.0.0.zip) |

## 版本历史

### v1.2.0 (2024-11-15)

**新功能：**
- 新增自适应抓取功能
- 支持混合控制模式
- 优化触觉传感器滤波算法

**Bug修复：**
- 修复EtherCAT通信偶尔丢包的问题
- 修复力控模式下的稳定性问题

**已知问题：**
- 在某些Linux发行版上，首次连接可能需要重启

---

## 技术支持

如有任何问题，请联系：
- **邮箱**：support@xingdong.tech
- **电话**：+86-xxx-xxxx-xxxx`
      }
    ]
  }
];

export function findDocByPath(path: string): DocSection | undefined {
  function search(sections: DocSection[]): DocSection | undefined {
    for (const section of sections) {
      if (section.path === path) {
        return section;
      }
      if (section.children) {
        const found = search(section.children);
        if (found) return found;
      }
    }
    return undefined;
  }
  return search(docsData);
}

export function getAllPaths(): string[] {
  const paths: string[] = [];
  function collect(sections: DocSection[]) {
    for (const section of sections) {
      paths.push(section.path);
      if (section.children) {
        collect(section.children);
      }
    }
  }
  collect(docsData);
  return paths;
}
