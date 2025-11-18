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

**XHAND1** 是星动纪元自研的全直驱仿人五指灵巧手，代表了当前机器人灵巧操作领域的前沿技术水平。

## 产品定位

XHAND1 重新定义了仿人五指灵巧手的行业标准，具备12个主动自由度，其中拇指和食指各有3个自由度，其余三指各有2个自由度。特别支持食指的侧摆动作，能够模拟和复现人手的复杂动作。

## 核心亮点

### 全直驱设计

XHAND1 采用创新的全直驱（Fully-actuated）设计理念，每个手指关节通过齿轮和电机驱动，实现每个关节的独立运动控制。相比传统的欠驱（Under-actuated）方案，XHAND1 能够完成更加复杂和精细的操作任务。

### 高分辨率触觉传感

每个手指可配备一个270度环绕指尖的高分辨率触觉传感器（超过100点），提供精确的三维力触觉和温度信息。在传统的压力触觉之外可以感知切向力，高空间分辨率能精确感知物体边缘、形状和硬度。

### 强大的抓握力

单手最大抓握力可达80N，最大负载（掌心向上）可达25kg，使其能够胜任从精细化操作到重物搬运的多样化任务。

### 高智能性

搭配机械臂与机器人通过强化学习、模仿学习全链路验证，支持多种控制模式和算法开发。

## 软件支持

1. 开发工具包(SDK)：提供标准的Linux系统SDK和ROS兼容应用，便于算法开发
2. 远程遥操作：提供VisionPro远程操作支持，兼容手套和视觉映射，便于客户操作测试和训练数据收集
3. 上位机评估应用：配备标准的上位机评估应用程序，支持客户调试和嵌入式系统研发

## 应用场景

XHAND1 广泛应用于人形机器人、工业自动化、科研教学、医疗康复、服务机器人等领域，是实现机器人灵巧操作的理想选择。`
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
      id: "host-software",
      title: "上位机软件使用",
      path: "/docs/quickstart/host-software",
      content: `# 上位机软件使用

## 软件简介

XHAND1 上位机软件是一款专为灵巧手设计的图形化控制工具，提供直观的界面和丰富的功能，让您无需编程即可快速测试和控制灵巧手。

## 系统要求

### 硬件要求
- **处理器**：Intel Core i5 或更高
- **内存**：4GB RAM 或更高
- **网络接口**：支持EtherCAT的网卡或RS485转USB适配器

### 软件要求
- **Windows**：Windows 10/11 (64位)
- **Linux**：Ubuntu 20.04/22.04 LTS
- **macOS**：macOS 12+ (仅支持RS485连接)

## 软件安装

### Windows 安装

1. 从[下载中心](/docs/support/downloads)下载最新版本的上位机软件安装包
2. 双击 \`XHAND1_HostSoftware_vX.X.X_Win64.exe\` 运行安装程序
3. 按照安装向导完成安装
4. 首次运行时，可能需要安装驱动程序，请允许安装

### Linux 安装

\`\`\`bash
# 下载安装包
wget https://downloads.xingdong.tech/xhand1_host_vX.X.X_linux.AppImage

# 添加执行权限
chmod +x xhand1_host_vX.X.X_linux.AppImage

# 运行软件
./xhand1_host_vX.X.X_linux.AppImage
\`\`\`

## 软件界面

### 主界面布局

上位机软件采用直观的分区布局：

- **左侧控制面板**：连接设置、控制模式选择、参数配置
- **中央3D视图**：实时显示灵巧手的姿态和运动状态
- **右侧状态监控**：关节角度、力矩、触觉传感器数据
- **底部日志窗口**：显示操作日志和错误信息

## 连接灵巧手

### 1. 硬件连接

按照[开箱与安全须知](/docs/quickstart/unboxing)完成硬件连接。

### 2. 配置连接参数

在左侧控制面板中：

1. **选择通信方式**：EtherCAT 或 RS485
2. **EtherCAT连接**：
   - 选择网卡接口（如 \`eth0\`）
   - 设置从站地址（默认为 0）
3. **RS485连接**：
   - 选择串口（如 \`COM3\` 或 \`/dev/ttyUSB0\`）
   - 波特率：115200
   - 设备地址：默认为 1

### 3. 建立连接

点击"连接"按钮，软件会自动搜索并连接灵巧手。连接成功后：

- 状态指示灯变为绿色
- 3D视图中显示灵巧手模型
- 右侧面板开始显示实时数据

## 基本操作

### 控制模式

上位机支持多种控制模式：

#### 1. 手动控制模式

- 使用滑块调节每个关节的目标位置
- 实时查看当前位置和目标位置的差异
- 支持单关节控制和整手协同控制

#### 2. 预设姿态模式

软件内置多种预设姿态：

- **张开手**：所有关节回到0度
- **握拳**：所有手指弯曲
- **捏取**：拇指和食指对捏
- **OK手势**：拇指和食指形成圆圈
- **点赞**：拇指竖起

选择姿态后，点击"执行"即可让灵巧手做出相应动作。

#### 3. 力控模式

- 设置目标抓握力（单位：N）
- 灵巧手会自动调整手指位置，保持恒定的抓握力
- 适用于抓取不同硬度的物体

### 触觉传感器监控

右侧状态面板实时显示16个触觉传感器的数据：

- **数值显示**：显示每个传感器的接触力（单位：N）
- **热力图**：用颜色深浅表示接触力大小
- **历史曲线**：查看传感器数据的时间变化

### 数据记录

上位机支持记录运动和传感器数据：

1. 点击"开始记录"按钮
2. 执行需要记录的操作
3. 点击"停止记录"
4. 数据保存为CSV格式，可用于后续分析

## 高级功能

### 轨迹回放

1. 录制一段运动轨迹
2. 保存为轨迹文件（.traj格式）
3. 随时加载并回放轨迹

### 参数调节

在"高级设置"中可以调节：

- PID控制参数
- 速度和加速度限制
- 力控制参数
- 触觉传感器滤波参数

### 固件更新

上位机集成了固件更新功能：

1. 点击"工具" → "固件更新"
2. 选择固件文件（.bin格式）
3. 点击"开始更新"
4. 等待更新完成（约2-3分钟）

## 常见问题

### Q: 连接失败怎么办？

A: 请检查：
1. 硬件连接是否正确
2. 网卡或串口是否被其他程序占用
3. 防火墙是否阻止了软件
4. 驱动程序是否正确安装

### Q: 3D视图不显示怎么办？

A: 可能是显卡驱动问题，请：
1. 更新显卡驱动到最新版本
2. 在设置中切换到"简化渲染模式"

### Q: 如何导出数据？

A: 点击"文件" → "导出数据"，选择导出格式（CSV、JSON、MATLAB）。`
    },
    {
      id: "first-test",
      title: "第一次测试运行",
      path: "/docs/quickstart/first-test",
      content: `# 第一次测试运行

## 准备工作

在进行第一次测试之前，请确保：

- ✅ 已完成硬件连接
- ✅ 已安装上位机软件
- ✅ 灵巧手处于安全的测试环境中
- ✅ 周围没有易碎或危险物品

## 测试步骤

### 1. 启动上位机软件

双击桌面图标或从开始菜单启动"XHAND1 上位机"。

### 2. 连接灵巧手

1. 在左侧控制面板选择通信方式（推荐使用EtherCAT）
2. 选择正确的网卡接口
3. 点击"连接"按钮
4. 等待连接成功提示

### 3. 执行自检

连接成功后，建议先执行系统自检：

1. 点击"工具" → "系统自检"
2. 软件会自动检测：
   - 所有关节是否正常
   - 触觉传感器是否工作
   - 通信是否稳定
3. 查看自检报告，确认无异常

### 4. 测试基本动作

#### 测试1：张开和闭合

1. 在预设姿态中选择"张开手"
2. 点击"执行"，观察灵巧手是否完全张开
3. 选择"握拳"姿态
4. 点击"执行"，观察灵巧手是否顺利握拳
5. 重复几次，确认动作流畅

#### 测试2：单指控制

1. 切换到"手动控制模式"
2. 选择"食指MCP"关节
3. 拖动滑块，将目标位置设置为45度
4. 观察食指是否精确移动到目标位置
5. 依次测试其他手指

#### 测试3：力控抓取

1. 准备一个柔软的物体（如海绵球）
2. 将物体放置在灵巧手掌心
3. 切换到"力控模式"
4. 设置目标抓握力为5N
5. 点击"执行"
6. 观察灵巧手是否轻柔地抓住物体
7. 查看触觉传感器数据，确认有接触力反馈

### 5. 查看传感器数据

在右侧状态面板中：

1. 观察关节角度是否与3D视图一致
2. 查看触觉传感器热力图
3. 用手指轻触灵巧手的不同部位，观察传感器响应

### 6. 记录测试数据

1. 点击"开始记录"
2. 执行一次完整的"张开-握拳-张开"动作
3. 点击"停止记录"
4. 保存数据文件，用于后续分析

## 测试检查清单

完成以下检查项，确保灵巧手工作正常：

- [ ] 所有关节能够正常运动
- [ ] 关节运动平滑，无卡顿或异响
- [ ] 触觉传感器能够正确响应接触
- [ ] 力控模式能够稳定抓取物体
- [ ] 通信稳定，无频繁断连
- [ ] 3D视图与实际姿态一致
- [ ] 无异常发热或异味

## 常见问题

### Q: 手指运动不到位怎么办？

A: 可能原因：
1. 机械限位设置不正确 → 在高级设置中校准限位
2. 负载过大 → 减轻负载或降低速度
3. 电源电压不足 → 检查电源适配器

### Q: 触觉传感器读数异常？

A: 请执行传感器校准：
1. 确保手指无接触
2. 点击"工具" → "传感器校准"
3. 等待校准完成

### Q: 如何保存当前姿态？

A: 
1. 调整到目标姿态
2. 点击"姿态" → "保存当前姿态"
3. 输入姿态名称
4. 下次可以从"自定义姿态"中加载

## 下一步

恭喜您完成了第一次测试！接下来您可以：

- 学习[SDK开发指南](/docs/sdk/overview)，编写自己的控制程序
- 探索[遥操作功能](/docs/teleoperation/overview)，使用VR设备控制灵巧手
- 查看[应用实践](/docs/applications/grasping)，了解更多高级应用

## 安全提示

⚠️ 在测试过程中，如果发现任何异常情况（如异响、过热、动作异常），请立即：

1. 点击"急停"按钮
2. 断开电源
3. 联系技术支持

**技术支持联系方式**：
- 邮箱：support@xingdong.tech
- 电话：+86-xxx-xxxx-xxxx`
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

## 概述

灵巧手 SDK（Software Development Kit）是一套软件开发工具包，旨在帮助开发者和系统集成商快速集成和控制灵巧手机器人。该 SDK 通过简化复杂的底层通讯协议和控制逻辑，使开发者能够更专注于应用开发和功能实现，以便他们能够为灵巧手机器人设计和实现各种控制算法和应用程序。

灵巧手是一种高度灵活的机器人手，能够模仿人手的动作和功能，广泛应用于科研、工业自动化、医疗康复、服务机器人等领域。

## SDK 组成部分

灵巧手 SDK 通常包括以下几个方面的内容：

### 1. API 接口

提供一组函数和方法，允许开发者控制灵巧手的运动等。支持多种编程语言：

- **Python SDK**：适合快速原型开发和算法验证
- **C++ SDK**：适合高性能实时控制应用
- **ROS1/ROS2**：适合机器人系统集成

### 2. 示例代码和文档

提供示例代码和详细的开发文档，帮助开发者快速上手和理解如何使用 SDK。

### 3. 调试和模拟工具

可能包括用于测试和模拟环境，以及用于监控灵巧手状态和控制的工具。

**上位机下载地址**：[https://www.robotera.com/](https://www.robotera.com/)

## 支持的平台

- **操作系统**：Linux (Ubuntu 20.04/22.04)
- **通信协议**：RS485、EtherCAT
- **实时性**：83Hz 控制频率

## 快速开始

请选择您喜欢的开发语言和平台：

- [Python SDK](/docs/sdk/python) - 适合快速原型开发
- [C++ SDK](/docs/sdk/cpp) - 适合高性能应用
- [ROS1 集成](/docs/sdk/ros1) - 适合ROS Noetic系统
- [ROS2 集成](/docs/sdk/ros2) - 适合ROS2 Humble系统`
    },
    {
      id: "python",
      title: "Python SDK",
      path: "/docs/sdk/python",
      content: `# Python SDK

XHAND1 Python SDK 提供了简洁易用的 Python 接口，适合快速原型开发和算法验证。通过 Python SDK，您可以轻松实现位置控制、力控制、触觉数据读取等功能。

## 主要特性

- 简洁的 API 设计，易于上手
- 支持异步和同步控制模式
- 完整的触觉传感器数据访问
- 丰富的示例代码和文档

## 安装方法

请参考 SDK 架构概述中的安装指南，或访问下载中心获取最新版本。`
    },
    {
      id: "cpp",
      title: "C++ SDK",
      path: "/docs/sdk/cpp",
      content: `# C++ SDK

XHAND1 C++ SDK 提供了高性能的 C++ 接口，适合实时控制和高频响应应用。C++ SDK 采用现代 C++17 标准，提供了低延迟、高吞吐的控制能力。

## 主要特性

- 高性能实时控制
- 低延迟通信协议
- 线程安全设计
- 完整的类型安全

## 编译和集成

请参考 SDK 架构概述中的编译指南，或访问下载中心获取最新版本。`
    },
    {
      id: "ros",
      title: "ROS1 集成",
      path: "/docs/sdk/ros",
      content: `# ROS1 集成

## 简介

XHAND1 提供了完整的ROS（Robot Operating System）功能包，支持ROS Noetic版本，让您能够轻松地将灵巧手集成到ROS机器人系统中。

## 系统要求

- **操作系统**：Ubuntu 20.04 LTS
- **ROS版本**：ROS Noetic Desktop Full
- **依赖库**：
  - \`ros-noetic-control-msgs\`
  - \`ros-noetic-sensor-msgs\`
  - \`ros-noetic-std-srvs\`

## 安装

### 1. 安装ROS Noetic

如果您还没有安装ROS Noetic，请按照以下步骤安装：

\`\`\`bash
# 设置软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 添加密钥
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 更新软件源
sudo apt-get update

# 安装ROS Noetic完整版
sudo apt-get install ros-noetic-desktop-full

# 初始化rosdep
sudo rosdep init
rosdep update

# 配置环境变量
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
\`\`\`

### 2. 创建工作空间

\`\`\`bash
# 创建catkin工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace

# 编译工作空间
cd ~/catkin_ws
catkin_make

# 配置环境变量
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
\`\`\`

### 3. 安装XHAND1 ROS包

\`\`\`bash
# 进入工作空间的src目录
cd ~/catkin_ws/src

# 克隆XHAND1 ROS包
git clone https://github.com/xingdong-robotics/xhand_control_ros.git

# 安装依赖
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
catkin_make

# 刷新环境变量
source devel/setup.bash
\`\`\`

## ROS包结构

XHAND1 ROS包包含以下功能包：

- **xhand_description**：URDF模型和可视化配置
- **xhand_control**：核心控制节点
- **xhand_msgs**：自定义消息和服务定义
- **xhand_bringup**：启动文件和配置
- **xhand_examples**：示例代码

## 启动灵巧手

### 基本启动

\`\`\`bash
# 启动XHAND1控制节点
roslaunch xhand_bringup xhand_control.launch

# 可选参数：
# - hand_id: 灵巧手ID（默认为0）
# - interface: 网络接口（默认为eth0）
# - control_mode: 控制模式（position/force/hybrid）

# 示例：指定网卡和控制模式
roslaunch xhand_bringup xhand_control.launch interface:=enp3s0 control_mode:=position
\`\`\`

### 启动可视化

\`\`\`bash
# 在RViz中可视化灵巧手
roslaunch xhand_description display.launch
\`\`\`

## Topics（话题）

### 发布的Topics

| Topic | 消息类型 | 频率 | 描述 |
|-------|---------|------|------|
| \`/xhand/joint_states\` | \`sensor_msgs/JointState\` | 100Hz | 关节状态（位置、速度、力矩） |
| \`/xhand/tactile_data\` | \`xhand_msgs/TactileArray\` | 100Hz | 触觉传感器数据 |
| \`/xhand/hand_state\` | \`xhand_msgs/HandState\` | 10Hz | 灵巧手整体状态 |

### 订阅的Topics

| Topic | 消息类型 | 描述 |
|-------|---------|------|
| \`/xhand/joint_command\` | \`xhand_msgs/JointCommand\` | 关节控制命令 |
| \`/xhand/grasp_command\` | \`xhand_msgs/GraspCommand\` | 抓取命令 |

## Services（服务）

| Service | 服务类型 | 描述 |
|---------|---------|------|
| \`/xhand/set_control_mode\` | \`xhand_msgs/SetControlMode\` | 设置控制模式 |
| \`/xhand/execute_preset\` | \`xhand_msgs/ExecutePreset\` | 执行预设姿态 |
| \`/xhand/calibrate_sensors\` | \`std_srvs/Trigger\` | 校准触觉传感器 |
| \`/xhand/emergency_stop\` | \`std_srvs/Trigger\` | 紧急停止 |

## Actions（动作）

| Action | 动作类型 | 描述 |
|--------|---------|------|
| \`/xhand/grasp_action\` | \`xhand_msgs/GraspAction\` | 执行抓取动作 |
| \`/xhand/trajectory_action\` | \`control_msgs/FollowJointTrajectoryAction\` | 执行轨迹跟踪 |

## 代码示例

### 示例1：读取关节状态

\`\`\`python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def joint_state_callback(msg):
    print("关节名称:", msg.name)
    print("关节位置:", msg.position)
    print("关节速度:", msg.velocity)
    print("关节力矩:", msg.effort)
    print("-" * 50)

if __name__ == '__main__':
    rospy.init_node('joint_state_reader')
    rospy.Subscriber('/xhand/joint_states', JointState, joint_state_callback)
    rospy.spin()
\`\`\`

### 示例2：控制单个关节

\`\`\`python
#!/usr/bin/env python3
import rospy
from xhand_msgs.msg import JointCommand

if __name__ == '__main__':
    rospy.init_node('joint_controller')
    pub = rospy.Publisher('/xhand/joint_command', JointCommand, queue_size=10)
    rospy.sleep(1)  # 等待连接建立
    
    # 创建关节命令
    cmd = JointCommand()
    cmd.joint_names = ['index_mcp']
    cmd.positions = [1.57]  # 90度（弧度制）
    cmd.velocities = [0.5]  # 可选：指定速度
    
    # 发布命令
    pub.publish(cmd)
    rospy.loginfo("已发送关节命令")
\`\`\`

### 示例3：执行预设姿态

\`\`\`python
#!/usr/bin/env python3
import rospy
from xhand_msgs.srv import ExecutePreset, ExecutePresetRequest

if __name__ == '__main__':
    rospy.init_node('preset_executor')
    
    # 等待服务可用
    rospy.wait_for_service('/xhand/execute_preset')
    
    try:
        # 创建服务代理
        execute_preset = rospy.ServiceProxy('/xhand/execute_preset', ExecutePreset)
        
        # 调用服务
        req = ExecutePresetRequest()
        req.preset_name = "close"  # 握拳姿态
        resp = execute_preset(req)
        
        if resp.success:
            rospy.loginfo("成功执行预设姿态")
        else:
            rospy.logwarn("执行失败: " + resp.message)
            
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: " + str(e))
\`\`\`

### 示例4：读取触觉数据

\`\`\`python
#!/usr/bin/env python3
import rospy
from xhand_msgs.msg import TactileArray

def tactile_callback(msg):
    print("传感器数量:", len(msg.sensors))
    for i, sensor in enumerate(msg.sensors):
        print(f"传感器{i}: {sensor.force:.2f} N")
    print("-" * 50)

if __name__ == '__main__':
    rospy.init_node('tactile_reader')
    rospy.Subscriber('/xhand/tactile_data', TactileArray, tactile_callback)
    rospy.spin()
\`\`\`

### 示例5：使用Action执行抓取

\`\`\`python
#!/usr/bin/env python3
import rospy
import actionlib
from xhand_msgs.msg import GraspAction, GraspGoal

if __name__ == '__main__':
    rospy.init_node('grasp_action_client')
    
    # 创建Action客户端
    client = actionlib.SimpleActionClient('/xhand/grasp_action', GraspAction)
    client.wait_for_server()
    
    # 创建目标
    goal = GraspGoal()
    goal.grasp_type = "power"  # 力量抓取
    goal.target_force = 10.0   # 目标抓握力（N）
    goal.timeout = 5.0         # 超时时间（秒）
    
    # 发送目标
    client.send_goal(goal)
    
    # 等待结果
    client.wait_for_result()
    result = client.get_result()
    
    if result.success:
        rospy.loginfo("抓取成功！")
    else:
        rospy.logwarn("抓取失败: " + result.message)
\`\`\`

## 与MoveIt集成

XHAND1 ROS包支持与MoveIt运动规划框架集成：

\`\`\`bash
# 安装MoveIt
sudo apt-get install ros-noetic-moveit

# 启动MoveIt配置
roslaunch xhand_moveit_config demo.launch
\`\`\`

## 常见问题

### Q: 如何查看所有可用的Topics？

\`\`\`bash
rostopic list | grep xhand
\`\`\`

### Q: 如何录制和回放数据？

\`\`\`bash
# 录制
rosbag record /xhand/joint_states /xhand/tactile_data -O xhand_data.bag

# 回放
rosbag play xhand_data.bag
\`\`\`

### Q: 如何调试通信问题？

\`\`\`bash
# 查看节点信息
rosnode info /xhand_control_node

# 查看Topic信息
rostopic info /xhand/joint_states

# 实时查看消息
rostopic echo /xhand/joint_states
\`\`\``
    },
    {
      id: "ros2",
      title: "ROS2 集成",
      path: "/docs/sdk/ros2",
      content: `# ROS2 集成

## 简介

XHAND1 提供了完整的ROS2功能包，支持ROS2 Humble版本，采用最新的DDS通信架构，提供更好的实时性和可靠性。

## 系统要求

- **操作系统**：Ubuntu 22.04 LTS
- **ROS2版本**：ROS2 Humble Hawksbill
- **依赖库**：
  - \`ros-humble-control-msgs\`
  - \`ros-humble-sensor-msgs\`
  - \`ros-humble-std-srvs\`

## 安装

### 1. 安装ROS2 Humble

\`\`\`bash
# 设置locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS2软件源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2
sudo apt update
sudo apt install ros-humble-desktop

# 配置环境变量
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
\`\`\`

### 2. 创建工作空间

\`\`\`bash
# 创建colcon工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 克隆XHAND1 ROS2包
git clone https://github.com/xingdong-robotics/xhand_control_ros2.git

# 安装依赖
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
cd ~/ros2_ws
colcon build

# 配置环境变量
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
\`\`\`

## 启动灵巧手

### 基本启动

\`\`\`bash
# 启动XHAND1控制节点
ros2 launch xhand_bringup xhand_control.launch.py

# 可选参数：
ros2 launch xhand_bringup xhand_control.launch.py \\
    hand_id:=0 \\
    interface:=eth0 \\
    control_mode:=position
\`\`\`

### 启动可视化

\`\`\`bash
# 在RViz2中可视化
ros2 launch xhand_description display.launch.py
\`\`\`

## Topics（话题）

### 发布的Topics

| Topic | 消息类型 | QoS | 描述 |
|-------|---------|-----|------|
| \`/xhand/joint_states\` | \`sensor_msgs/msg/JointState\` | Reliable | 关节状态 |
| \`/xhand/tactile_data\` | \`xhand_msgs/msg/TactileArray\` | Reliable | 触觉数据 |
| \`/xhand/hand_state\` | \`xhand_msgs/msg/HandState\` | Reliable | 手部状态 |

### 订阅的Topics

| Topic | 消息类型 | 描述 |
|-------|---------|------|
| \`/xhand/joint_command\` | \`xhand_msgs/msg/JointCommand\` | 关节命令 |
| \`/xhand/grasp_command\` | \`xhand_msgs/msg/GraspCommand\` | 抓取命令 |

## Services（服务）

| Service | 服务类型 | 描述 |
|---------|---------|------|
| \`/xhand/set_control_mode\` | \`xhand_msgs/srv/SetControlMode\` | 设置控制模式 |
| \`/xhand/execute_preset\` | \`xhand_msgs/srv/ExecutePreset\` | 执行预设姿态 |
| \`/xhand/calibrate_sensors\` | \`std_srvs/srv/Trigger\` | 校准传感器 |

## Actions（动作）

| Action | 动作类型 | 描述 |
|--------|---------|------|
| \`/xhand/grasp\` | \`xhand_msgs/action/Grasp\` | 抓取动作 |
| \`/xhand/follow_joint_trajectory\` | \`control_msgs/action/FollowJointTrajectory\` | 轨迹跟踪 |

## 代码示例

### 示例1：读取关节状态（Python）

\`\`\`python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateReader(Node):
    def __init__(self):
        super().__init__('joint_state_reader')
        self.subscription = self.create_subscription(
            JointState,
            '/xhand/joint_states',
            self.joint_state_callback,
            10
        )
    
    def joint_state_callback(self, msg):
        self.get_logger().info(f'关节位置: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    node = JointStateReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

### 示例2：控制关节（Python）

\`\`\`python
import rclpy
from rclpy.node import Node
from xhand_msgs.msg import JointCommand

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher = self.create_publisher(
            JointCommand,
            '/xhand/joint_command',
            10
        )
        self.timer = self.create_timer(1.0, self.send_command)
    
    def send_command(self):
        cmd = JointCommand()
        cmd.joint_names = ['index_mcp']
        cmd.positions = [1.57]  # 90度
        self.publisher.publish(cmd)
        self.get_logger().info('已发送关节命令')

def main(args=None):
    rclpy.init(args=args)
    node = JointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

### 示例3：调用服务（Python）

\`\`\`python
import rclpy
from rclpy.node import Node
from xhand_msgs.srv import ExecutePreset

class PresetExecutor(Node):
    def __init__(self):
        super().__init__('preset_executor')
        self.client = self.create_client(ExecutePreset, '/xhand/execute_preset')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务...')
    
    def execute_preset(self, preset_name):
        request = ExecutePreset.Request()
        request.preset_name = preset_name
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    node = PresetExecutor()
    
    future = node.execute_preset('close')
    rclpy.spin_until_future_complete(node, future)
    
    if future.result().success:
        node.get_logger().info('成功执行预设姿态')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

### 示例4：使用Action（Python）

\`\`\`python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from xhand_msgs.action import Grasp

class GraspActionClient(Node):
    def __init__(self):
        super().__init__('grasp_action_client')
        self._action_client = ActionClient(self, Grasp, '/xhand/grasp')
    
    def send_goal(self, grasp_type, target_force):
        goal_msg = Grasp.Goal()
        goal_msg.grasp_type = grasp_type
        goal_msg.target_force = target_force
        
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'当前力: {feedback.current_force} N')
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝')
            return
        
        self.get_logger().info('目标已接受')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('抓取成功！')
        else:
            self.get_logger().info(f'抓取失败: {result.message}')

def main(args=None):
    rclpy.init(args=args)
    node = GraspActionClient()
    node.send_goal('power', 10.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
\`\`\`

## 与MoveIt2集成

\`\`\`bash
# 安装MoveIt2
sudo apt install ros-humble-moveit

# 启动MoveIt2配置
ros2 launch xhand_moveit_config demo.launch.py
\`\`\`

## 常见命令

\`\`\`bash
# 查看所有Topics
ros2 topic list

# 查看Topic信息
ros2 topic info /xhand/joint_states

# 实时查看消息
ros2 topic echo /xhand/joint_states

# 查看节点信息
ros2 node info /xhand_control_node

# 查看服务列表
ros2 service list

# 录制数据
ros2 bag record /xhand/joint_states /xhand/tactile_data

# 回放数据
ros2 bag play xhand_data
\`\`\`

## ROS1与ROS2对比

| 特性 | ROS1 | ROS2 |
|------|------|------|
| 通信架构 | TCPROS/UDPROS | DDS |
| 实时性 | 一般 | 优秀 |
| 安全性 | 基础 | 增强 |
| 多机器人支持 | 需要配置 | 原生支持 |
| 工具链 | catkin | colcon |

## 性能优化

### 1. QoS配置

\`\`\`python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 配置高性能QoS
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

self.subscription = self.create_subscription(
    JointState,
    '/xhand/joint_states',
    self.callback,
    qos_profile
)
\`\`\`

### 2. 使用Executor优化

\`\`\`python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
\`\`\``
    }
  ]
},
  {
  id: "teleoperation",
  title: "遥操作",
  path: "/docs/teleoperation",
  children: [
    {
      id: "overview",
      title: "遥操作概述",
      path: "/docs/teleoperation/overview",
      content: `# 遥操作概述

## 什么是遥操作

支持VR头显、动捕手套、外骨骼手套等多种遥操作方案。提供自研高精度遥操作软件，实现人手与 XHAND 系列灵巧手的 retargeting（动作重定向）功能，使客户能够快速完成远程操作测试和高质量数据采集。

## 核心技术：Retargeting（动作重定向）

**Retargeting（动作重定向）**是遥操作的核心技术，它解决了人手与机器人手之间的差异问题：

- **运动学差异**：人手与XHAND1的关节数量、活动范围不完全相同
- **尺寸差异**：人手与灵巧手的尺寸比例不同
- **坐标系差异**：需要将人手坐标系转换到机器人坐标系

我们的自研Retargeting算法能够：

✅ **实时映射**：低延迟（<10ms）的动作映射
✅ **自然控制**：保持人手操作的直观性
✅ **精确重现**：高保真度的动作复现
✅ **自适应优化**：根据任务自动调整映射策略

## 支持的遥操作方案

XHAND1 支持多种遥操作设备和方案：

### 1. VR头显方案

- **Meta Quest 2/3/Pro**：无线VR头显 + 手部追踪
- **Apple Vision Pro**：高精度空间计算 + 手势识别

**优势**：
- 无需额外设备，使用头显内置的手部追踪
- 沉浸式体验，适合远程操作
- 成本相对较低

**适用场景**：
- 远程操作演示
- 虚拟现实应用
- 教育培训

### 2. 数据手套方案

- **MANUS Meta/Quantum手套**：高精度指尖追踪
- **外骨骼手套**：带力反馈的专业手套

**优势**：
- 极高的精度和灵敏度
- 支持力反馈（部分型号）
- 适合专业应用

**适用场景**：
- 高质量数据采集
- 精密操作任务
- 科研实验

### 3. 动作捕捉方案

- **光学动捕系统**：多相机追踪标记点
- **惯性动捕系统**：IMU传感器追踪

**优势**：
- 大范围追踪
- 多人协作支持
- 高精度

**适用场景**：
- 复杂场景操作
- 多机器人协同
- 动作数据库构建

## 自研遥操作软件

星动纪元提供自研的高精度遥操作软件，具有以下特点：

### 核心功能

1. **多设备支持**
   - 统一的软件架构，支持所有主流遥操作设备
   - 即插即用，自动识别设备类型
   - 设备间无缝切换

2. **智能Retargeting**
   - 自适应算法，自动优化映射参数
   - 支持自定义映射规则
   - 实时可视化调试

3. **数据采集与回放**
   - 高频数据记录（1000Hz）
   - 多模态数据同步（视觉、触觉、运动）
   - 轨迹回放和编辑

4. **远程操作支持**
   - 低延迟网络传输
   - 断线自动重连
   - 操作录像和回放

### 软件架构

\`\`\`
┌─────────────────────────────────────────┐
│         遥操作设备                        │
│  (Quest/AVP/MANUS/动捕系统)              │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│       设备驱动层 (Device Driver)         │
│  统一的设备接口，支持多种设备              │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│    Retargeting引擎 (Retargeting Engine)  │
│  动作重定向、运动学求解、碰撞检测          │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│      控制层 (Control Layer)              │
│  位置控制、力控制、混合控制                │
└─────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────┐
│         XHAND1 灵巧手                    │
└─────────────────────────────────────────┘
\`\`\`

## 应用场景

### 1. 远程操作

通过遥操作，操作员可以在远程控制XHAND1完成复杂任务：

- **危险环境作业**：核辐射、化学污染区域
- **医疗手术**：远程手术辅助
- **空间探索**：太空站维护

### 2. 数据采集

遥操作是采集高质量机器人操作数据的最佳方式：

- **模仿学习数据集**：用于训练AI模型
- **动作库构建**：建立标准操作动作库
- **性能评估**：对比人手与机器手的性能

### 3. 人机协作

遥操作使人类专家的技能能够通过机器人得以延伸：

- **精密装配**：利用人类的灵巧性和机器人的稳定性
- **艺术创作**：远程雕刻、绘画
- **技能传承**：记录和复现专家的操作技巧

## 性能指标

我们的遥操作系统达到了业界领先的性能：

| 指标 | 数值 |
|------|------|
| **端到端延迟** | < 10 ms |
| **位置精度** | < 2 mm |
| **姿态精度** | < 5° |
| **数据采集频率** | 1000 Hz |
| **支持的网络** | 局域网/互联网 |

## 快速开始

准备好开始使用遥操作了吗？请根据您的设备选择相应的指南：

- [Meta Quest 遥操作](/docs/teleoperation/meta-quest)
- [Apple Vision Pro 遥操作](/docs/teleoperation/apple-vision-pro)
- [MANUS手套 遥操作](/docs/teleoperation/manus-glove)

## 技术支持

如有任何问题，请联系我们的技术支持团队：

- **邮箱**：teleoperation@xingdong.tech
- **电话**：+86-xxx-xxxx-xxxx
- **在线文档**：https://docs.xingdong.tech/teleoperation`
    },
    {
      id: "meta-quest",
      title: "Meta Quest 遥操作",
      path: "/docs/teleoperation/meta-quest",
      content: `# Meta Quest 遥操作

## 简介

Meta Quest系列（Quest 2、Quest 3、Quest Pro）是Meta公司推出的一体式VR头显，内置强大的手部追踪功能，无需额外控制器即可实现自然的手势交互。

通过我们的遥操作软件，您可以使用Quest头显直接控制XHAND1灵巧手，实现沉浸式的远程操作体验。

## 支持的设备

| 设备型号 | 手部追踪精度 | 推荐程度 |
|---------|-------------|---------|
| **Meta Quest Pro** | 高（前后摄像头） | ⭐⭐⭐⭐⭐ 最推荐 |
| **Meta Quest 3** | 高（彩色透视） | ⭐⭐⭐⭐⭐ 推荐 |
| **Meta Quest 2** | 中（黑白摄像头） | ⭐⭐⭐⭐ 可用 |

## 系统要求

### Quest头显要求
- 系统版本：v50 或更高
- 已启用开发者模式
- 已启用手部追踪功能

### PC端要求
- **操作系统**：Windows 10/11 或 Ubuntu 20.04/22.04
- **处理器**：Intel Core i5 或更高
- **内存**：8GB RAM 或更高
- **网络**：与Quest头显在同一局域网

### XHAND1要求
- 已完成硬件连接和基本配置
- 与PC在同一网络或直连

## 安装步骤

### 1. 准备Quest头显

#### 启用开发者模式

1. 在手机上安装Meta Quest应用
2. 登录您的Meta账号
3. 进入"设置" → "开发者模式"
4. 开启开发者模式

#### 启用手部追踪

1. 戴上Quest头显
2. 进入"设置" → "动作" → "手和控制器"
3. 开启"手部追踪"
4. 按照提示完成手部追踪校准

### 2. 安装Quest应用

#### 方式A：通过SideQuest安装（推荐）

1. 在PC上安装[SideQuest](https://sidequestvr.com/)
2. 用USB-C线连接Quest到PC
3. 在Quest上允许USB调试
4. 下载XHAND1遥操作APK：[xhand_teleoperation_quest.apk](/downloads/xhand_teleoperation_quest.apk)
5. 在SideQuest中点击"Install APK"，选择下载的APK文件
6. 等待安装完成

#### 方式B：通过ADB安装

\`\`\`bash
# 安装ADB工具
# Windows: 下载Android Platform Tools
# Linux: sudo apt-get install android-tools-adb

# 连接Quest
adb devices

# 安装APK
adb install xhand_teleoperation_quest.apk
\`\`\`

### 3. 安装PC端软件

#### Windows

1. 下载安装包：[XHAND_Teleoperation_Windows.exe](/downloads/XHAND_Teleoperation_Windows.exe)
2. 双击运行安装程序
3. 按照向导完成安装

#### Linux

\`\`\`bash
# 下载安装包
wget https://downloads.xingdong.tech/xhand_teleoperation_linux.tar.gz

# 解压
tar -xzf xhand_teleoperation_linux.tar.gz

# 安装
cd xhand_teleoperation
sudo ./install.sh
\`\`\`

## 使用步骤

### 1. 启动PC端软件

1. 打开"XHAND1 Teleoperation"软件
2. 在"设备类型"中选择"Meta Quest"
3. 点击"启动服务器"
4. 记下显示的IP地址和端口号（如 \`192.168.1.100:8080\`）

### 2. 连接XHAND1

1. 在软件中点击"连接灵巧手"
2. 选择连接方式（EtherCAT或RS485）
3. 配置连接参数
4. 点击"连接"，等待连接成功

### 3. 启动Quest应用

1. 戴上Quest头显
2. 进入"应用库" → "未知来源"
3. 找到并启动"XHAND1 Teleoperation"应用
4. 在应用中输入PC端的IP地址和端口号
5. 点击"连接"

### 4. 校准和配置

#### 手部校准

1. 按照屏幕提示，将双手放在摄像头可见范围内
2. 做出"张开手"的姿势，保持3秒
3. 做出"握拳"的姿势，保持3秒
4. 校准完成后，屏幕会显示"校准成功"

#### Retargeting配置

在PC端软件中，可以调整Retargeting参数：

- **映射模式**：
  - "直接映射"：1:1映射人手动作
  - "优化映射"：自动优化，提高操作精度
  - "任务映射"：针对特定任务优化

- **缩放因子**：调整人手与灵巧手的尺寸比例（建议0.8-1.2）
- **平滑系数**：调整动作的平滑程度（建议0.1-0.5）

### 5. 开始遥操作

1. 在Quest中，您会看到虚拟的XHAND1灵巧手
2. 移动您的右手，虚拟灵巧手会实时跟随
3. 做出抓取、张开等动作，灵巧手会同步执行
4. 在PC端可以看到实时的关节角度和触觉数据

## 操作技巧

### 手势控制

| 手势 | 功能 |
|------|------|
| 张开手 | 灵巧手张开 |
| 握拳 | 灵巧手握拳 |
| 捏取（拇指+食指） | 精密抓取 |
| OK手势 | 切换控制模式 |
| 竖起大拇指 | 确认/保存当前姿态 |

### 菜单操作

- **左手掌心向上**：打开菜单
- **用右手食指点击**：选择菜单项
- **握拳**：关闭菜单

### 视角控制

- **转动头部**：改变观察视角
- **前后移动**：调整观察距离
- **双击空间**：重置视角

## 高级功能

### 1. 双手协同控制

如果您有两只XHAND1灵巧手：

1. 在PC端软件中启用"双手模式"
2. 分别连接左右手灵巧手
3. 在Quest中，双手可以独立控制两只灵巧手

### 2. 力反馈模拟

虽然Quest没有物理力反馈，但软件提供视觉和音频反馈：

- **视觉反馈**：抓取物体时，虚拟手会变色
- **音频反馈**：接触时播放触觉音效
- **震动反馈**：Quest控制器震动（如果使用控制器）

### 3. 录制和回放

在PC端软件中：

1. 点击"开始录制"
2. 执行需要录制的操作
3. 点击"停止录制"
4. 保存为轨迹文件
5. 随时加载并回放

### 4. 远程操作

通过互联网远程控制：

1. 在PC端启用"远程模式"
2. 配置端口转发或使用VPN
3. 在Quest中输入公网IP地址
4. 注意：网络延迟会影响操作体验

## 性能优化

### 提高追踪精度

- **光照条件**：确保环境光线充足且均匀
- **背景环境**：避免复杂的背景干扰
- **手部可见性**：保持手部在摄像头视野内
- **手部姿态**：避免手指遮挡

### 降低延迟

- **网络优化**：使用5GHz Wi-Fi或有线连接
- **减少负载**：关闭其他网络应用
- **降低质量**：在设置中降低视频流质量

### 提高舒适度

- **调整头显**：确保头显佩戴舒适
- **定期休息**：每30分钟休息5-10分钟
- **调整亮度**：根据环境调整屏幕亮度

## 故障排查

### Q: Quest无法连接到PC

A: 请检查：
1. Quest和PC是否在同一局域网
2. 防火墙是否阻止了连接
3. IP地址和端口号是否正确
4. PC端软件是否已启动服务器

### Q: 手部追踪不准确

A: 请尝试：
1. 重新校准手部追踪
2. 改善环境光照条件
3. 清洁Quest摄像头
4. 更新Quest系统版本

### Q: 延迟过大

A: 请尝试：
1. 使用5GHz Wi-Fi或有线连接
2. 减少网络负载
3. 降低视频流质量
4. 在PC端启用"低延迟模式"

### Q: 灵巧手动作不自然

A: 请调整Retargeting参数：
1. 增加平滑系数
2. 调整缩放因子
3. 切换到"优化映射"模式

## 示例应用

### 示例1：远程抓取演示

\`\`\`python
# 在PC端运行此脚本，配合Quest遥操作

import xhand_teleoperation as xt

# 初始化遥操作系统
tele = xt.TeleoperationSystem(device='quest')

# 连接灵巧手
hand = tele.connect_hand(interface='eth0')

# 启动遥操作
tele.start()

# 等待用户操作
print("请在Quest中进行遥操作...")
tele.wait_for_completion()

# 保存操作数据
tele.save_trajectory('demo_grasp.traj')
\`\`\`

### 示例2：数据采集

\`\`\`python
import xhand_teleoperation as xt

tele = xt.TeleoperationSystem(device='quest')
hand = tele.connect_hand()

# 开始数据采集
tele.start_recording('dataset/grasp_001.h5')

# 采集10次抓取动作
for i in range(10):
    print(f"请执行第{i+1}次抓取...")
    tele.wait_for_trigger()  # 等待用户触发
    tele.record_episode()

tele.stop_recording()
print("数据采集完成！")
\`\`\`

## 相关资源

- [Meta Quest 开发者文档](https://developer.oculus.com/)
- [手部追踪最佳实践](https://developer.oculus.com/documentation/native/android/mobile-hands/)
- [XHAND1 遥操作API参考](/docs/api/teleoperation)

## 技术支持

如有任何问题，请联系：
- **邮箱**：quest-support@xingdong.tech
- **Discord社区**：https://discord.gg/xhand1`
    },
    {
      id: "apple-vision-pro",
      title: "Apple Vision Pro 遥操作",
      path: "/docs/teleoperation/apple-vision-pro",
      content: `# Apple Vision Pro 遥操作

## 简介

Apple Vision Pro是苹果公司推出的革命性空间计算设备，配备了业界领先的手部追踪和眼动追踪技术。其高精度的传感器和强大的M2+R1芯片，为XHAND1遥操作提供了极致的体验。

## 核心优势

相比其他VR设备，Vision Pro在遥操作方面具有独特优势：

- **超高精度**：亚毫米级的手部追踪精度
- **低延迟**：端到端延迟<5ms
- **眼动追踪**：结合眼神和手势的多模态控制
- **高分辨率显示**：4K+ micro-OLED显示，细节清晰
- **空间音频**：精准的3D音效反馈

## 系统要求

### Vision Pro要求
- **系统版本**：visionOS 1.0 或更高
- **存储空间**：至少500MB可用空间
- **网络**：Wi-Fi 6 或更高

### Mac端要求
- **操作系统**：macOS 14 Sonoma 或更高
- **处理器**：Apple Silicon (M1/M2/M3) 或 Intel Core i7
- **内存**：16GB RAM 或更高
- **网络**：与Vision Pro在同一局域网

### XHAND1要求
- 已完成硬件连接和基本配置
- 与Mac在同一网络或通过Mac中继

## 安装步骤

### 1. 安装Vision Pro应用

#### 通过App Store安装（推荐）

1. 在Vision Pro上打开App Store
2. 搜索"XHAND1 Teleoperation"
3. 点击"获取"并安装
4. 等待下载和安装完成

#### 通过Xcode安装（开发者）

\`\`\`bash
# 在Mac上克隆项目
git clone https://github.com/xingdong-robotics/xhand_teleoperation_visionos.git

# 打开Xcode项目
cd xhand_teleoperation_visionos
open XHANDTeleoperation.xcodeproj

# 连接Vision Pro并运行
# 在Xcode中选择Vision Pro作为目标设备
# 点击运行按钮
\`\`\`

### 2. 安装Mac端软件

\`\`\`bash
# 下载安装包
curl -O https://downloads.xingdong.tech/xhand_teleoperation_macos.dmg

# 挂载DMG
hdiutil attach xhand_teleoperation_macos.dmg

# 拖拽到Applications文件夹
cp -R "/Volumes/XHAND Teleoperation/XHAND Teleoperation.app" /Applications/

# 卸载DMG
hdiutil detach "/Volumes/XHAND Teleoperation"
\`\`\`

## 使用步骤

### 1. 启动Mac端软件

1. 打开"XHAND1 Teleoperation"应用
2. 在"设备类型"中选择"Apple Vision Pro"
3. 点击"启动服务"
4. 软件会自动发现局域网内的Vision Pro设备

### 2. 连接XHAND1

1. 点击"连接灵巧手"
2. 选择连接方式（EtherCAT或RS485）
3. 配置连接参数
4. 点击"连接"

### 3. 启动Vision Pro应用

1. 戴上Vision Pro
2. 在主屏幕找到"XHAND1 Teleoperation"应用
3. 用眼神注视并捏合手指打开应用
4. 应用会自动搜索并连接到Mac端

### 4. 空间校准

Vision Pro的空间校准非常重要：

1. **定义工作空间**：
   - 用手指在空中画出一个矩形区域
   - 这将作为遥操作的工作空间
   - 建议大小：60cm × 60cm × 60cm

2. **设置原点**：
   - 将右手放在舒适的起始位置
   - 捏合拇指和食指3秒
   - 这将设置为灵巧手的"零位"

3. **校准手部姿态**：
   - 按照屏幕提示做出以下姿势：
     - 张开手（5秒）
     - 握拳（5秒）
     - 捏取姿势（5秒）
   - 每个姿势保持稳定，让系统学习您的手部特征

### 5. Retargeting配置

Vision Pro版本提供了更高级的Retargeting选项：

#### 基础模式
- **直接映射**：1:1映射，最直观
- **缩放映射**：自动调整尺寸比例
- **优化映射**：AI优化，提高精度

#### 高级模式
- **任务导向映射**：针对特定任务优化
- **学习型映射**：从用户操作中学习
- **混合映射**：结合多种策略

### 6. 开始遥操作

1. 在Vision Pro中，您会看到：
   - 虚拟的XHAND1灵巧手（高精度3D模型）
   - 实时的关节角度显示
   - 触觉传感器热力图
   - 操作提示和反馈

2. 移动您的右手：
   - 灵巧手会实时跟随
   - 延迟极低（<5ms）
   - 动作流畅自然

3. 执行操作：
   - 抓取、张开、捏取等动作
   - 灵巧手会精确复现
   - 触觉反馈通过视觉和音频呈现

## 独特功能

### 1. 眼动辅助控制

Vision Pro的眼动追踪可以辅助手部控制：

- **视线聚焦**：注视物体，手部动作更精确
- **意图预测**：根据视线预测操作意图
- **多模态融合**：眼神+手势的协同控制

使用方法：
1. 在设置中启用"眼动辅助"
2. 注视目标物体
3. 执行抓取动作
4. 系统会自动优化抓取姿态

### 2. 空间锚点

利用Vision Pro的空间感知能力：

- **虚拟物体放置**：在真实环境中放置虚拟参考物
- **工作区域标记**：标记安全操作区域
- **多视角观察**：从不同角度观察灵巧手

### 3. 协作模式

多人协作遥操作：

- **观察者模式**：其他人可以通过Mac或iPad观看
- **教学模式**：专家演示，学习者跟随
- **接力模式**：多人轮流控制

### 4. 数字孪生

实时的数字孪生体验：

- **物理仿真**：预测抓取结果
- **碰撞检测**：避免危险操作
- **轨迹规划**：自动规划最优路径

## 高级应用

### 1. 精密装配

利用Vision Pro的高精度：

\`\`\`swift
// Swift代码示例（在Vision Pro上运行）
import XHANDTeleoperation

let tele = XHANDTeleoperation()
tele.connect()

// 启用精密模式
tele.setPrecisionMode(true)
tele.setScaleFactor(0.5)  // 缩小动作幅度，提高精度

// 开始装配任务
tele.startTask("precision_assembly")
\`\`\`

### 2. 远程手术辅助

医疗应用场景：

\`\`\`swift
import XHANDTeleoperation

let tele = XHANDTeleoperation()
tele.connect()

// 启用安全模式
tele.setSafetyMode(true)
tele.setForceLimit(5.0)  // 限制最大力度

// 启用力反馈模拟
tele.enableHapticFeedback(true)

// 开始手术辅助
tele.startTask("surgical_assistance")
\`\`\`

### 3. 艺术创作

用于雕刻、绘画等艺术应用：

\`\`\`swift
import XHANDTeleoperation

let tele = XHANDTeleoperation()
tele.connect()

// 启用艺术模式
tele.setArtisticMode(true)
tele.enableMotionSmoothing(0.8)  // 高度平滑

// 录制创作过程
tele.startRecording("sculpture_session_001")
\`\`\`

## 性能指标

Vision Pro版本的遥操作性能：

| 指标 | 数值 |
|------|------|
| **手部追踪精度** | < 1 mm |
| **手部追踪频率** | 90 Hz |
| **端到端延迟** | < 5 ms |
| **姿态精度** | < 2° |
| **工作空间** | 1m³ |
| **同时追踪** | 双手 + 眼动 |

## 故障排查

### Q: Vision Pro无法发现Mac

A: 请检查：
1. 确保在同一Wi-Fi网络
2. 关闭Mac的防火墙或添加例外
3. 重启Vision Pro和Mac端软件
4. 检查网络设置中的"本地网络"权限

### Q: 手部追踪不稳定

A: 请尝试：
1. 改善环境光照（自然光最佳）
2. 避免手部遮挡摄像头
3. 重新校准手部追踪
4. 更新visionOS到最新版本

### Q: 延迟过大

A: 请检查：
1. 使用Wi-Fi 6或更高
2. 减少网络负载
3. 在Mac端启用"性能模式"
4. 关闭其他占用网络的应用

### Q: 动作映射不准确

A: 请调整：
1. 重新进行空间校准
2. 调整Retargeting参数
3. 尝试不同的映射模式
4. 在设置中微调缩放因子

## 开发者资源

### SDK集成

\`\`\`swift
import XHANDTeleoperation

// 初始化遥操作系统
let tele = XHANDTeleoperation()

// 设置回调
tele.onHandPoseUpdate = { pose in
    print("手部姿态更新: \\(pose)")
}

tele.onTactileData = { data in
    print("触觉数据: \\(data)")
}

// 连接并启动
tele.connect(host: "192.168.1.100")
tele.start()
\`\`\`

### 自定义Retargeting

\`\`\`swift
// 创建自定义Retargeting策略
let customRetargeting = CustomRetargeting()

customRetargeting.setMappingRule(
    from: .humanThumb,
    to: .robotThumb,
    transform: { humanPose in
        // 自定义转换逻辑
        return transformedPose
    }
)

tele.setRetargetingStrategy(customRetargeting)
\`\`\`

## 相关资源

- [Apple Vision Pro 开发者文档](https://developer.apple.com/visionos/)
- [ARKit 手部追踪指南](https://developer.apple.com/documentation/arkit/arkit_hand_tracking)
- [XHAND1 Vision Pro API参考](/docs/api/teleoperation-visionos)

## 技术支持

如有任何问题，请联系：
- **邮箱**：visionpro-support@xingdong.tech
- **技术论坛**：https://forum.xingdong.tech/visionpro`
    },
    {
      id: "manus-glove",
      title: "MANUS手套 遥操作",
      path: "/docs/teleoperation/manus-glove",
      content: `# MANUS手套 遥操作

## 简介

MANUS是业界领先的专业数据手套制造商，其产品广泛应用于动作捕捉、VR/AR、机器人遥操作等领域。MANUS手套提供了极高的精度和可靠性，是专业应用的首选。

XHAND1与MANUS手套深度集成，提供了完整的遥操作解决方案。

## 支持的MANUS产品

| 产品型号 | 传感器数量 | 精度 | 力反馈 | 推荐程度 |
|---------|-----------|------|--------|---------|
| **MANUS Quantum** | 20+ | 极高 | ✅ | ⭐⭐⭐⭐⭐ 最推荐 |
| **MANUS Prime II** | 12 | 高 | ❌ | ⭐⭐⭐⭐ 推荐 |
| **MANUS Meta** | 8 | 中 | ❌ | ⭐⭐⭐ 可用 |

## 核心优势

### 1. 极高精度

MANUS手套使用IMU（惯性测量单元）和磁力计的组合：

- **关节角度精度**：< 1°
- **位置精度**：< 2mm
- **采样频率**：高达1000Hz
- **无漂移**：磁力计校正，长时间使用无累积误差

### 2. 力反馈（Quantum型号）

MANUS Quantum配备了先进的力反馈系统：

- **每个手指独立反馈**
- **力度范围**：0-20N
- **响应时间**：< 10ms
- **真实触感**：模拟物体的硬度和形状

### 3. 全手追踪

覆盖手部所有关键关节：

- **拇指**：CMC、MCP、IP（3个关节）
- **其他手指**：MCP、PIP、DIP（每指3个关节）
- **手腕**：腕关节姿态
- **手掌**：手掌朝向

## 系统要求

### 硬件要求
- **MANUS手套**：Quantum/Prime II/Meta任意型号
- **MANUS Core**：MANUS配套的接收器和处理单元
- **PC**：
  - 处理器：Intel Core i7 或 AMD Ryzen 7
  - 内存：16GB RAM
  - 显卡：NVIDIA GTX 1060 或更高（用于可视化）
  - USB接口：USB 3.0或更高

### 软件要求
- **操作系统**：Windows 10/11 (64位)
- **MANUS Core软件**：v2.0或更高
- **XHAND1遥操作软件**：最新版本

## 安装步骤

### 1. 安装MANUS Core

1. 从[MANUS官网](https://www.manus-meta.com/)下载MANUS Core软件
2. 运行安装程序
3. 按照向导完成安装
4. 重启计算机

### 2. 连接MANUS手套

#### 有线连接（推荐）

1. 将MANUS Core接收器连接到PC的USB 3.0接口
2. 将手套的数据线连接到MANUS Core
3. 打开MANUS Core软件
4. 等待手套连接成功（指示灯变绿）

#### 无线连接

1. 确保手套已充电
2. 打开手套电源开关
3. 在MANUS Core中点击"搜索设备"
4. 选择您的手套并连接

### 3. 校准MANUS手套

校准是确保精度的关键步骤：

#### 磁力计校准

1. 在MANUS Core中选择"校准" → "磁力计校准"
2. 按照屏幕提示，缓慢旋转手套
3. 确保覆盖所有方向
4. 等待校准完成

#### 手部姿态校准

1. 选择"校准" → "手部姿态校准"
2. 做出以下姿势并保持3秒：
   - **平展手**：手指完全伸直
   - **握拳**：手指完全弯曲
   - **自然手**：放松状态
3. 校准完成后保存配置

### 4. 安装XHAND1遥操作软件

1. 下载安装包：[XHAND_Teleoperation_MANUS.exe](/downloads/XHAND_Teleoperation_MANUS.exe)
2. 运行安装程序
3. 选择安装路径
4. 完成安装

## 使用步骤

### 1. 启动系统

1. **启动MANUS Core**：
   - 打开MANUS Core软件
   - 确认手套已连接（绿色指示）
   - 检查数据流是否正常

2. **启动XHAND1遥操作软件**：
   - 打开"XHAND1 Teleoperation"
   - 在"设备类型"中选择"MANUS Glove"
   - 选择手套型号（Quantum/Prime II/Meta）
   - 点击"连接MANUS"

3. **连接XHAND1灵巧手**：
   - 点击"连接灵巧手"
   - 配置连接参数（EtherCAT/RS485）
   - 点击"连接"

### 2. Retargeting配置

MANUS手套版本提供了最精细的Retargeting控制：

#### 关节映射

可以为每个关节单独配置映射关系：

| 人手关节 | XHAND1关节 | 映射比例 |
|---------|-----------|---------|
| 拇指CMC | thumb_cmc | 0.8-1.2 |
| 拇指MCP | thumb_mcp | 0.8-1.2 |
| 拇指IP | thumb_ip | 0.8-1.2 |
| 食指MCP | index_mcp | 0.8-1.2 |
| ... | ... | ... |

#### 高级选项

- **死区设置**：过滤小幅度抖动
- **速度限制**：限制最大运动速度
- **力矩限制**：保护灵巧手
- **平滑滤波**：调整动作平滑度

### 3. 力反馈配置（仅Quantum）

如果您使用MANUS Quantum手套：

1. 在软件中启用"力反馈"
2. 配置反馈参数：
   - **反馈强度**：0-100%
   - **触发阈值**：接触力阈值（N）
   - **反馈模式**：
     - "直接映射"：直接映射触觉传感器数据
     - "物理模拟"：基于物理仿真的反馈
     - "自定义"：自定义反馈曲线

3. 测试力反馈：
   - 让XHAND1抓取物体
   - 您应该能在手套中感受到相应的力反馈

### 4. 开始遥操作

1. 戴上MANUS手套
2. 将手放在舒适的位置
3. 在软件中点击"开始遥操作"
4. 移动手指，观察XHAND1的响应
5. 执行各种操作任务

## 专业应用

### 1. 高质量数据采集

MANUS手套是采集机器人学习数据的理想工具：

\`\`\`python
import xhand_teleoperation as xt

# 初始化MANUS遥操作系统
tele = xt.TeleoperationSystem(device='manus', model='quantum')
hand = tele.connect_hand(interface='eth0')

# 配置数据采集
tele.configure_recording(
    frequency=1000,  # 1000Hz采样
    modalities=['joint_pos', 'joint_vel', 'joint_torque', 'tactile', 'force_feedback'],
    format='hdf5'
)

# 采集100个抓取样本
for i in range(100):
    print(f"采集样本 {i+1}/100")
    
    # 等待操作员准备
    input("按Enter开始...")
    
    # 开始录制
    tele.start_recording(f'dataset/grasp_{i:03d}.h5')
    
    # 执行抓取
    print("请执行抓取动作...")
    tele.wait_for_completion()
    
    # 停止录制
    tele.stop_recording()
    
    print("样本采集完成！")

print("数据采集完成！")
\`\`\`

### 2. 精密装配任务

利用MANUS的高精度进行精密操作：

\`\`\`python
import xhand_teleoperation as xt

tele = xt.TeleoperationSystem(device='manus', model='quantum')
hand = tele.connect_hand()

# 启用精密模式
tele.set_precision_mode(True)
tele.set_force_limit(5.0)  # 限制力度，避免损坏零件

# 启用力反馈
tele.enable_force_feedback(True)
tele.set_feedback_gain(0.8)

# 执行装配任务
print("开始精密装配任务...")
tele.start_task('precision_assembly')

# 实时监控
while tele.is_running():
    status = tele.get_status()
    print(f"当前力: {status['contact_force']:.2f} N")
    
    # 如果力过大，警告
    if status['contact_force'] > 10.0:
        print("警告：接触力过大！")
        tele.pause()

tele.stop()
\`\`\`

### 3. 双手协同操作

使用两只MANUS手套控制两只XHAND1：

\`\`\`python
import xhand_teleoperation as xt

# 初始化双手系统
tele = xt.TeleoperationSystem(device='manus', hands='both')

# 连接左右手灵巧手
left_hand = tele.connect_hand(hand_id=0, interface='eth0')
right_hand = tele.connect_hand(hand_id=1, interface='eth1')

# 配置协同模式
tele.set_coordination_mode('independent')  # 或 'synchronized'

# 开始双手操作
tele.start()

print("双手遥操作已启动！")
print("左手控制左侧灵巧手，右手控制右侧灵巧手")

tele.wait_for_completion()
\`\`\`

### 4. 远程手术辅助

医疗应用中的高精度要求：

\`\`\`python
import xhand_teleoperation as xt

tele = xt.TeleoperationSystem(device='manus', model='quantum')
hand = tele.connect_hand()

# 启用医疗模式
tele.set_medical_mode(True)
tele.set_safety_limits(
    max_force=3.0,      # 最大力度3N
    max_velocity=0.1,   # 最大速度0.1 m/s
    workspace_bounds=[  # 限制工作空间
        [-0.1, 0.1],    # X范围
        [-0.1, 0.1],    # Y范围
        [0.0, 0.2]      # Z范围
    ]
)

# 启用力反馈和震动警告
tele.enable_force_feedback(True)
tele.enable_haptic_warnings(True)

# 记录手术过程
tele.start_recording('surgery_session_001.h5')

print("医疗辅助模式已启动")
print("安全限制已激活")

tele.start()
\`\`\`

## 性能指标

MANUS手套 + XHAND1 系统性能：

| 指标 | MANUS Quantum | MANUS Prime II | MANUS Meta |
|------|--------------|----------------|------------|
| **关节角度精度** | < 1° | < 2° | < 5° |
| **位置精度** | < 2mm | < 5mm | < 10mm |
| **采样频率** | 1000Hz | 500Hz | 250Hz |
| **端到端延迟** | < 5ms | < 10ms | < 20ms |
| **力反馈** | ✅ 0-20N | ❌ | ❌ |
| **工作时间** | 8小时 | 6小时 | 4小时 |

## 故障排查

### Q: 手套无法连接

A: 请检查：
1. USB线是否连接牢固
2. MANUS Core是否正常运行
3. 手套电量是否充足（无线模式）
4. 驱动程序是否正确安装

### Q: 追踪精度下降

A: 请尝试：
1. 重新校准磁力计
2. 远离强磁场干扰源
3. 检查手套是否佩戴正确
4. 更新MANUS Core到最新版本

### Q: 力反馈不工作

A: 请检查：
1. 是否使用Quantum型号
2. 力反馈是否在软件中启用
3. 反馈强度是否设置过低
4. 手套固件是否最新

### Q: 延迟过大

A: 请尝试：
1. 使用有线连接
2. 降低采样频率
3. 关闭其他占用CPU的程序
4. 升级到更高性能的PC

## 维护和保养

### 日常维护

- **清洁**：用干净的软布擦拭手套表面
- **存放**：放在干燥、阴凉的地方
- **充电**：每次使用后充电，避免过度放电
- **检查**：定期检查线缆和连接器

### 定期校准

建议每周进行一次完整校准：

1. 磁力计校准
2. 手部姿态校准
3. 力反馈校准（Quantum）

### 固件更新

定期检查并更新手套固件：

1. 打开MANUS Core
2. 点击"设置" → "固件更新"
3. 如有可用更新，点击"更新"
4. 等待更新完成（约5-10分钟）

## 相关资源

- [MANUS官方文档](https://www.manus-meta.com/documentation)
- [MANUS开发者论坛](https://forum.manus-meta.com/)
- [XHAND1 + MANUS集成指南](/docs/guides/manus-integration)
- [数据采集最佳实践](/docs/guides/data-collection)

## 技术支持

如有任何问题，请联系：

**XHAND1技术支持**：
- 邮箱：manus-support@xingdong.tech
- 电话：+86-xxx-xxxx-xxxx

**MANUS技术支持**：
- 邮箱：support@manus-meta.com
- 网站：https://www.manus-meta.com/support`
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
