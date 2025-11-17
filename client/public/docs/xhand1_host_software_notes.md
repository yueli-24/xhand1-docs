# XHAND1 上位机用户快速使用说明关键信息

## 环境说明

**操作系统**：Ubuntu 20.04 LTS (Focal Fossa)

**浏览器要求**：
- 推荐使用谷歌浏览器
- 版本不低于 130.0.6723.69（正式版本）（64位）

**硬件要求**：
- CPU: Intel i5-12400 及以上
- 内存: 8GB 及以上
- 硬盘: 128GB SSD 及以上

## 软件安装

### 1. 下载XHand安装包

下载地址：https://di6kz6gamrw.feishu.cn/drive/folder/WGyhflqb1lRu9ddtc0scjDhwngg

### 2. 首次安装需要联网安装前置依赖

**安装 JDK**（需大于等于 8，建议 jdk 在 8-17 之间）：
```bash
sudo apt-get update
sudo apt-get install -y openjdk-8-jdk
```

**安装 pip3**：
```bash
sudo apt-get install -y python3-pip
sudo apt install libboost-filesystem-dev -y
sudo pip3 install pybind11  # 上位机版本大于等于 v1.1.7 的不执行此命令
sudo apt install -y nlohmann-json3-dev  # 上位机版本大于等于 v1.1.10 的需要执行此命令
```

### 3. 验证安装

基于 Ubuntu 20.04 LTS，安装结果校验：

```bash
$ java -version
openjdk version "1.8.0_422"
OpenJDK Runtime Environment (build 1.8.0_422-8u422-b05-1~20.04-b05)
OpenJDK 64-Bit Server VM (build 25.422-b05, mixed mode)

$ python3 --version
Python 3.8.10

$ pip3 --version
pip 20.0.2 from /usr/lib/python3/dist-packages/pip (python 3.8)

$ dpkg -l | grep libboost-filesystem-dev
ii libboost-filesystem-dev:amd64 1.71.0.0ubuntu2 amd64
filesystem operations (portable paths, iteration over directories, etc) in C++
(default version)
```

### 4. 安装XHand软件

通过终端安装 xhand.deb 安装包进行安装：

```bash
# 卸载
sudo dpkg -r xhand

# 安装
sudo dpkg -i xhand_v1.x.x.deb
```

**注意**：请在普通用户权限下进行安装，不支持 root 权限及免密权限用户下安装

## 设备连接

### 1. 硬件连接

XH04 调试线与 XHand 连接，电源适配器与电源接口连接。

### 2. 通讯线缆连接

通过 XH04 通讯调试线缆连接灵巧手与电脑：

- **USB 接口和 RJ45 接口**：只需连接一个
  - **EtherCAT 协议**请使用 RJ45 接口
  - **485 协议**请使用 USB 接口

### 3. 启动XOS应用

双击【XHAND】应用，等待 10s 弹窗输入密码，输入密码后会浏览器自动弹窗 XOS 上位机界面。

### 4. 搜索连接设备

将灵巧手设备与电脑通过 XH04 通讯调试线连接后：
1. 点击【搜索连接】开始尝试搜索连接设备
2. 选择【EtherCAT】通讯协议
3. 点击【确定】
4. 选择设备 ID（自动读取）
5. 点击【确定】进入下一步

### 5. 连接成功

连接成功后进入 XOS 主界面。

## 主页展示

### 操作步骤

**1. 完成标定任务后**，进入 XHand 主页展示灵巧手设备基本信息：
- 电流
- 扭矩
- 关节角度
- 指尖触觉综合力值信息

**2. 当使用灵巧手抓握物体后**：
- 主页展示指尖触觉综合力值等信息
- 当灵巧手设备松开物体后，合力值仍展示数据
- 点击【传感器清零】按钮，将合力值做归零操作

## 设备调试

### 1. 调试设备

点击【调试设备】，测试灵巧手是否处于联动状态。

### 2. 位置模式

切换【位置模式】拖动滑块查看灵巧手每个指关节角度变化，检查灵巧手是否可以正常运动。

## 设备标定（默认不需要）

如果 XHand 1 初始化位置**未在默认零位**，需要对灵巧手设备进行标定：

1. 先将 XHand 1 放入标定模具内
2. 点击【开始标定】按钮
3. 执行一键标定程序

标定完成后 XOS 上位机软件会弹窗提示标定成功，此时可正常按需使用 XHand 1。

## 维保体系

- **官网链接**：www.robotera.com
- **帮助支持**：support@robotera.com
