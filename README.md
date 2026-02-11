# HKU ATI Soft Exoskeleton Controller Firmware

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)> 
 


## 📖 简介 (Introduction)

本工程 (`HipRobotProject_Basic_DMA`) 是专为 **HKU ATI ** 软体外骨骼机器人开发的控制器基础固件。

该代码旨在为底层硬件控制提供稳健的驱动支持，适配 HKU ATI 自研控制器与深圳智能机器人研究院开发的 FPGA 电机驱动器。工程实现了基础的功能测试、电机闭环控制及实时数据回传功能，是进一步开发高级步态控制算法或人在环（Human-in-the-loop）优化策略的基础平台。

> **注意**：本仓库代码主要包含底层驱动与基础控制逻辑，**不包含**高级步态规划或复杂的优化算法。

---

## 🛠️ 硬件架构 (Hardware Setup)

本系统采用模块化硬件设计，核心组件包括 HKU ATI 自研主控制器与高性能 FPGA 电机驱动器。

### 核心组件
1.  **主控制器 (Main Controller)**: HKU ATI 自研开发板。
2.  **电机驱动器 (Motor Driver)**: 深圳智能机器人研究院 (Shenzhen Institute of Artificial Intelligence and Robotics for Society) 开发的 FPGA 4轴电机驱动器。

### 连接说明
*   **接口标准**: DB15 接口。
*   **接线方式**: Pin-to-Pin 直连（即控制器的 Pin 1 对应驱动器的 Pin 1，以此类推）。

| 组件 | 说明 |
| :--- | :---  |
| **主控制器** | HKU ATI 自研控制器，集成多种通讯接口与模拟电路。  |
| **电机驱动器** | 基于 FPGA 的 4 通道电机驱动方案。 |

### 硬件配置与调试
在使用前，请确保完成以下硬件配置：

*   **模拟电路调整**: 需调整主控制器上的放大器增益。
    *   *推荐配置*: 增益电阻 **2kΩ**，适配量程为 **0-500N** 的力传感器。
*   **调试接口**: 支持无线 **DAPLINK** 进行代码下载与在线调试。
*   **IMU 配置**:
    *   型号: 维特智能 (WitMotion) WT901-485
    *   回传频率: **25Hz**
*   **CAN 总线记录仪**: 推荐使用 ZLG USBCAN II+ 或 ZLG WIFI-CAN TCP 进行数据监控。

### 遥控器操作说明
系统配备专用遥控器用于快速测试与模式切换。


*   **按键 A**: 电流模式 - 正转 (Current Mode - Forward)
*   **按键 B**: 速度模式 - 反转 (Speed Mode - Reverse)
*   **按键 C**: 紧急停机 (Stop)
*   **按键 D**: 复位/回到初始位置 (Reset/Home)

---

## 💻 软件架构 (Software Architecture)

本固件基于 **FreeRTOS V1** 实时操作系统开发，确保了任务调度的实时性与稳定性。系统核心任务及其对应的入口函数如下：

| 任务类型 | 任务函数名 (Task Function) | 功能描述 |
| :--- | :--- | :--- |
| **核心控制** | `TSAxContollFUN` | 负责电机的底层控制逻辑（如PID计算）与指令发送。内部包含2个电机控制线程。 |
| **数据解码** | `MotorRXDecodeGxFUN` | 负责解析来自 FPGA 驱动器的编码器反馈数据。 |
| **姿态解算** | `IMU_TaskFUN` | 专门用于解析 WT901-485 IMU 的姿态数据。 |
| **数据回传** | `StateRETFUN` | 负责收集系统状态，并通过 CAN 或 UART 进行数据打包回传。 |

此外，系统还包含若干软件定时器 (Software Timers) 用于辅助任务，具体配置可在 STM32CubeMX (`.ioc` 文件) 中查看与修改。

---

## 🔌 扩展能力 (Expansion Capabilities)

HKU ATI 控制器预留了丰富的接口，支持高度的定制化扩展：

### 接口资源列表
*   **2x RS422**: 可用于连接额外的工业级传感器。
*   **1x RS485**: 默认用于连接 IMU 阵列。
*   **2x CAN BUS**:
    *   CAN1: 数据回传 / 调试。
    *   CAN2: 连接肌电 (EMG) 传感器或其他 CAN 设备。
*   **2x Loadcell In**: 模拟量输入，用于读取力传感器数据（感知驱动器输出力）。
*   **1x UART I/O**: 通用串行通讯。
*   **3x GPIO I/O**: 通用输入输出，可用于连接遥控器接收机或触发信号。

### 系统扩展潜力
基于上述接口，本系统理论上支持：
1.  **8轴电机控制**: 通过级联 2 个 FPGA 驱动器，最多控制 8 个 16mm 微型电机。
2.  **全身姿态感知**: 支持连接最多 4 个 IMU 模块。
3.  **力交互控制**: 配合 2 路力传感器接口实现力位混合控制。

---

## 🚀 快速开始 (Getting Started)

1.  克隆本仓库:
    ```bash
    git clone https://github.com/justinling97/HipRobotProject_Basic_DMA.git
    ```
2.  使用 STM32CubeIDE 或 Keil MDK 打开工程文件。
3.  连接无线 DAPLINK 至控制器。
4.  编译并下载固件。
5.  打开 ZLG ZCANPRO 软件监控数据流。

---

## 📄 许可证 (License)

本项目采用 **Apache License 2.0** 许可证。详情请参阅 [LICENSE](LICENSE) 文件。

---

**HKU ATI Lab**
