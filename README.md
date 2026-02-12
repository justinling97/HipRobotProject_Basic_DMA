# HKU ATI Soft Exoskeleton Controller Firmware

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)> 
 
中文版本  CHINESE Version

## 📖 简介 (Introduction)

本工程 (`HipRobotProject_Basic_DMA`) 是专为 **HKU ATI ** 软体外骨骼机器人开发的控制器基础固件。

该代码旨在为底层硬件控制提供稳健的驱动支持，适配 HKU ATI 自研控制器与深圳智能机器人研究院开发的 FPGA 电机驱动器。工程实现了基础的功能测试、电机闭环控制及实时数据回传功能，是进一步开发高级步态控制算法或人在环（Human-in-the-loop）优化策略的基础平台。

> **注意**：本仓库代码主要包含底层驱动与基础控制逻辑，**不包含**高级步态规划或复杂的优化算法。

---

## 🛠️ 硬件架构 (Hardware Setup)

本系统采用模块化硬件设计，核心组件包括 HKU ATI 自研主控制器与高性能 FPGA 电机驱动器。

### 核心组件
1.  **主控制器 (Main Controller)**: HKU ATI 自研开发板。
2.  **电机驱动器 (Motor Driver)**: 深圳智能机器人研究院 (SZAR) 开发的 FPGA 4电机驱动器。

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


英文版本 ENGLISH Version

# HKU ATI Soft Exoskeleton Controller Firmware

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 📖 Introduction

This project (`HipRobotProject_Basic_DMA`) provides the foundational firmware designed for the soft exoskeleton robot developed by **HKU ATI**.

The code is engineered to provide robust low-level hardware support, specifically adapted for the HKU ATI custom controller and the FPGA motor drivers developed by the Shenzhen Institute of Artificial Intelligence and Robotics for Society (AIRS). This project implements basic functional testing, closed-loop motor control, and real-time data telemetry. It serves as the base platform for further development of advanced gait control algorithms or Human-in-the-loop optimization strategies.

> **Note**: This repository contains low-level drivers and basic control logic. It **does not** include advanced gait planning or complex optimization algorithms.

---

## 🛠️ Hardware Setup

The system utilizes a modular hardware design, featuring the HKU ATI custom main controller and high-performance FPGA motor drivers.

### Core Components
1.  **Main Controller**: HKU ATI Custom Development Board.
2.  **Motor Driver**: FPGA 4-Axis Motor Driver developed by the SZAR.

### Connection Guide
*   **Interface Standard**: DB15 Interface.
*   **Wiring Method**: Pin-to-Pin direct connection (Pin 1 of the controller corresponds to Pin 1 of the driver, and so on).

| Component | Description |
| :--- | :---|
| **Main Controller** | HKU ATI custom controller with integrated communication interfaces and analog circuits. |
| **Motor Driver** | FPGA-based 4-channel motor driver solution.  |

### Configuration & Debugging
Before operation, please ensure the following hardware configurations are met:

*   **Analog Circuit Adjustment**: The amplifier gain on the main controller needs adjustment.
    *   *Recommended*: **2kΩ** gain resistor, adapted for **0-500N** load cells.
*   **Debugging Interface**: Supports wireless **DAPLINK** for code flashing and online debugging.
*   **IMU Configuration**:
    *   Model: WitMotion WT901-485.
    *   Telemetry Frequency: **25Hz**.
*   **CAN Bus Analyzer**: Recommended to use ZLG USBCAN II+ or ZLG WIFI-CAN TCP for data monitoring.

### Remote Control Operation
The system is equipped with a dedicated remote controller for quick testing and mode switching (as shown in Figure 3).


*   **Button A**: Current Mode - Forward Rotation
*   **Button B**: Speed Mode - Reverse Rotation
*   **Button C**: Emergency Stop
*   **Button D**: Reset / Return to Home Position

---

## 💻 Software Architecture

The firmware is built on **FreeRTOS V1**, ensuring real-time performance and stability for task scheduling. The core tasks and their corresponding entry functions are as follows:

| Task Type | Function Name | Description |
| :--- | :--- | :--- |
| **Core Control** | `TSAxContollFUN` | Handles low-level motor control logic (e.g., PID calculation) and command transmission. Contains 2 internal motor control threads. |
| **Data Decoding** | `MotorRXDecodeGxFUN` | Decodes encoder feedback data received from the FPGA drivers. |
| **IMU Parsing** | `IMU_TaskFUN` | Dedicated to parsing attitude data from the WT901-485 IMU. |
| **Telemetry** | `StateRETFUN` | Collects system state and packages data for transmission via CAN or UART. |

Additionally, the system includes several Software Timers for auxiliary tasks. Specific configurations can be viewed and modified in the STM32CubeMX (`.ioc`) file.

---

## 🔌 Expansion Capabilities

The HKU ATI controller reserves a rich set of interfaces to support high-level customization:

### Interface Resources
*   **2x RS422**: For connecting additional industrial-grade sensors.
*   **1x RS485**: Default for connecting IMU arrays.
*   **2x CAN BUS**:
    *   CAN1: Data telemetry / Debugging.
    *   CAN2: Connecting EMG sensors or other CAN devices.
*   **2x Loadcell In**: Analog inputs for reading force sensor data (sensing driver output force).
*   **1x UART I/O**: General-purpose serial communication.
*   **3x GPIO I/O**: General-purpose I/O, usable for connecting remote control receivers or trigger signals.

### System Potential
Based on these interfaces, the system theoretically supports:
1.  **8-Axis Motor Control**: By cascading 2 FPGA drivers, controlling up to 8 units of 16mm micro-motors.
2.  **Full-Body Sensing**: Supports connection of up to 4 IMU modules.
3.  **Force Interaction**: Implements force-position hybrid control using the 2 load cell interfaces.

---

## 🚀 Getting Started

1.  Clone the repository:
    ```bash
    git clone https://github.com/justinling97/HipRobotProject_Basic_DMA.git
    ```
2.  Open the project using STM32CubeIDE or Keil MDK.
3.  Connect the Wireless DAPLINK to the controller.
4.  Compile and flash the firmware.
5.  Open ZLG ZCANPro software to monitor the data stream.

---

## 📄 License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

---

**HKU ATI Lab**







---

**HKU ATI Lab**
