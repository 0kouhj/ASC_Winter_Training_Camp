# ASC_Winter_Training_Camp
## 项目成员
邓涵天 王泽源
## 仓库文件结构
- docs/ : 存放项目相关文档与图文资料
- Firmware/ : 放置单片机项目工程代码
- Hardware/ : 放置电路原理图及 PCB 源文件
- Mechanical/ : 放置各类机械结构 3D 模型文件 (STL/STEP)
## 项目代码结构 (Firmware)
- 工程基础文件 : 芯片启动文件及HAL库
- App/ : 应用层，用于存放控制算法（如 PID、滤波等）
- Bsp/ : 板级支持包，负责底层外设驱动（如 电机、传感器驱动）
- Config/ : 配置文件，存放系统参数设置与小车物理数据
## PCB丝印
![PCB_V2.0](https://raw.githubusercontent.com/0kouhj/ASC_Winter_Training_Camp/main/docs/IMG/PCB_V2.0_2026-01-17.png)
## 硬件选型
1. 电机:MG513(GMR) 12V 1:30
2. 68mm硅胶轮胎
3. ICM42688姿态传感器
