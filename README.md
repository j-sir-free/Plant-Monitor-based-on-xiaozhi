# 植物生长环境监控系统 — 项目文件说明

## 作者声明

本项目由 **j-sir-free** 开发维护，基于 [小智 AI 聊天机器人](https://github.com/j-sir-free/Plant-Monitor-based-on-xiaozhi) 项目的 `bread-compact-wifi-s3cam` 板级代码修改而来。

硬件平台：ESP32-S3-CAM 开发板 + OV2640 摄像头 + ST7789/ILI9341/GC9A01 LCD（240x240）
开源协议：继承上游项目协议(详情见网址：https://github.com/78/xiaozhi-esp32)

> 注意1：摄像头占用了 ESP32-S3 的 GPIO19/20（USB 引脚），因此无法同时使用 USB 和 LCD。
> 注意2：作者只保留了`bread-compact-wifi-s3cam`一种板型，更多板型请前往虾哥开源网址。

---

## 项目概述

这是一个面向植物生长环境的智能监控系统，运行在 ESP32-S3 上，具备以下功能：

- **环境传感**：DHT11 温湿度、光照传感器（ADC）、土壤湿度传感器（模拟量 + 数字量双模）
- **自动控制**：水泵、补光灯、加热片三路继电器自动控制（根据阈值智能调节）
- **LCD 显示**：SPI 接口 LCD 实时显示传感器数值、阈值、继电器状态
- **摄像头监控**：OV2640 摄像头 MJPEG 流 + 快照
- **Web 服务器**：内嵌网页，实时查看传感器数据和摄像头画面
- **云平台对接**：OneNET MQTT 数据上传与远程控制
- **AI 集成**：11 个 MCP 工具，支持小智 AI 语音助手进行环境分析和自动控制

---

## 文件清单与功能说明

### 1. `config.h` — 引脚定义与硬件配置

| 模块 | 关键定义 |
|------|---------|
| 音频 I2S | GPIO1/2/42 (麦克风)，GPIO39/40/41 (扬声器) |
| 板级 IO | LED=GPIO48，启动按钮=GPIO0 |
| 摄像头 OV2640 | D0-D7(11,9,8,10,12,18,17,16)，XCLK=15，PCLK=13，VSYNC=6，HREF=7，SCCB SDA=4 SCL=5 |
| LCD SPI | MOSI=20，CLK=19，DC=47，RST=21，CS=45，背光=38。（为什么不用PCF8574连接LCD：SPI LCD 需要 40MHz 时钟，PCF8574 通过 I2C 通信最高只能到 ~100kHz，差了 400 倍，屏幕根本无法工作。）|
| LCD 型号 | 支持 ST7789/ST7735/ST7796/ILI9341/GC9A01 多款屏幕（通过 menuconfig 选择） |
| DHT11 | GPIO14（单总线） |
| 光照传感器 | GPIO3（ADC1_CH2） |
| PCF8574 (I2C) | 地址 0x20，P0=水泵 P1=补光灯 P2=加热片 P4=土壤DO |
| ADS1115 (I2C ADC) | 地址 0x48，16-bit ADC，土壤湿度模拟量采集，土壤湿度传感器使用了A0，还有A1、A2、A3可供使用 |
| OneNET | 产品 ID、设备名、密钥、MQTT Broker |

### 2. `compact_wifi_board_s3cam.cc` — 主板初始化和显示管理

最核心的板级文件（~500 行），包含：

- **`CompactWifiBoardS3Cam` 类**：继承 `WifiBoard`，初始化所有硬件：
  - `InitializeSpi()`：SPI3 总线初始化（LCD）
  - `InitializeLcdDisplay()`：创建 LCD 面板（ST7789/ILI9341/GC9A01）
  - `InitializeCamera()`：配置 OV2640 摄像头（VGA，RGB565）
  - `InitializePlantMonitor()`：启动植物监控系统
  - `InitializeOnenet()`：连接 OneNET 云平台
  - `InitializeButtons()`：启动按钮（单击切换聊天状态，长按配网）

- **`PlantDisplay` 类**：继承 `SpiLcdDisplay`，自定义植物监控 UI：
  - `SetupUI()`：隐藏表情区域，创建传感器面板
  - `CreateSensorPanel()`：创建 LVGL 标签（温度、湿度、光照、土壤、阈值、继电器）
  - `UpdateSensorDisplay()`：每 3 秒刷新显示数据，土壤湿度按比例颜色编码（红<20%<绿<60%<蓝）

- **`DisplayUpdateCallback()`**：定时器回调，每 3 秒获取传感器数据 → 推送到 LVGL 主线程更新屏幕；WiFi 就绪后自动启动 Web 服务器

- **OneNET 命令处理**：解析云平台下发的 JSON 命令（阈值设置、继电器控制、数据刷新）

### 3. `plant_monitor.h` — 植物监控系统头文件

定义了核心数据结构：

- **`SensorThresholds`**：传感器阈值（温度 18-32℃、湿度 40-85%、光照 2000-3000、土壤干燥 20%）
- **`SensorData`**：传感器实时数据（温度、湿度、光照 ADC、土壤模拟量百分比+原始值+数字量、三个继电器状态）

`PlantMonitor` 类是单例模式（`GetInstance()`），提供：

| 方法 | 功能 |
|------|------|
| `Initialize()` | 初始化所有传感器和外设 |
| `Update()` | 周期性读取传感器并执行自动控制 |
| `GetSensorData()` | 获取最新传感器读数 |
| `GetThresholds()` / `SetThresholds()` | 获取/设置控制阈值 |
| `SetRelay()` | 手动控制继电器 |
| 私有方法 | DHT11 驱动、PCF8574 位模拟 I2C 驱动、ADS1115 ADC 驱动、自动控制逻辑、MCP 工具注册 |

### 4. `plant_monitor.cc` — 植物监控系统实现

最核心的逻辑文件（~790 行），包含：

**传感器驱动：**
- **DHT11 驱动**（~80 行）：完全匹配普中科技 ESP32-S3 参考实现，开漏输出+内部上拉单总线模式，带 3 次重试
- **PCF8574 I2C 驱动**（~130 行）：GPIO4/GPIO5 位模拟 I2C 主控（与摄像头 SCCB 共享引脚），包括 `I2cStart`/`I2cStop`/`I2cWriteByte`/`I2cReadByte`
- **ADS1115 I2C ADC 驱动**（~55 行）：16-bit 模数转换，与 PCF8574 共用 I2C 总线（地址 0x48），连续转换模式 128SPS
- **光照传感器**：ESP32-S3 内置 ADC1_CH2（12-bit，0-4095）

**业务逻辑：**
- **`AutoControl()`**：根据阈值自动控制三路继电器
  - 温度 < 下限 → 开加热片；温度 > 上限 → 关加热片
  - 光照 < 下限 → 开补光灯；光照 > 上限 → 关补光灯（支持反转型传感器）
  - 土壤湿度 < 干燥阈值 或 空气湿度 < 下限 → 开水泵
- **`ReadSensorsForInit()`**：初始化时读取一次用于 LCD 显示
- **`Update()`**：定时读取 DHT11 + 光照 + 土壤湿度 + 执行自动控制
- **`Initialize()`**：7 步初始化流程（PCF8574 → ADS1115 → ADC → DHT11 GPIO → MCP 工具 → 初始读数 → 3 秒后启动定时更新）

**MCP 工具（11 个）：**

| 工具名 | 功能 |
|--------|------|
| `plant.get_sensor_data` | 获取所有传感器数据 |
| `plant.set_temp_threshold` | 设置温度阈值 |
| `plant.set_humidity_threshold` | 设置湿度阈值 |
| `plant.set_light_threshold` | 设置光照阈值 |
| `plant.set_soil_moisture_threshold` | 设置土壤湿度浇水阈值（0-100%） |
| `plant.get_thresholds` | 获取所有阈值 |
| `plant.control_pump` | 手动控制水泵 |
| `plant.control_light` | 手动控制补光灯 |
| `plant.control_heater` | 手动控制加热片 |
| `plant.adjust_all_thresholds` | 一次性调整所有阈值 |
| `plant.analyze_environment` | 读取传感器 + 捕获摄像头，含花卉养护知识库 |

### 5. `camera_web_server.h` — Web 服务器头文件

`CameraWebServer` 类声明（~25 行），提供：
- `Start(port)` / `Stop()`：启停 HTTP 服务器
- 4 个 HTTP 端点处理器：首页、MJPEG 流、快照、传感器 API

### 6. `camera_web_server.cc` — Web 服务器实现

嵌入式 Web 服务器（~250 行），注册 4 个 HTTP 端点：

| 路由 | 功能 |
|------|------|
| `/` | 内嵌 HTML5 仪表盘页面 |
| `/stream` | MJPEG 摄像头实时视频流（multipart/x-mixed-replace） |
| `/snapshot` | JPEG 单帧快照 |
| `/api/sensors` | JSON 格式传感器数据 API |

**网页界面特性：**
- 2x2 网格卡片：温度、湿度、光照、土壤湿度
- 每项数值下方显示对应阈值
- 三个继电器状态徽章（绿色=开，灰色=关）
- 每 2 秒 AJAX 轮询自动刷新
- 深色主题 UI（`#1a1a2e` 背景）
- 向后兼容旧版固件（`soil_moisture` 字段降级检测）

### 7. `onenet_client.h` — OneNET 云客户端头文件

`OnenetClient` 类声明（~80 行），单例模式，提供：
- `Initialize()`：配置 MQTT 连接并启动定时上传（每 15 秒）
- `UploadSensorData()`：上传传感器数据到 `thing/property/post`
- `ReportDeviceStatus()`：上传设备事件到 `thing/event/post`
- `SetCommandCallback()`：注册云平台下发命令的处理回调
- 自动订阅 `thing/property/set` 主题

### 8. `onenet_client.cc` — OneNET 云客户端实现

OneNET MQTT 客户端实现（~250 行），包含：
- **MQTT 连接管理**：TCP 连接、自动重连
- **数据上传**：JSON 格式传感器数据（温度、湿度、光照、土壤百分比+原始值）
- **命令处理**：解析云平台下发的 JSON 命令（阈值设置、继电器控制），自动回复 `set_reply`
- **事件上报**：继电器状态变更事件
- 定时器每 15 秒触发一次上传

### 9. `README.md` — 编译指南

简要的编译配置说明（`idf.py set-target esp32s3` → `menuconfig` 选板 → `build flash`）。

---

## 数据流架构

```
传感器硬件                          云平台/用户界面
─────────                          ────────────────
DHT11 (GPIO14) ──┐
                  │
光照 (GPIO3 ADC) ─┤
                  ├── PlantMonitor::Update() ──┬── LCD (SPI, 3秒刷新)
土壤 (ADS1115) ──┤                            ├── Web UI (HTTP, 2秒轮询)
                  │                            ├── OneNET MQTT (15秒上传)
土壤 (PCF8574) ──┤                            └── MCP Tools (AI语音交互)
                  │
PCF8574 GPIO ────┴── 水泵/补光灯/加热片 继电器
```

## 引脚占用总览

| GPIO | 功能 | GPIO | 功能 |
|------|------|------|------|
| 0 | 启动按钮 | 19 | LCD SPI CLK |
| 1 | I2S 麦克风 WS | 20 | LCD SPI MOSI |
| 2 | I2S 麦克风 SCK | 21 | LCD RST |
| 3 | 光照传感器 ADC | 38 | LCD 背光 PWM |
| 4 | I2C SDA (摄像头+PCF8574+ADS1115) | 39 | I2S 扬声器 DOUT |
| 5 | I2C SCL (摄像头+PCF8574+ADS1115) | 40 | I2S 扬声器 BCLK |
| 6 | 摄像头 VSYNC | 41 | I2S 扬声器 LRCK |
| 7 | 摄像头 HREF | 42 | I2S 麦克风 DIN |
| 8-18 | 摄像头 D0-D7+XCLK+PCLK | 45 | LCD CS |
| 14 | DHT11 数据 | 47 | LCD DC |
| 15 | 摄像头 XCLK | 48 | 内置 LED |

---

## I2C 总线拓扑

```
GPIO4 (SDA) ──┬── PCF8574 (0x20) ──┬── P0: 水泵继电器
              │                    ├── P1: 补光灯继电器
              │                    ├── P2: 加热片继电器
              │                    ├── P3: 空闲
              │                    ├── P4: 土壤湿度 DO (数字)
              │                    ├── P5-P7: 空闲
              │
              └── ADS1115 (0x48) ──┬── A0: 土壤湿度 AO (模拟)
                                   ├── A1-A3: 空闲

GPIO5 (SCL) ──┴── (共用时钟线)
```
