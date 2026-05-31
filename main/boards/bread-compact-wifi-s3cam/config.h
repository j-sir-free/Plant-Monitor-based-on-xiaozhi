/**
 * @file    config.h
 * @brief   面包板ESP32-S3CAM 植物监控系统 — 硬件引脚与参数配置
 * @author  JJD-YOURFATHER
 *
 * 本文件集中定义所有硬件引脚映射、传感器参数、通信地址。
 * 通过 #ifdef CONFIG_LCD_* 宏支持多种LCD型号的选择 (在 menuconfig 中切换)。
 * 通过 #ifdef RELAY_ACTIVE_HIGH / LIGHT_SENSOR_INVERTED 适配不同硬件接线。
 */
#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

/* ================================================================
 * 音频 I2S 配置
 * ----------------------------------------------------------------
 * 使用 Simplex 模式: 麦克风和扬声器各自独立的I2S通道
 * 采样率: 输入16kHz (语音识别), 输出24kHz (TTS播放)
 * ================================================================ */
#define AUDIO_INPUT_SAMPLE_RATE  16000     // 麦克风采样率 (Hz)
#define AUDIO_OUTPUT_SAMPLE_RATE 24000     // 扬声器采样率 (Hz)

// 如果使用 Duplex I2S 模式 (麦克风和扬声器共用I2S), 请注释下面一行
#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX
/* ---- Simplex 模式: 独立 I2S 通道 ---- */
#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_1     // 麦克风: 字选信号 (Word Select)
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_2     // 麦克风: 串行时钟 (Serial Clock)
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_42    // 麦克风: 数据输入 (Data In)
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_39    // 扬声器: 数据输出 (Data Out)
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_40    // 扬声器: 位时钟 (Bit Clock)
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_41    // 扬声器: 左右通道时钟 (LR Clock)

#else
/* ---- Duplex 模式: 共用 I2S 通道 (与摄像头SCCB引脚冲突, 不推荐) ---- */
#define AUDIO_I2S_GPIO_WS   GPIO_NUM_4        // I2S 字选
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_5        // I2S 位时钟
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6        // I2S 数据输入
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7        // I2S 数据输出
#endif


/* ================================================================
 * 板级基本 IO
 * ----------------------------------------------------------------
 * GPIO_NC 表示该引脚未连接 (No Connection)
 * ================================================================ */
#define BUILTIN_LED_GPIO        GPIO_NUM_48    // 板载LED (低电平点亮)
#define BOOT_BUTTON_GPIO        GPIO_NUM_0     // BOOT按键 (按下=低电平, 复用为配网/对话)
#define TOUCH_BUTTON_GPIO       GPIO_NUM_NC    // 触摸按键 (未使用)
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC    // 音量+ (未使用)
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC    // 音量- (未使用)

/* ================================================================
 * 摄像头 OV2640 配置
 * ----------------------------------------------------------------
 * 使用 8-bit 并行 DVP 接口, XCLK=20MHz, 输出 VGA RGB565
 * SCCB (Serial Camera Control Bus) 与 I2C 协议兼容, 占用 GPIO4/5
 * 注意: 摄像头占用了 ESP32-S3 USB 引脚 GPIO19/20
 * ================================================================ */
#define CAMERA_PIN_D0    GPIO_NUM_11      // 数据总线 D0
#define CAMERA_PIN_D1    GPIO_NUM_9       // 数据总线 D1
#define CAMERA_PIN_D2    GPIO_NUM_8       // 数据总线 D2
#define CAMERA_PIN_D3    GPIO_NUM_10      // 数据总线 D3
#define CAMERA_PIN_D4    GPIO_NUM_12      // 数据总线 D4
#define CAMERA_PIN_D5    GPIO_NUM_18      // 数据总线 D5
#define CAMERA_PIN_D6    GPIO_NUM_17      // 数据总线 D6
#define CAMERA_PIN_D7    GPIO_NUM_16      // 数据总线 D7
#define CAMERA_PIN_XCLK  GPIO_NUM_15      // 主时钟输出 (20MHz)
#define CAMERA_PIN_PCLK  GPIO_NUM_13      // 像素时钟输入
#define CAMERA_PIN_VSYNC GPIO_NUM_6       // 帧同步信号
#define CAMERA_PIN_HREF  GPIO_NUM_7       // 行参考信号
#define CAMERA_PIN_SIOC  GPIO_NUM_5       // SCCB 时钟 (与 PCF8574/ADS1115 I2C 共用)
#define CAMERA_PIN_SIOD  GPIO_NUM_4       // SCCB 数据 (与 PCF8574/ADS1115 I2C 共用)
#define CAMERA_PIN_PWDN  GPIO_NUM_NC      // 摄像头断电控制 (未使用)
#define CAMERA_PIN_RESET GPIO_NUM_NC      // 摄像头复位 (未使用)
#define XCLK_FREQ_HZ     20000000         // 摄像头主时钟频率 (20MHz)

/* ================================================================
 * LCD 显示屏 SPI 接口引脚 (所有LCD型号共用)
 * ----------------------------------------------------------------
 * 使用 SPI3_HOST, 40MHz 时钟
 * MOSI/CLK 与 ESP32-S3 USB 引脚 GPIO19/20 冲突
 * DC: Data/Command 选择信号 (高电平=数据, 低电平=命令)
 * CS: Chip Select 片选信号
 * RST: 硬件复位引脚
 * ================================================================ */
#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_38   // LCD 背光 (PWM 调光)
#define DISPLAY_MOSI_PIN      GPIO_NUM_20   // SPI 主发从收 (Master Out Slave In)
#define DISPLAY_CLK_PIN       GPIO_NUM_19   // SPI 时钟 (Serial Clock)
#define DISPLAY_DC_PIN        GPIO_NUM_47   // SPI 数据/命令选择 (Data/Command)
#define DISPLAY_RST_PIN       GPIO_NUM_21   // LCD 硬件复位
#define DISPLAY_CS_PIN        GPIO_NUM_45   // SPI 片选 (Chip Select)

/* ================================================================
 * LCD 型号参数配置 (通过 menuconfig 选择 CONFIG_LCD_* 选项)
 * ----------------------------------------------------------------
 * 每款LCD的参数组合:
 *   DISPLAY_WIDTH/HEIGHT  — 分辨率
 *   DISPLAY_MIRROR_X/Y    — 水平/垂直镜像
 *   DISPLAY_SWAP_XY       — 交换XY轴 (横屏/竖屏切换)
 *   DISPLAY_INVERT_COLOR  — 颜色反转 (IPS屏通常关闭)
 *   DISPLAY_RGB_ORDER     — RGB/BGR 像素顺序
 *   DISPLAY_OFFSET_X/Y    — 显示区域偏移 (适配特殊面板)
 *   DISPLAY_SPI_MODE      — SPI模式 (0或3)
 *   LCD_TYPE_*_SERIAL     — 选择驱动芯片 (ST7789/ILI9341/GC9A01)
 * ================================================================ */

// --- ST7789 240x320 (标准) ---
#ifdef CONFIG_LCD_ST7789_240X320
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7789 240x320 (非IPS面板, 无颜色反转) ---
#ifdef CONFIG_LCD_ST7789_240X320_NO_IPS
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    false
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7789 170x320 ---
#ifdef CONFIG_LCD_ST7789_170X320
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   170
#define DISPLAY_HEIGHT  320
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  35
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7789 172x320 ---
#ifdef CONFIG_LCD_ST7789_172X320
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   172
#define DISPLAY_HEIGHT  320
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  34
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7789 240x280 ---
#ifdef CONFIG_LCD_ST7789_240X280
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  280
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  20
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7789 240x240 (方形屏) ---
#ifdef CONFIG_LCD_ST7789_240X240
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7789 240x240 7-Pin (SPI Mode 3) ---
#ifdef CONFIG_LCD_ST7789_240X240_7PIN
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 3      // SPI Mode 3: CPOL=1, CPHA=1
#endif

// --- ST7789 240x135 (圆角屏) ---
#ifdef CONFIG_LCD_ST7789_240X135
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  135
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY true
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  40
#define DISPLAY_OFFSET_Y  53
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7735 128x160 ---
#ifdef CONFIG_LCD_ST7735_128X160
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  160
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    false
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7735 128x128 ---
#ifdef CONFIG_LCD_ST7735_128X128
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  128
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR  false
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  32
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7796 320x480 ---
#ifdef CONFIG_LCD_ST7796_320X480
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  480
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ST7796 320x480 (非IPS) ---
#ifdef CONFIG_LCD_ST7796_320X480_NO_IPS
#define LCD_TYPE_ST7789_SERIAL
#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  480
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    false
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ILI9341 240x320 ---
#ifdef CONFIG_LCD_ILI9341_240X320
#define LCD_TYPE_ILI9341_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- ILI9341 240x320 (非IPS) ---
#ifdef CONFIG_LCD_ILI9341_240X320_NO_IPS
#define LCD_TYPE_ILI9341_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    false
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- GC9A01 240x240 (圆形屏驱动, 方形面板) ---
#ifdef CONFIG_LCD_GC9A01_240X240
#define LCD_TYPE_GC9A01_SERIAL
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  240
#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_BGR
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif

// --- 自定义 LCD ---
#ifdef CONFIG_LCD_CUSTOM
#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  320
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false
#define DISPLAY_INVERT_COLOR    true
#define DISPLAY_RGB_ORDER  LCD_RGB_ELEMENT_ORDER_RGB
#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false
#define DISPLAY_SPI_MODE 0
#endif


/* ================================================================
 * 植物生长环境监控系统 — 传感器与执行器引脚
 * ================================================================ */

// DHT11 温湿度传感器 — 单总线数字协议
// 数据格式: 5字节 (湿度整数+湿度小数+温度整数+温度小数+校验和)
// 通信时序: 主机拉低≥18ms → 拉高30μs → 等待DHT11应答 → 读取40bit
#define DHT11_DATA_PIN          GPIO_NUM_14

// 光照传感器 — 模拟输出, 接 ESP32-S3 ADC1 通道2
// ADC1 不受WiFi干扰, 12-bit 分辨率 (0-4095), 12dB 衰减 (0-3.3V量程)
#define LIGHT_SENSOR_ADC_PIN    GPIO_NUM_3

// PCF8574 — I2C 8-bit GPIO 扩展器
// 与摄像头 SCCB 共享 I2C 总线 (GPIO4=SDA, GPIO5=SCL)
// 使用位模拟 (bit-banged) I2C 而非 ESP-IDF 硬件驱动
#define PCF8574_I2C_ADDR        0x20    // PCF8574 7位I2C地址 (A0=A1=A2=GND)

// PCF8574 引脚分配 (8个GPIO):
// P0-P2: 继电器控制 (输出)
// P4:    土壤湿度数字量输入
// P3,P5,P6,P7: 空闲 (可扩展)
#define PCF8574_PIN_RELAY_PUMP     0    // P0 - 水泵继电器
#define PCF8574_PIN_RELAY_LIGHT    1    // P1 - 补光灯继电器
#define PCF8574_PIN_RELAY_HEATER   2    // P2 - 加热片继电器
#define PCF8574_PIN_SOIL_DO        4    // P4 - 土壤湿度传感器 DO (数字输入, 0=干燥, 1=潮湿)

// 继电器接线模式:
//   RELAY_ACTIVE_HIGH: COM+NC接线, 高电平→继电器不吸合→NC闭合→设备通电
//   取消注释: COM+NO接线, 低电平→继电器吸合→NO闭合→设备通电
#define RELAY_ACTIVE_HIGH

// 光照传感器模式:
//   LIGHT_SENSOR_INVERTED: 光照越强 ADC 值越低 (反转型光敏电阻)
//   取消注释: 标准型传感器, 光照越强 ADC 值越高
#define LIGHT_SENSOR_INVERTED

/* ================================================================
 * ADS1115 — I2C 16-bit ADC 模块 (土壤湿度模拟量采集)
 * ----------------------------------------------------------------
 * 与 PCF8574 共用 I2C 总线 (GPIO4=SDA, GPIO5=SCL)
 * 4通道差分/单端输入, 可编程增益放大器 (PGA)
 * 配置: AIN0 vs GND 单端, ±4.096V 量程, 连续转换, 128 SPS
 * ================================================================ */
#define ADS1115_I2C_ADDR            0x48    // 7位I2C地址 (ADDR引脚接GND)
#define ADS1115_REG_CONVERSION      0x00    // 转换结果寄存器 (16-bit, 只读)
#define ADS1115_REG_CONFIG          0x01    // 配置寄存器 (16-bit, 读写)

// ADS1115 配置寄存器 (16-bit) 的默认值:
//   Bit [15]:   OS=1 (开始转换, 在连续模式下无效但置1以触发首次转换)
//   Bit [14:12]: MUX=000 (AIN0 vs GND 单端输入)
//   Bit [11:9]:  PGA=001 (±4.096V 满量程, 1 LSB = 0.125mV)
//   Bit [8]:    MODE=0 (连续转换模式, 自动循环采样)
//   Bit [7:5]:  DR=100 (128 SPS, 每次转换约 7.8ms)
//   Bit [4]:    COMP_MODE=0 (传统比较器模式)
//   Bit [3]:    COMP_POL=0 (低电平有效)
//   Bit [2]:    COMP_LAT=0 (非锁存)
//   Bit [1:0]:  COMP_QUE=11 (禁用比较器, ALERT引脚不输出)
#define ADS1115_CONFIG_MSB          0x84
#define ADS1115_CONFIG_LSB          0x83

// 土壤湿度传感器 — 反转型 (干燥→高电压→高ADC, 浸水→低电压→低ADC)
// 线性映射: DRY_VALUE → 0%,  WET_VALUE → 100%
// 公式: percent = 100 - (raw - WET) * 100 / (DRY - WET)
// 查看串口日志中的 raw= 值, 根据实际读数标定这两个值
#define SOIL_MOISTURE_DRY_VALUE     21184   // 传感器干燥空气中的 ADC 读数
#define SOIL_MOISTURE_WET_VALUE     8000    // 传感器浸入水中的 ADC 读数

/* ================================================================
 * 自动 AI 模式
 * ----------------------------------------------------------------
 * 间隔: 每10秒检测一次传感器异常
 * 触发: 当温度/湿度/光照/土壤越过阈值时, 通过声学回环触发小智AI播报
 * 冷却: 两次播报之间至少间隔60秒
 * ================================================================ */
#define AUTO_AI_INTERVAL_MS         10000   // 传感器巡检间隔 (毫秒)

/* ================================================================
 * OneNET 云平台 MQTT 配置
 * ----------------------------------------------------------------
 * OneNET 是中国移动物联网开放平台 (v5.0 MQTT协议)
 * 设备接入流程: 创建产品→添加设备→获取设备Key→生成Token→配置参数
 * 数据上传主题: $sys/{产品ID}/{设备名}/thing/property/post
 * 命令接收主题: $sys/{产品ID}/{设备名}/thing/property/set
 * ================================================================ */
#define ONENET_PRODUCT_ID   "Kp43RWJB3j"       // 产品ID
#define ONENET_DEVICE_NAME  "esp32s3"           // 设备名称
// Token 由 OneNET 官方工具生成: https://open.iot.10086.cn/doc/v5/fuse/detail/919
#define ONENET_PASSWORD     "version=2018-10-31&res=products%2FKp43RWJB3j%2Fdevices%2Fesp32s3&et=2524579200&method=md5&sign=V%2FQ7odRsYd8uU2ikvi3o8w%3D%3D"

#define ONENET_MQTT_BROKER  "mqtts.heclouds.com"   // OneNET MQTT 服务器
#define ONENET_MQTT_PORT    1883                    // MQTT 端口 (非TLS)

#endif // _BOARD_CONFIG_H_
