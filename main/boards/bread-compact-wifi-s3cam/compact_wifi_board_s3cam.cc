/**
 * @file    compact_wifi_board_s3cam.cc
 * @brief   ESP32-S3CAM 植物监控板 — 主板初始化、LCD显示、按钮、摄像头、OneNET 命令处理
 * @author  JJD-YOURFATHER
 *
 * 本文件是硬件板级入口, 负责:
 *   - PlantDisplay: 植物监控专用 LVGL 显示界面 (传感器面板 + 阈值 + AUTO指示)
 *   - CompactWifiBoardS3Cam: 板级初始化 (SPI/LCD/摄像头/按键/背光/PlantMonitor/OneNET)
 *   - 3秒周期显示更新定时器 + 自动启动 Web 服务器
 *   - OneNET 云平台 JSON 命令解析 (阈值设置/继电器控制/数据刷新)
 *
 * 初始化顺序 (构造函数):
 *   SPI → LCD → 按钮 → 摄像头 → 背光 → PlantMonitor → OneNET → 显示定时器
 */
#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/single_led.h"
#include "esp32_camera.h"
#include "plant_monitor.h"
#include "onenet_client.h"
#include "camera_web_server.h"
#include <wifi_manager.h>

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <cJSON.h>
#include <lvgl.h>
#include <cstdio>

// 条件编译: 根据 menuconfig 选择的 LCD 型号引入对应驱动
#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
/** GC9A01 圆形屏驱动初始化命令序列 (寄存器+数据+延时) */
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0, (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                       0x04, 0x12, 0x14, 0x1f}, 14, 0},
    {0xf1, (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                       0x0C, 0x1A, 0x14, 0x1E}, 14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif

#define TAG "CompactWifiBoardS3Cam"

/* ================================================================
 * PlantDisplay — 植物监控专用 LVGL 显示界面
 * ----------------------------------------------------------------
 * 继承 SpiLcdDisplay, 覆盖:
 *   SetEmotion()  → 空操作 (植物模式不需要表情)
 *   SetupUI()     → 隐藏表情框, 创建传感器数据面板
 *
 * LCD 布局 (220x215 像素, 深色半透明背景):
 *   ┌────────────── Plant Monitor ────── [AUTO] ┐
 *   │  温度: XX.X C        湿度: XX %          │
 *   │  ────────────────────────────────────     │
 *   │  光照: XXXX          土壤: XX%            │
 *   │  ────────────────────────────────────     │
 *   │  Thresholds:                              │
 *   │  T:XX-XX  H:XX-XX  L:XX-XX  S:XX%        │
 *   │  PUMP   LIGHT  HEAT                       │
 *   └───────────────────────────────────────────┘
 *
 * 颜色编码:
 *   温度: 青色  湿度: 蓝色  光照: 黄色
 *   土壤: 红(<20%) / 绿(20-60%) / 蓝(>60%)
 *   继电器: 绿(ON) / 灰(OFF)
 * ================================================================ */
class PlantDisplay : public SpiLcdDisplay {
public:
    PlantDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                 int width, int height, int offset_x, int offset_y,
                 bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {}

    /** 覆盖: 植物监控模式不需要表情, 空操作 */
    void SetEmotion(const char* emotion) override {}

    /** 设置自动AI模式指示器 (AUTO 徽章, 位于面板右上角) */
    void SetAutoAiIndicator(bool on) {
        if (label_auto_ai_) {
            lv_label_set_text(label_auto_ai_, on ? "AUTO" : "");
        }
    }

    /** 覆盖: 初始化 LVGL UI → 隐藏表情框 → 创建传感器面板 */
    void SetupUI() override {
        SpiLcdDisplay::SetupUI();
        // 隐藏表情区域, 用传感器数据显示替代
        if (emoji_box_) lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        CreateSensorPanel();
    }

    /**
     * 更新传感器显示 (由 3秒显示定时器回调调用)
     * @param data       传感器实时数据快照
     * @param thresholds 当前控制阈值
     */
    void UpdateSensorDisplay(const SensorData& data, const SensorThresholds& thresholds) {
        if (!sensor_panel_) return;
        DisplayLockGuard lock(this);    // LVGL 线程安全锁
        char buf[32];

        // 温度 + 湿度
        snprintf(buf, sizeof(buf), "%.1f C", data.temperature);
        lv_label_set_text(label_temp_, buf);
        snprintf(buf, sizeof(buf), "%.0f %%", data.humidity);
        lv_label_set_text(label_humi_, buf);

        // 光照 + 土壤
        snprintf(buf, sizeof(buf), "%d", data.light_value);
        lv_label_set_text(label_light_, buf);
        snprintf(buf, sizeof(buf), "%d%%", data.soil_moisture_percent);
        lv_label_set_text(label_soil_, buf);

        // 土壤颜色编码: 红(<20%干燥) / 绿(20-60%正常) / 蓝(>60%湿润)
        lv_color_t soil_color;
        if (data.soil_moisture_percent < 20) {
            soil_color = lv_color_hex(0xFF4444);
        } else if (data.soil_moisture_percent < 60) {
            soil_color = lv_color_hex(0x88FF88);
        } else {
            soil_color = lv_color_hex(0x4488FF);
        }
        lv_obj_set_style_text_color(label_soil_, soil_color, 0);

        // 阈值显示
        snprintf(buf, sizeof(buf), "T:%d-%d", thresholds.temp_min, thresholds.temp_max);
        lv_label_set_text(label_temp_threshold_, buf);
        snprintf(buf, sizeof(buf), "H:%d-%d", thresholds.humidity_min, thresholds.humidity_max);
        lv_label_set_text(label_humi_threshold_, buf);
        snprintf(buf, sizeof(buf), "L:%d-%d", thresholds.light_min, thresholds.light_max);
        lv_label_set_text(label_light_threshold_, buf);
        snprintf(buf, sizeof(buf), "S:%d%%", thresholds.soil_moisture_dry);
        lv_label_set_text(label_soil_threshold_, buf);

        // 继电器状态: 绿(ON) / 灰(OFF)
        lv_obj_set_style_text_color(label_pump_,  lv_color_hex(data.relay_pump   ? 0x00FF00 : 0x808080), 0);
        lv_obj_set_style_text_color(label_lamp_,  lv_color_hex(data.relay_light  ? 0xFFFF00 : 0x808080), 0);
        lv_obj_set_style_text_color(label_heat_,  lv_color_hex(data.relay_heater ? 0xFF4400 : 0x808080), 0);
    }

private:
    // ---- LVGL 控件指针 ----
    lv_obj_t* sensor_panel_ = nullptr;
    lv_obj_t* label_temp_ = nullptr;
    lv_obj_t* label_humi_ = nullptr;
    lv_obj_t* label_light_ = nullptr;
    lv_obj_t* label_soil_ = nullptr;
    lv_obj_t* label_pump_ = nullptr;
    lv_obj_t* label_lamp_ = nullptr;
    lv_obj_t* label_heat_ = nullptr;
    lv_obj_t* label_temp_threshold_ = nullptr;
    lv_obj_t* label_humi_threshold_ = nullptr;
    lv_obj_t* label_light_threshold_ = nullptr;
    lv_obj_t* label_soil_threshold_ = nullptr;
    lv_obj_t* label_auto_ai_ = nullptr;

    /**
     * 创建传感器面板 — 构建完整的 LVGL 控件树
     * 使用 lv_obj_align() 绝对定位布局
     * lv_obj_set_style_text_color() 设置各控件颜色编码
     */
    void CreateSensorPanel() {
        auto* screen = lv_screen_active();       // 获取当前 LVGL 屏幕
        const lv_font_t* font = LV_FONT_DEFAULT;

        // 面板容器: 220x215, 30% 不透明度黑色背景, 圆角8px, 居中
        sensor_panel_ = lv_obj_create(screen);
        lv_obj_set_size(sensor_panel_, 220, 215);
        lv_obj_set_style_bg_opa(sensor_panel_, LV_OPA_30, 0);
        lv_obj_set_style_bg_color(sensor_panel_, lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_width(sensor_panel_, 0, 0);
        lv_obj_set_style_pad_all(sensor_panel_, 0, 0);
        lv_obj_set_style_radius(sensor_panel_, 8, 0);
        lv_obj_align(sensor_panel_, LV_ALIGN_CENTER, 0, 0);

        // 标题: "Plant Monitor" (居中, 顶部)
        lv_obj_t* title = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(title, font, 0);
        lv_obj_set_style_text_color(title, lv_color_hex(0xAAAAAA), 0);
        lv_label_set_text(title, "Plant Monitor");
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

        // AUTO AI 指示器 (右上角, 绿色, 默认隐藏)
        label_auto_ai_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_auto_ai_, font, 0);
        lv_obj_set_style_text_color(label_auto_ai_, lv_color_hex(0x00FF44), 0);
        lv_label_set_text(label_auto_ai_, "");
        lv_obj_align(label_auto_ai_, LV_ALIGN_TOP_RIGHT, -4, 4);

        // 温度标签 (左上)
        label_temp_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_temp_, font, 0);
        lv_obj_set_style_text_color(label_temp_, lv_color_hex(0x00FFAA), 0);  // 青色
        lv_label_set_text(label_temp_, "0.0 C");
        lv_obj_align(label_temp_, LV_ALIGN_TOP_LEFT, 8, 26);

        // 湿度标签 (右上)
        label_humi_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_humi_, font, 0);
        lv_obj_set_style_text_color(label_humi_, lv_color_hex(0x00AAFF), 0);  // 蓝色
        lv_label_set_text(label_humi_, "0 %");
        lv_obj_align(label_humi_, LV_ALIGN_TOP_LEFT, 120, 26);

        // 分隔线1
        lv_obj_t* line = lv_obj_create(sensor_panel_);
        lv_obj_set_size(line, 200, 1);
        lv_obj_set_style_bg_color(line, lv_color_hex(0x444444), 0);
        lv_obj_set_style_border_width(line, 0, 0);
        lv_obj_align(line, LV_ALIGN_TOP_MID, 0, 54);

        // 光照标签 (左下)
        label_light_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_light_, font, 0);
        lv_obj_set_style_text_color(label_light_, lv_color_hex(0xFFFF88), 0); // 黄色
        lv_label_set_text(label_light_, "0");
        lv_obj_align(label_light_, LV_ALIGN_TOP_LEFT, 8, 62);

        // 土壤标签 (右下)
        label_soil_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_soil_, font, 0);
        lv_obj_set_style_text_color(label_soil_, lv_color_hex(0x88FF88), 0);  // 绿
        lv_label_set_text(label_soil_, "0%");
        lv_obj_align(label_soil_, LV_ALIGN_TOP_LEFT, 120, 62);

        // 分隔线2
        lv_obj_t* line2 = lv_obj_create(sensor_panel_);
        lv_obj_set_size(line2, 200, 1);
        lv_obj_set_style_bg_color(line2, lv_color_hex(0x444444), 0);
        lv_obj_set_style_border_width(line2, 0, 0);
        lv_obj_align(line2, LV_ALIGN_TOP_MID, 0, 90);

        // 阈值标题
        lv_obj_t* thresh_title = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(thresh_title, font, 0);
        lv_obj_set_style_text_color(thresh_title, lv_color_hex(0x888888), 0);
        lv_label_set_text(thresh_title, "Thresholds:");
        lv_obj_align(thresh_title, LV_ALIGN_TOP_LEFT, 8, 96);

        // 温度/湿度/光照/土壤 阈值标签 (两列布局)
        label_temp_threshold_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_temp_threshold_, font, 0);
        lv_obj_set_style_text_color(label_temp_threshold_, lv_color_hex(0xCC8888), 0);
        lv_label_set_text(label_temp_threshold_, "T:18-32");
        lv_obj_align(label_temp_threshold_, LV_ALIGN_TOP_LEFT, 8, 114);

        label_humi_threshold_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_humi_threshold_, font, 0);
        lv_obj_set_style_text_color(label_humi_threshold_, lv_color_hex(0x8888CC), 0);
        lv_label_set_text(label_humi_threshold_, "H:40-85");
        lv_obj_align(label_humi_threshold_, LV_ALIGN_TOP_LEFT, 120, 114);

        label_light_threshold_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_light_threshold_, font, 0);
        lv_obj_set_style_text_color(label_light_threshold_, lv_color_hex(0xCCCC88), 0);
        lv_label_set_text(label_light_threshold_, "L:2000-3000");
        lv_obj_align(label_light_threshold_, LV_ALIGN_TOP_LEFT, 8, 132);

        label_soil_threshold_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_soil_threshold_, font, 0);
        lv_obj_set_style_text_color(label_soil_threshold_, lv_color_hex(0x88CC88), 0);
        lv_label_set_text(label_soil_threshold_, "S:20%");
        lv_obj_align(label_soil_threshold_, LV_ALIGN_TOP_LEFT, 120, 132);

        // 继电器状态标签 (底部平铺)
        label_pump_ = CreateRelayLabel(sensor_panel_, font, "PUMP",  8, 155);
        label_lamp_ = CreateRelayLabel(sensor_panel_, font, "LIGHT", 8, 178);
        label_heat_ = CreateRelayLabel(sensor_panel_, font, "HEAT",  8, 201);
    }

    /** 创建继电器状态标签 (默认灰色) */
    lv_obj_t* CreateRelayLabel(lv_obj_t* parent, const lv_font_t* font,
                                const char* name, int x, int y) {
        lv_obj_t* label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, font, 0);
        lv_obj_set_style_text_color(label, lv_color_hex(0x808080), 0);  // 灰色 = OFF
        lv_label_set_text(label, name);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y);
        return label;
    }
};

/* ================================================================
 * CompactWifiBoardS3Cam — 主板类
 * ----------------------------------------------------------------
 * 继承 WifiBoard, 实现所有硬件初始化
 * 通过 DECLARE_BOARD 宏注册到工厂系统
 * ================================================================ */
class CompactWifiBoardS3Cam : public WifiBoard {
private:
    Button boot_button_;                         // BOOT 按键 (单击=对话, 长按=自动AI)
    PlantDisplay* display_;                      // 植物监控 LCD 显示
    Esp32Camera* camera_;                        // OV2640 摄像头
    esp_timer_handle_t display_update_timer_ = nullptr; // 3秒周期显示刷新
    CameraWebServer* web_server_ = nullptr;      // 内嵌 Web 服务器
    bool web_server_started_ = false;            // Web 服务器启动标志

    /**
     * SPI 总线初始化 — LCD 通过 SPI3_HOST 连接
     * MOSI=GPIO20, CLK=GPIO19, 无 MISO, DMA 自动分配
     */
    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;   // GPIO20
        buscfg.miso_io_num = GPIO_NUM_NC;        // 不使用 MISO
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;    // GPIO19
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    /**
     * LCD 显示初始化
     * 算法: SPI Panel IO 创建 → LCD 驱动创建 (ST7789/ILI9341/GC9A01)
     * → 硬件复位 → 面板初始化 → 颜色反转/轴交换/镜像 → 创建 PlantDisplay
     */
    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 创建 SPI Panel IO (CS=GPIO45, DC=GPIO47, 40MHz, SPI模式由菜单配置)
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;   // 40 MHz
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 创建 LCD 面板驱动 (根据菜单配置选择驱动类型)
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;        // RGB565
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel)); // 默认 ST7789
#endif
        // 面板初始化序列: 复位 → 初始化寄存器 → 颜色反转 → 轴交换 → 镜像
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif

        // 创建 PlantDisplay (LVGL 集成的传感器数据显示面板)
        display_ = new PlantDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                    DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                    DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    /**
     * 摄像头 OV2640 初始化
     * 8-bit DVP 并行接口, VGA (640x480) RGB565, JPEG 质量12, PSRAM帧缓冲
     * SCCB 使用 I2C_NUM_0 (GPIO4/5, 与 PCF8574/ADS1115 共享)
     */
    void InitializeCamera() {
        camera_config_t config = {};
        config.pin_d0 = CAMERA_PIN_D0;
        config.pin_d1 = CAMERA_PIN_D1;
        config.pin_d2 = CAMERA_PIN_D2;
        config.pin_d3 = CAMERA_PIN_D3;
        config.pin_d4 = CAMERA_PIN_D4;
        config.pin_d5 = CAMERA_PIN_D5;
        config.pin_d6 = CAMERA_PIN_D6;
        config.pin_d7 = CAMERA_PIN_D7;
        config.pin_xclk = CAMERA_PIN_XCLK;
        config.pin_pclk = CAMERA_PIN_PCLK;
        config.pin_vsync = CAMERA_PIN_VSYNC;
        config.pin_href = CAMERA_PIN_HREF;
        config.pin_sccb_sda = CAMERA_PIN_SIOD;      // GPIO4
        config.pin_sccb_scl = CAMERA_PIN_SIOC;      // GPIO5
        config.sccb_i2c_port = 0;                    // I2C_NUM_0
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;          // 20 MHz
        config.pixel_format = PIXFORMAT_RGB565;      // 16-bit 彩色
        config.frame_size = FRAMESIZE_VGA;           // 640x480
        config.jpeg_quality = 12;                    // 高质量 JPEG
        config.fb_count = 1;                         // 1个帧缓冲
        config.fb_location = CAMERA_FB_IN_PSRAM;     // 帧缓冲在 PSRAM
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;   // 空闲时抓取
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(false);                  // 禁用水平镜像
    }

    void InitializePlantMonitor() {
        GetPlantMonitor().Initialize();
    }

    /**
     * OneNET 云平台初始化 + 命令回调注册
     *
     * 命令格式 (JSON):
     *   设置阈值: {"temp_min":18,"temp_max":32,"humidity_min":40,...}
     *   控制继电器: {"relay":0,"state":true}
     *   刷新数据: {"refresh":true}
     */
    void InitializeOnenet() {
        auto& onenet = GetOnenetClient();
        onenet.SetCommandCallback([](const std::string& cmd_json) -> std::string {
            auto& monitor = GetPlantMonitor();
            cJSON* root = cJSON_Parse(cmd_json.c_str()); // 解析 JSON
            if (!root) return "{\"error\":\"invalid json\"}";

            // 提取各阈值字段 (可为 null)
            cJSON* temp_min = cJSON_GetObjectItem(root, "temp_min");
            cJSON* temp_max = cJSON_GetObjectItem(root, "temp_max");
            cJSON* humi_min = cJSON_GetObjectItem(root, "humidity_min");
            cJSON* humi_max = cJSON_GetObjectItem(root, "humidity_max");
            cJSON* light_min = cJSON_GetObjectItem(root, "light_min");
            cJSON* light_max = cJSON_GetObjectItem(root, "light_max");
            cJSON* soil_dry = cJSON_GetObjectItem(root, "soil_moisture_dry");

            // 如果提供了温湿光三对阈值 → 设置阈值
            if (temp_min && temp_max && humi_min && humi_max && light_min && light_max) {
                SensorThresholds t;
                t.temp_min = temp_min->valueint;
                t.temp_max = temp_max->valueint;
                t.humidity_min = humi_min->valueint;
                t.humidity_max = humi_max->valueint;
                t.light_min = light_min->valueint;
                t.light_max = light_max->valueint;
                if (soil_dry) t.soil_moisture_dry = soil_dry->valueint; // 土壤阈值可选
                monitor.SetThresholds(t);
                cJSON_Delete(root);
                return "{\"success\":true,\"message\":\"阈值已更新\"}";
            }

            // 继电器控制: relay=索引, state=开关
            cJSON* relay = cJSON_GetObjectItem(root, "relay");
            cJSON* state = cJSON_GetObjectItem(root, "state");
            if (relay && state) {
                monitor.SetRelay(relay->valueint, cJSON_IsTrue(state));
                cJSON_Delete(root);
                return "{\"success\":true,\"message\":\"继电器已控制\"}";
            }

            // 数据刷新请求
            cJSON* refresh = cJSON_GetObjectItem(root, "refresh");
            if (refresh) {
                monitor.Update();
                auto data = monitor.GetSensorData();
                char buf[320];
                snprintf(buf, sizeof(buf),
                    "{\"temperature\":%.1f,\"humidity\":%.1f,\"light\":%d,"
                    "\"soil_percent\":%d,\"soil_raw\":%d,"
                    "\"pump\":%s,\"light_relay\":%s,\"heater\":%s}",
                    data.temperature, data.humidity, data.light_value,
                    data.soil_moisture_percent, data.soil_moisture_raw,
                    data.relay_pump ? "true" : "false",
                    data.relay_light ? "true" : "false",
                    data.relay_heater ? "true" : "false");
                cJSON_Delete(root);
                return std::string(buf);
            }

            cJSON_Delete(root);
            return "{\"error\":\"unknown command\"}";
        });
        onenet.Initialize();
    }

    /**
     * 显示更新回调 — 3秒周期定时器
     * 1. WiFi 就绪后自动启动摄像头 Web 服务器
     * 2. 读取传感器数据 + 自动AI状态 → 推送到主线程更新 LCD
     */
    static void DisplayUpdateCallback(void* arg) {
        auto* self = static_cast<CompactWifiBoardS3Cam*>(arg);
        auto& app = Application::GetInstance();
        auto data = GetPlantMonitor().GetSensorData();
        auto thresholds = GetPlantMonitor().GetThresholds();

        // 懒启动 Web 服务器 (WiFi 连接后首次触发)
        if (!self->web_server_started_) {
            auto& wifi = WifiManager::GetInstance();
            if (wifi.IsConnected() && !wifi.GetIpAddress().empty()) {
                self->web_server_ = new CameraWebServer();
                if (self->web_server_->Start(80)) {
                    self->web_server_started_ = true;
                    ESP_LOGI(TAG, "摄像头Web服务器已启动 http://%s", wifi.GetIpAddress().c_str());
                }
            }
        }

        bool auto_ai = GetPlantMonitor().IsAutoAiMode();

        // 通过 Schedule 投递到主线程 (LVGL 必须在主线程操作)
        app.Schedule([self, data, thresholds, auto_ai]() {
            if (self->display_) {
                self->display_->UpdateSensorDisplay(data, thresholds);
                self->display_->SetAutoAiIndicator(auto_ai);
            }
        });
    }

    /**
     * 按钮初始化
     * 单击 (OnClick):      启动/停止小智对话, 或进入配网模式
     * 长按 (OnLongPress):  切换自动 AI 模式
     */
    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();               // 启动阶段 → 配网
                return;
            }
            app.ToggleChatState();                   // 正常状态 → 对话开关
        });

        boot_button_.OnLongPress([this]() {
            auto& monitor = GetPlantMonitor();
            if (monitor.IsAutoAiMode()) {
                monitor.StopAutoAiMode();            // 已开启 → 关闭
            } else {
                monitor.StartAutoAiMode();           // 未开启 → 启动
            }
        });
    }

public:
    /**
     * 构造函数 — 执行完整的硬件初始化序列
     *
     * 顺序:
     *   SPI → LCD → 按钮 → 摄像头 → 背光 → PlantMonitor → OneNET → 显示定时器
     */
    CompactWifiBoardS3Cam() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();                             // 1. SPI3 总线
        InitializeLcdDisplay();                      // 2. LCD + PlantDisplay
        InitializeButtons();                         // 3. 按键回调
        InitializeCamera();                          // 4. OV2640 摄像头
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();     // 5. 恢复背光亮度
        }
        InitializePlantMonitor();                    // 6. 传感器 + 继电器 + MCP
        InitializeOnenet();                          // 7. OneNET 云平台

        // 8. 启动 3秒周期显示刷新定时器
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = DisplayUpdateCallback;
        timer_args.arg = this;
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.name = "display_update";
        timer_args.skip_unhandled_events = false;
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &display_update_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(display_update_timer_, 3000000)); // 3,000,000 μs = 3s
    }

    ~CompactWifiBoardS3Cam() {
        if (display_update_timer_) {
            esp_timer_stop(display_update_timer_);
            esp_timer_delete(display_update_timer_);
        }
        if (web_server_) {
            web_server_->Stop();
            delete web_server_;
        }
    }

    /** 板载 LED: 单色 LED, GPIO48, 低电平点亮 */
    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    /**
     * 音频编解码器: NoAudioCodecSimplex
     * 无物理音频编解码芯片, I2S 直接驱动数字功放+模拟麦克风
     * Simplex 模式: 麦克风和扬声器各自独立 I2S 通道
     */
    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    /** PWM 背光控制, GPIO38, 25000Hz */
    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }
};

/** 注册板级到工厂系统: Board::GetInstance() 通过此宏创建实例 */
DECLARE_BOARD(CompactWifiBoardS3Cam);
