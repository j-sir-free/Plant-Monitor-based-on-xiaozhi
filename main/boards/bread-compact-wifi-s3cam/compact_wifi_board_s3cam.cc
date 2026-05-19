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

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>
#include <cJSON.h>
#include <lvgl.h>
#include <cstdio>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
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
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif

#define TAG "CompactWifiBoardS3Cam"

// 植物监控专用显示类：去掉表情，显示传感器数据
class PlantDisplay : public SpiLcdDisplay {
public:
    PlantDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                 int width, int height, int offset_x, int offset_y,
                 bool mirror_x, bool mirror_y, bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {}

    void SetEmotion(const char* emotion) override {
        // 去掉表情显示，改为显示传感器数据
    }

    void SetupUI() override {
        SpiLcdDisplay::SetupUI();
        // 隐藏表情区域，改为传感器数据显示
        if (emoji_box_) lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        CreateSensorPanel();
    }

    void UpdateSensorDisplay(const SensorData& data) {
        if (!sensor_panel_) return;
        DisplayLockGuard lock(this);
        char buf[32];

        snprintf(buf, sizeof(buf), "%.1f C", data.temperature);
        lv_label_set_text(label_temp_, buf);

        snprintf(buf, sizeof(buf), "%.0f %%", data.humidity);
        lv_label_set_text(label_humi_, buf);

        snprintf(buf, sizeof(buf), "%d", data.light_value);
        lv_label_set_text(label_light_, buf);

        snprintf(buf, sizeof(buf), "%s", data.soil_moisture ? "OK" : "DRY");
        lv_label_set_text(label_soil_, buf);

        lv_obj_set_style_text_color(label_pump_,  lv_color_hex(data.relay_pump   ? 0x00FF00 : 0x808080), 0);
        lv_obj_set_style_text_color(label_lamp_,  lv_color_hex(data.relay_light  ? 0xFFFF00 : 0x808080), 0);
        lv_obj_set_style_text_color(label_heat_,  lv_color_hex(data.relay_heater ? 0xFF4400 : 0x808080), 0);
    }

private:
    lv_obj_t* sensor_panel_ = nullptr;
    lv_obj_t* label_temp_ = nullptr;
    lv_obj_t* label_humi_ = nullptr;
    lv_obj_t* label_light_ = nullptr;
    lv_obj_t* label_soil_ = nullptr;
    lv_obj_t* label_pump_ = nullptr;
    lv_obj_t* label_lamp_ = nullptr;
    lv_obj_t* label_heat_ = nullptr;

    void CreateSensorPanel() {
        auto* screen = lv_screen_active();
        const lv_font_t* font = LV_FONT_DEFAULT;

        sensor_panel_ = lv_obj_create(screen);
        lv_obj_set_size(sensor_panel_, 220, 170);
        lv_obj_set_style_bg_opa(sensor_panel_, LV_OPA_30, 0);
        lv_obj_set_style_bg_color(sensor_panel_, lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_width(sensor_panel_, 0, 0);
        lv_obj_set_style_pad_all(sensor_panel_, 0, 0);
        lv_obj_set_style_radius(sensor_panel_, 8, 0);
        lv_obj_align(sensor_panel_, LV_ALIGN_CENTER, 0, 0);

        // 标题
        lv_obj_t* title = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(title, font, 0);
        lv_obj_set_style_text_color(title, lv_color_hex(0xAAAAAA), 0);
        lv_label_set_text(title, "Plant Monitor");
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 4);

        // 温度
        label_temp_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_temp_, font, 0);
        lv_obj_set_style_text_color(label_temp_, lv_color_hex(0x00FFAA), 0);
        lv_label_set_text(label_temp_, "0.0 C");
        lv_obj_align(label_temp_, LV_ALIGN_TOP_LEFT, 8, 26);

        // 湿度
        label_humi_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_humi_, font, 0);
        lv_obj_set_style_text_color(label_humi_, lv_color_hex(0x00AAFF), 0);
        lv_label_set_text(label_humi_, "0 %");
        lv_obj_align(label_humi_, LV_ALIGN_TOP_LEFT, 120, 26);

        // 分隔线
        lv_obj_t* line = lv_obj_create(sensor_panel_);
        lv_obj_set_size(line, 200, 1);
        lv_obj_set_style_bg_color(line, lv_color_hex(0x444444), 0);
        lv_obj_set_style_border_width(line, 0, 0);
        lv_obj_align(line, LV_ALIGN_TOP_MID, 0, 54);

        // 光照
        label_light_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_light_, font, 0);
        lv_obj_set_style_text_color(label_light_, lv_color_hex(0xFFFF88), 0);
        lv_label_set_text(label_light_, "0");
        lv_obj_align(label_light_, LV_ALIGN_TOP_LEFT, 8, 62);

        // 土壤
        label_soil_ = lv_label_create(sensor_panel_);
        lv_obj_set_style_text_font(label_soil_, font, 0);
        lv_obj_set_style_text_color(label_soil_, lv_color_hex(0x88FF88), 0);
        lv_label_set_text(label_soil_, "--");
        lv_obj_align(label_soil_, LV_ALIGN_TOP_LEFT, 120, 62);

        // 继电器标识 + 状态标签（平铺）
        label_pump_ = CreateRelayLabel(sensor_panel_, font, "PUMP",  8, 95);
        label_lamp_ = CreateRelayLabel(sensor_panel_, font, "LIGHT", 8, 118);
        label_heat_ = CreateRelayLabel(sensor_panel_, font, "HEAT",  8, 141);
    }

    lv_obj_t* CreateRelayLabel(lv_obj_t* parent, const lv_font_t* font,
                                const char* name, int x, int y) {
        lv_obj_t* label = lv_label_create(parent);
        lv_obj_set_style_text_font(label, font, 0);
        lv_obj_set_style_text_color(label, lv_color_hex(0x808080), 0);
        lv_label_set_text(label, name);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y);
        return label;
    }
};

class CompactWifiBoardS3Cam : public WifiBoard {
private:
    Button boot_button_;
    PlantDisplay* display_;
    Esp32Camera* camera_;
    esp_timer_handle_t display_update_timer_ = nullptr;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef  LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new PlantDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                    DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

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
        config.pin_sccb_sda = CAMERA_PIN_SIOD;
        config.pin_sccb_scl = CAMERA_PIN_SIOC;
        config.sccb_i2c_port = 0;
        config.pin_pwdn = CAMERA_PIN_PWDN;
        config.pin_reset = CAMERA_PIN_RESET;
        config.xclk_freq_hz = XCLK_FREQ_HZ;
        config.pixel_format = PIXFORMAT_RGB565;
        config.frame_size = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        camera_ = new Esp32Camera(config);
        camera_->SetHMirror(false);
    }

    void InitializePlantMonitor() {
        GetPlantMonitor().Initialize();
    }

    void InitializeOnenet() {
        auto& onenet = GetOnenetClient();
        onenet.SetCommandCallback([](const std::string& cmd_json) -> std::string {
            auto& monitor = GetPlantMonitor();
            cJSON* root = cJSON_Parse(cmd_json.c_str());
            if (!root) return "{\"error\":\"invalid json\"}";

            cJSON* temp_min = cJSON_GetObjectItem(root, "temp_min");
            cJSON* temp_max = cJSON_GetObjectItem(root, "temp_max");
            cJSON* humi_min = cJSON_GetObjectItem(root, "humidity_min");
            cJSON* humi_max = cJSON_GetObjectItem(root, "humidity_max");
            cJSON* light_min = cJSON_GetObjectItem(root, "light_min");
            cJSON* light_max = cJSON_GetObjectItem(root, "light_max");

            if (temp_min && temp_max && humi_min && humi_max && light_min && light_max) {
                SensorThresholds t;
                t.temp_min = temp_min->valueint;
                t.temp_max = temp_max->valueint;
                t.humidity_min = humi_min->valueint;
                t.humidity_max = humi_max->valueint;
                t.light_min = light_min->valueint;
                t.light_max = light_max->valueint;
                monitor.SetThresholds(t);
                cJSON_Delete(root);
                return "{\"success\":true,\"message\":\"阈值已更新\"}";
            }

            cJSON* relay = cJSON_GetObjectItem(root, "relay");
            cJSON* state = cJSON_GetObjectItem(root, "state");
            if (relay && state) {
                monitor.SetRelay(relay->valueint, cJSON_IsTrue(state));
                cJSON_Delete(root);
                return "{\"success\":true,\"message\":\"继电器已控制\"}";
            }

            cJSON* refresh = cJSON_GetObjectItem(root, "refresh");
            if (refresh) {
                monitor.Update();
                auto data = monitor.GetSensorData();
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "{\"temperature\":%.1f,\"humidity\":%.1f,\"light\":%d,"
                    "\"soil\":%d,\"pump\":%s,\"light_relay\":%s,\"heater\":%s}",
                    data.temperature, data.humidity, data.light_value,
                    data.soil_moisture,
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

    static void DisplayUpdateCallback(void* arg) {
        auto* self = static_cast<CompactWifiBoardS3Cam*>(arg);
        auto& app = Application::GetInstance();
        auto data = GetPlantMonitor().GetSensorData();

        app.Schedule([self, data]() {
            if (self->display_) {
                self->display_->UpdateSensorDisplay(data);
            }
        });
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

public:
    CompactWifiBoardS3Cam() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeCamera();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        InitializePlantMonitor();
        InitializeOnenet();

        // 启动显示屏定时刷新（每3秒更新一次传感器数据到屏幕）
        esp_timer_create_args_t timer_args = {};
        timer_args.callback = DisplayUpdateCallback;
        timer_args.arg = this;
        timer_args.dispatch_method = ESP_TIMER_TASK;
        timer_args.name = "display_update";
        timer_args.skip_unhandled_events = false;
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &display_update_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(display_update_timer_, 3000000));
    }

    ~CompactWifiBoardS3Cam() {
        if (display_update_timer_) {
            esp_timer_stop(display_update_timer_);
            esp_timer_delete(display_update_timer_);
        }
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

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

DECLARE_BOARD(CompactWifiBoardS3Cam);
