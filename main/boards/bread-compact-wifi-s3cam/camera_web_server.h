/**
 * @file    camera_web_server.h
 * @brief   摄像头 Web 服务器 — 内嵌 HTTP 服务, 提供实时视频流和传感器数据 API
 * @author  JJD-YOURFATHER
 *
 * 基于 ESP-IDF httpd 组件实现的轻量级 Web 服务器。
 * 端口 80, 单例模式 (静态 instance_ 指针), 自动在 WiFi 就绪后启动。
 *
 * 4 个 HTTP 端点:
 *   GET /              — HTML5 仪表盘页面
 *   GET /stream        — MJPEG 实时视频流 (multipart/x-mixed-replace)
 *   GET /snapshot      — JPEG 单帧快照
 *   GET /api/sensors   — JSON 传感器数据 API
 */
#ifndef CAMERA_WEB_SERVER_H
#define CAMERA_WEB_SERVER_H

#include <esp_http_server.h>

class CameraWebServer {
public:
    CameraWebServer();
    ~CameraWebServer();

    /**
     * 启动 HTTP 服务器
     * @param port  监听端口 (默认 80)
     * @return      true=启动成功, false=启动失败
     */
    bool Start(int port = 80);

    /** 停止 HTTP 服务器并释放资源 */
    void Stop();

    /** 检查服务器是否正在运行 */
    bool IsRunning() const { return server_handle_ != nullptr; }

private:
    httpd_handle_t server_handle_ = nullptr;    // ESP-IDF HTTP 服务器句柄

    // ---- HTTP 请求处理器 (静态函数, 作为 httpd_uri_t::handler 注册) ----

    /** 首页 — 返回内嵌 HTML5 仪表盘页面 */
    static esp_err_t IndexHandler(httpd_req_t* req);

    /**
     * MJPEG 视频流 — 循环捕获摄像头帧, JPEG 编码, 以 multipart 格式持续推送
     * 客户端断开连接时自动停止
     */
    static esp_err_t StreamHandler(httpd_req_t* req);

    /** 单帧快照 — 捕获一帧, JPEG 编码, 返回 image/jpeg */
    static esp_err_t SnapshotHandler(httpd_req_t* req);

    /**
     * 传感器 API — 返回 JSON 格式的传感器数据
     * 包含: temperature, humidity, light, soil_moisture_percent/raw/digital,
     *       relay 状态, thresholds 阈值
     */
    static esp_err_t ApiSensorsHandler(httpd_req_t* req);

    static CameraWebServer* instance_;          // 单例指针 (供静态 handler 访问)
};

#endif
