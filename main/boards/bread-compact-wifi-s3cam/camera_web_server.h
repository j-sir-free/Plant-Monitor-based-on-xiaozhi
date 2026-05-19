#ifndef CAMERA_WEB_SERVER_H
#define CAMERA_WEB_SERVER_H

#include <esp_http_server.h>

class CameraWebServer {
public:
    CameraWebServer();
    ~CameraWebServer();

    bool Start(int port = 80);
    void Stop();
    bool IsRunning() const { return server_handle_ != nullptr; }

private:
    httpd_handle_t server_handle_ = nullptr;

    static esp_err_t IndexHandler(httpd_req_t* req);
    static esp_err_t StreamHandler(httpd_req_t* req);
    static esp_err_t SnapshotHandler(httpd_req_t* req);
    static esp_err_t ApiSensorsHandler(httpd_req_t* req);
    static CameraWebServer* instance_;
};

#endif
