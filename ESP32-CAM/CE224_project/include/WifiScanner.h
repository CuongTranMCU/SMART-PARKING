#ifndef WIFI_SCANNER_H
#define WIFI_SCANNER_H

#include "WiFi.h"

class WiFiScanner
{
public:
  //Khởi tạo wifi
  void init();

  //In ra list các wifi có sẵn
  void scan();

  //Kết nối 1 wifi có sẵn
  void connectWifi(const char *ssid, const char *password);

  //Kiểm tra kết nối
  bool isConnected();

  //Chỉnh sửa biến connected
  void setConnected(bool connected);

private:
  bool connected;
};

#endif // WIFI_SCANNER_H
