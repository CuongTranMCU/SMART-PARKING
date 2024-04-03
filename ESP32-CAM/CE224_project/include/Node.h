#ifndef SEVER_H
#define SEVER_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <map>

class Node
{
public:
    //Gửi GET request tới url
    void RequestGET(const char *url, String &response);

    //Gửi POST request tới url cùng với dữ liệu là 1 map key->value
    void RequestPOST(const char *url, const std::map<String, String>& data, String &response);

    //Áp dụng cho đồ án. Gửi request Tạo RFID trống với dữ liệu truyền là card_uid
    void CreateCard(const char *card_uid, String &response);

    //Áp dụng cho đồ án. Gửi request ra vào với dữ liệu là card_uid và plate(biển số xe)
    void Action(const char *card_uid, const char *plate, String &response);

private:
    HTTPClient http;
};

#endif