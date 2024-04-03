#include "Node.h"

String createJsonData(const std::map<String, String> &keyValuePairs)
{
  // Tạo đối tượng JSON
  DynamicJsonDocument jsonDoc(256);

  // Thêm các cặp key-value vào JSON
  for (const auto &pair : keyValuePairs)
  {
    jsonDoc[pair.first] = pair.second;
  }

  // Chuyển đổi JSON thành chuỗi
  String jsonData;
  serializeJson(jsonDoc, jsonData);

  return jsonData;
}

void Node::RequestPOST(const char *severurl, const std::map<String, String> &data, String &response)
{

  // Tạo URL hoàn chỉnh bằng cách kết hợp serverUrl với đường dẫn cụ thể (nếu có)
  String url = severurl;

  // Bắt đầu yêu cầu HTTP
  http.begin(url);

  // Thiết lập tiêu đề của yêu cầu
  http.addHeader("Content-Type", "application/json");

  // Tạo dữ liệu JSON để gửi
  String jsonData = createJsonData(data);

  // Gửi yêu cầu POST đến server với dữ liệu JSON
  int httpCode = http.POST(jsonData);

  // Kiểm tra kết quả
  if (httpCode > 0)
    response = http.getString();
  else
    response = "Can not connect to sever";

  // Đóng kết nối
  http.end();
}

void Node::RequestGET(const char *severurl, String &response)
{

  // Tạo URL hoàn chỉnh bằng cách kết hợp serverUrl với đường dẫn cụ thể (nếu có)
  String url = severurl;

  // Bắt đầu yêu cầu HTTP
  http.begin(url);

  // Gửi yêu cầu POST đến server với dữ liệu JSON
  int httpCode = http.GET();

  // Kiểm tra kết quả
  if (httpCode > 0)
    response = http.getString();
  else
    response = "Can not connect to sever";

  // Đóng kết nối
  http.end();
}

void Node::CreateCard(const char *card_uid, String &response)
{
  // Tạo URL hoàn chỉnh bằng cách kết hợp serverUrl với đường dẫn cụ thể (nếu có)
  String url = "https://ce224.azurewebsites.net/api/createCard";

  // Bắt đầu yêu cầu HTTP
  http.begin(url);

  // Thiết lập tiêu đề của yêu cầu
  http.addHeader("Content-Type", "application/json");

  // Tạo dữ liệu JSON để gửi
  String jsonData = "{\"card_uid\":\"" + String(card_uid) + "\"}";

  // Gửi yêu cầu POST đến server với dữ liệu JSON
  int httpCode = http.POST(jsonData);

  // Kiểm tra kết quả
  if (httpCode > 0)
    response = http.getString();
  else
    response = "Can not connect to sever";

  // Đóng kết nối
  http.end();
}

void Node::Action(const char *card_uid, const char *plate, String &response) 
{
   // Tạo URL hoàn chỉnh bằng cách kết hợp serverUrl với đường dẫn cụ thể (nếu có)
  String url = "https://ce224.azurewebsites.net/api/action";

  // Bắt đầu yêu cầu HTTP
  http.begin(url);

  // Thiết lập tiêu đề của yêu cầu
  http.addHeader("Content-Type", "application/json");

  // Tạo dữ liệu JSON để gửi
  String jsonData = "{\"card_uid\":\"" + String(card_uid) + "\",\"license_plates\":\"" + String(plate) + "\"}";

  // Gửi yêu cầu POST đến server với dữ liệu JSON
  int httpCode = http.POST(jsonData);

  // Kiểm tra kết quả
  if (httpCode > 0)
    response = http.getString();
  else
    response = "Can not connect to sever";

  // Đóng kết nối
  http.end();
}
