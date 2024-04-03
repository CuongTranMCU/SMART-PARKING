/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* Includes ---------------------------------------------------------------- */
#include <bat1871999-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"

#include <Arduino.h>
#include "WifiScanner.h"
#include "Node.h"
#include <HardwareSerial.h>
#include <cstring>

/* Defines ----------------------------------------------------------------- */
// Enable/Disable Serial debug - comment to disable
// #define SERIAL_DEBUG

/* Camera parameters ------------------------------------------------------- */
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define LED_BUILTIN 4

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3

/* Function definitions ---------------------------------------------------- */

void ei_camera_init(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void get_license_plate(char *license_plate);

void handleIdResponse(const String &jsonResponse);
void handleJsonResponse(const String &jsonResponse);

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; // points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    // XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, // YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,   // QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, // 0-63 lower number means higher quality
    .fb_count = 1,      // if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

WiFiScanner wifi;
Node client;
const char *ssid = "Nasmes_INF";                                 // Tên Wifi
const char *password = "167294381";                        // Mật khẩu wifi
const char *severUrl = "https://ce224.azurewebsites.net"; // sever url
String response;                                          // reponse từ sever
String data_to_server;
String data_from_stm;
String data_to_stm;
char license_plate[10] = {'\0'};

/**
 * @brief      Arduino setup function
 */
void setup()
{
  // Setup Flashlight
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(4, LOW);

  // Start UART
  Serial.begin(115200);
  while (!Serial) ;

  // Start camera
  ei_camera_init();

#ifdef SERIAL_DEBUG
  if (is_initialised)
  {
    Serial.print("Camera initialized\r\n");
  }
  else
  {
    Serial.print("Failed to initialize Camera!\r\n");
  }
#endif

  // Start wifi
  wifi.init();                      // Khởi tạo wifi
  wifi.connectWifi(ssid, password); // Kết nối wifi

  if (wifi.isConnected())
  {
    const char *endpoint = "/welcome";
    char url[256];

    strcpy(url, severUrl);
    strcat(url, endpoint);

    client.RequestGET(url, response);
#ifdef SERIAL_DEBUG
    Serial.println(response); // Sẽ in ra message: welcome.
#endif
  }

  delay(1000);
}

/**
 * @brief      Main loop.
 */
void loop()
{

  if (Serial.available()) // kiểm tra nếu có dữ liệu được truyền qua uart từ stm
  {
    char buffer[10] = {'\0'};
    if (Serial.readBytes(buffer, 9) > 0)
    {
      data_from_stm = buffer; // đọc dữ liệu truyền được truyền từ stm
    }
    data_to_server = data_from_stm.substring(0, 8);
    const char *data_to_server_cstr = data_to_server.c_str();
    String mode = data_from_stm.substring(8);
    const char *mode_cstr = mode.c_str();
    

#ifdef SERIAL_DEBUG
    Serial.println(buffer);
    Serial.println(data_to_server);
    Serial.println(data_to_server_cstr);
    Serial.println(mode);
    Serial.println(mode_cstr);
#endif
    if (strcmp(mode_cstr, "1") == 0)
    {
      // Serial.println(data_to_server_cstr);
      client.CreateCard(data_to_server_cstr, response);
      // Serial.println(response);
      handleIdResponse(response);
    }
    else
    {
      // Lay bien so xe
      get_license_plate(license_plate);
      
      for(int i = 5; i != 0; i--)
      {
        get_license_plate(license_plate);
        if (license_plate == nullptr)
        {
          return;
        }
        if (strcmp(license_plate, "no_detect") != 0) break; //nếu lấy được biển số thì break
      }

      if (strcmp(license_plate, "no_detect") == 0)
      {
#ifdef SERIAL_DEBUG
        Serial.println("Cannot detect license plate!"); // báo lỗi không thể nhận diện biển số
#endif
        Serial.write('0'); // trả về stm32 ký tự 0 = không cho ra
      }
      else
      {
        client.Action(data_to_server_cstr,license_plate, response);
        handleJsonResponse(response);
      }
    }
  }

  // Serial avaiable
  // Read Serial

  // Server
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
void ei_camera_init(void)
{

  if (is_initialised)
    return;

  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK)
    return;

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);      // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, 0); // lower the saturation
  }

  is_initialised = true;
  return;
}

/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf)
{
  bool do_resize = false;

  if (!is_initialised)
  {
#ifdef SERIAL_DEBUG
    ei_printf("ERR: Camera is not initialized\r\n");
#endif
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb)
  {
#ifdef SERIAL_DEBUG
    ei_printf("Camera capture failed\n");
#endif
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

  esp_camera_fb_return(fb);

  if (!converted)
  {
#ifdef SERIAL_DEBUG
    ei_printf("Conversion failed\n");
#endif
    return false;
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS) || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS))
  {
    do_resize = true;
  }

  if (do_resize)
  {
    ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
  }

  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
  // we already have a RGB888 buffer, so recalculate offset into pixel index
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0)
  {
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  // and done!
  return 0;
}

/**
 * @brief       Take a picture and put the license plate into the char pointer.
 *              Null terminated, license plate is first 9 characters.
 *              If pointer is null, nothing will be done.
 *
 * @param[in]   license_plate   Char array pointer, must be at least 10 in length.
 */
void get_license_plate(char *license_plate)
{
  if (license_plate == nullptr)
  {
#ifdef SERIAL_DEBUG
    ei_printf("ERR: License plate pointer is null!\n");
#endif
    return;
  }

  // Allocate buffer for the image captured
  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

  // check if allocation was successful
  if (snapshot_buf == nullptr)
  {
#ifdef SERIAL_DEBUG
    ei_printf("ERR: Failed to allocate buffer!\n");
#endif
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false)
  {
#ifdef SERIAL_DEBUG
    ei_printf("Failed to capture image\r\n");
#endif
    free(snapshot_buf);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = {0};

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK)
  {
#ifdef SERIAL_DEBUG
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
#endif
    return;
  }

  size_t pred_index = 0;
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
  {
    if (result.classification[ix].value > result.classification[pred_index].value)
      pred_index = ix;
  }
  if (!strcmp(result.classification[pred_index].label, "no_detection"))
    strcpy(license_plate, "no_detect");
  else
    strcpy(license_plate, result.classification[pred_index].label);
  license_plate[9] = '\0';

#ifdef SERIAL_DEBUG
  // print the predictions
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n    %s: %.5f\n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly,
            license_plate, result.classification[pred_index].value);
#endif

  free(snapshot_buf);
}

/**
 * @brief       Parse json response from server and perform actions
 *
 * @param[in]   jsonResponse    json response from server
 *
 * @retval      None
 */
void handleIdResponse(const String &jsonResponse)
{
  if (jsonResponse == "Can not connect to sever")
  {
#ifdef SERIAL_DEBUG
    Serial.println("Can not connect to sever");
    // Thực hiện hành động khi không được phép
    Serial.print("Sending data to STM32: ");
#endif
    Serial.write('5');
  }

  // parse json response for ID check
  const size_t capacity = JSON_OBJECT_SIZE(1) + 70;
  DynamicJsonDocument doc(capacity * 4);
  DeserializationError error = deserializeJson(doc, jsonResponse);
  if (error)
  {
#ifdef SERIAL_DEBUG
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
#endif
    return;
  }

  // Kiểm tra xem id có tồn tại không
  const long id = doc["id"].as<long>();
  if (id)
  {
#ifdef SERIAL_DEBUG
    Serial.print("Server response - ID Check: ");
    Serial.println(id);
#endif

    // Kiểm tra giá trị của id để xác định hành động
    if (id > 0)
    {
      // Nếu id khác rỗng, có thể thực hiện các xử lý khác ở đây nếu cần
#ifdef SERIAL_DEBUG
      Serial.println("Success to resgister!");
      Serial.print("Sending data to STM32: ");
#endif
      Serial.write('3'); // gửi về stm32
    }
    else
    {
      // Nếu id rỗng, in ra "Fail to register!"
#ifdef SERIAL_DEBUG
      Serial.println("Fail to register!");
      Serial.print("Sending data to STM32: ");
#endif
      Serial.write('4');
    }
  }
#ifdef SERIAL_DEBUG
  else
  {
    Serial.println("Invalid JSON response, missing 'id' field");
  }
#endif
}

/**
 * @brief       Parse json response from server and perform actions
 *
 * @param[in]   jsonResponse    json response from server
 *
 * @retval      None
 */
void handleJsonResponse(const String &jsonResponse)
{
  if (jsonResponse == "Can not connect to sever")
  {
#ifdef SERIAL_DEBUG
    Serial.println("Can not connect to sever");
    // Thực hiện hành động khi không được phép
    Serial.print("Sending data to STM32: ");
#endif
    Serial.write('5');
  }

  // parse json response
  const size_t capacity = JSON_OBJECT_SIZE(2) + 70;
  DynamicJsonDocument doc(capacity * 4);
  // desertialize json data
  DeserializationError error = deserializeJson(doc, jsonResponse);
  // kiểm tra lỗi khi parse json
  if (error)
  {
#ifdef SERIAL_DEBUG
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
#endif
    return;
  }
  // Kiểm tra xem message là "allowed" hay "not allowed"
  const char *message = doc["message"];
  const char *error_message = doc["error"];
  if (message)
  {
#ifdef SERIAL_DEBUG
    Serial.print("Server response (message): ");
    Serial.println(message);
    Serial.print("Server response(error): ");
    Serial.println(error_message);
#endif

    // Kiểm tra giá trị cụ thể của message để xác định hành động
    if (strcmp(message, "Allowed to get in") == 0)
    {
#ifdef SERIAL_DEBUG
      Serial.println("Action: Allowed");
      // Thực hiện hành động khi được phép
      Serial.print("Sending data to STM32: ");
#endif
      Serial.write('1');
    }
    else if (strcmp(message, "Allowed to get out") == 0)
    {
      
#ifdef SERIAL_DEBUG
      Serial.println("Action: Allowed");
      // Thực hiện hành động khi được phép
      Serial.print("Sending data to STM32: ");
#endif
      Serial.write('2');
    }
    else if (strcmp(message, "Not Allowed to get out.") == 0)
    {
      if (strcmp(error_message, "Not enough money in account.") == 0)
      {
#ifdef SERIAL_DEBUG
        Serial.println("Action: Out of money");
        // Thực hiện hành động khi hết tiền
        Serial.print("Sending data to STM32: ");
#endif
        Serial.write('6');
      }
      else
      {
#ifdef SERIAL_DEBUG
        Serial.println("Action: Not Allowed");
        // Thực hiện hành động khi không được phép
        Serial.print("Sending data to STM32: ");
#endif
        Serial.write('0');
      }
    }
    else
    {
#ifdef SERIAL_DEBUG
      Serial.println("Unknown message");
      Serial.print("Sending data to STM32: ");
#endif
      Serial.write('0');
    }
  }
#ifdef SERIAL_DEBUG
  else
  {
    Serial.println("Invalid JSON response, missing 'message' field");
  }
#endif
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif