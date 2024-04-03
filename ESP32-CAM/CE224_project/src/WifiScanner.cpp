#include "WifiScanner.h"

bool WiFiScanner::isConnected() {
  return connected;
}

void WiFiScanner::setConnected(bool connected){
    this->connected = connected;
}
void WiFiScanner::init()
{
    Serial.begin(115200);
    Serial.println("Initializing WiFi...");
    WiFi.mode(WIFI_STA);
    Serial.println("Setup done!");
}

void WiFiScanner::scan()
{
    Serial.println("Scanning...");

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
    Serial.println("Scan done!");
    if (n == 0)
    {
        Serial.println("No networks found.");
    }
    else
    {
        Serial.println();
        Serial.print(n);
        Serial.println(" networks found");
        for (int i = 0; i < n; ++i)
        {
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
            delay(10);
        }
    }

    Serial.println("");

    // Wait a bit before scanning again
    delay(5000);
}

void WiFiScanner::connectWifi(const char* ssid, const char* password) {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
#ifdef SERIAL_DEBUG
    Serial.println("Connecting to WiFi...");
#endif
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
#ifdef SERIAL_DEBUG
    Serial.println("Connected to WiFi Successfully");
#endif
    setConnected(true);
  } else {
#ifdef SERIAL_DEBUG
    Serial.println("Failed to connect to WiFi");
#endif
    setConnected(false);
  }
}

