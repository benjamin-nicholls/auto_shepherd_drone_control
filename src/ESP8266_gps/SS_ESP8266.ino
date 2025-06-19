#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>

// WiFi credentials
// const char* ssid = "Nothing"; // Change to network SSID
// const char* password = "123456789"; // Change to network password

const char* ssid = "RoboShepherd"; // Change to network SSID
const char* password = "Name1234"; // Change to network password

// Host IP and port of the machine running ROS 2 node
//const char* host = "192.168.142.107"; // Change to Socket receiver device IP
const char* host = "192.168.0.165";
const uint16_t port = 5000;
int kk = 1;
TinyGPSPlus gps;
WiFiClient client;

// Use SoftwareSerial for GPS on pins D5 (RX) and D6 (TX) or change as needed
SoftwareSerial gpsSerial(14, 12); // RX, TX on ESP8266

void setup() {
  Serial.begin(115200);      
  gpsSerial.begin(9600);       

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    
  }
  client.connect(host, port);
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
    //kk = kk + 1;
    //Serial.println(kk + 1);
    
    if (gps.location.isUpdated()) {
      if (client.connected()) {
        String payload = String(gps.satellites.value()) + "," +
                        String(gps.location.lat(), 6) + "," +
                        String(gps.location.lng(), 6) + "," +
                        String(gps.altitude.meters(), 2) + "," +
                        String(gps.hdop.hdop(), 2);
        client.println(payload);
        client.flush();  // Optional, but ensures it's sent immediately
        //Serial.println(String(gps.hdop.hdop(), 2));
      }
    }

      //Serial.println(payload);  // Print to Serial Monitor
  }
}
