#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#ifndef STASSID
#define STASSID "Greebo"
#define STAPSK  "monkeyshine42"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

byte mac[6];

WiFiUDP Udp;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  // Clear Serial noise.
  Serial.println("");

  // Setup WIFI.
  Serial.print("Connecting to:");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  // FInd and print the MAC address.
  Serial.print("system_id:");
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
  for (int i = 5; i >= 0; --i) {
    Serial.print(mac[i], HEX);
    Serial.print(":");
  }
  Serial.println("");
}

char serverAddress[] = "192.168.86.21";
int serverPort = 5005;

char UdpMsgBuffer[] = "message body";

void loop() {
  // put your main code here, to run repeatedly:
  Udp.beginPacket(serverAddress, serverPort);
  Udp.write(UdpMsgBuffer);
  Udp.endPacket();
}
