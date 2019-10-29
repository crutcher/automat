
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>


const char* ssid = "Greebo";
const char* password = "monkeyshine42";

String topicPrefix;
String stateTopic;
char* server = "192.168.86.214";

StaticJsonDocument<256> doc;


void callback(char* topic, byte* payload, unsigned int length) {
  // handle message arrived
}

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);


String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}


void setup() {
  Serial.begin(115200);
  delay(10);
  
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Generate client name based on MAC address and last 8 bits of microsecond counter
  uint8_t mac[6];
  WiFi.macAddress(mac);
  String macString = macToStr(mac);


  String topicPrefix = "automatCell/" + macString;

  Serial.print("Connecting to ");
  Serial.print(server);
  Serial.print(" as ");
  Serial.println(topicPrefix);

  String helloTopic = topicPrefix + "/hello";
  stateTopic = topicPrefix + "/state";
  
  if (client.connect((char*) topicPrefix.c_str())) {
    if (client.publish(
      (char*) helloTopic.c_str(),
      "hello")) {
        Serial.println("hello");
      }
  }
  else {
    Serial.println("MQTT connect failed");
    Serial.println("Will reset and try again...");
    abort();
  }
}

char outputBuffer[128];

void loop() {
  static int counter = 0;

  doc["micros"] = micros();
  doc["counter"] = counter;
  serializeJson(doc, outputBuffer);
  
  if (client.connected()){
   
    if (!client.publish((char*) stateTopic.c_str(), outputBuffer)) {
      Serial.println("Publish failed");
    }
  }
  ++counter;
}
