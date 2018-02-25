#include <WiFi.h>
#include <PubSubClient.h>

#define LED0 17
#define LED1 18
#define LED2 19
#define SERVO_PIN 5

// wifi configuration
const char* ssid = "my_ssid";
const char* password = "my_wifi_password";
// mqtt configuration
const char* server = "192.168.1.111"; // mqtt broker ip
const char* topic = "/servo";
const char* clientName = "com.gonzalo123.esp32";

int channel = 1;
int hz = 50;
int depth = 16;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void wifiConnect() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("*");
  }

  Serial.print("WiFi connected: ");
  Serial.println(WiFi.localIP());
}

void mqttReConnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(clientName)) {
      Serial.println("connected");
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);

  String data;
  for (int i = 0; i < length; i++) {
    data += (char)payload[i];
  }

  int value = data.toInt();
  cleanLeds();
  switch (value)  {
    case 0:
      ledcWrite(1, 3400);
      digitalWrite(LED0, HIGH);
      break;
    case 1:
      ledcWrite(1, 4900);
      digitalWrite(LED1, HIGH);
      break;
    case 2:
      ledcWrite(1, 6400);
      digitalWrite(LED2, HIGH);
      break;
  }
  Serial.print("] value:");
  Serial.println((int) value);
}

void cleanLeds() {
  digitalWrite(LED0, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
}

void setup() {
  Serial.begin(115200);

  ledcSetup(channel, hz, depth);
  ledcAttachPin(SERVO_PIN, channel);

  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  cleanLeds();
  wifiConnect();
  client.setServer(server, 1883);
  client.setCallback(callback);

  delay(1500);
}

void loop()
{
  if (!client.connected()) {
    mqttReConnect();
  }

  client.loop();
  delay(100);
}