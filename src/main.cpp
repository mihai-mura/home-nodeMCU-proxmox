#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ESP8266Ping.h>
#include <ArduinoJson.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <PubSubClient.h>
#include <env.cpp>

// #define DEBUG ;

// WiFi config
const IPAddress ip(192, 168, 100, 54);
const IPAddress gateway(192, 168, 100, 1);
const IPAddress bcastAddr(192, 168, 100, 255);
const IPAddress subnet(255, 255, 255, 0);
const IPAddress dns(192, 168, 100, 1);
AsyncWebServer server(80);

// WOL device config
const IPAddress device_ip(192, 168, 100, 200);
byte macAddr[6] = {0x00, 0x23, 0x24, 0xBA, 0x55, 0x2F};
const uint16_t boot_time = 200;
const int startStopTimeMargin = 10; // in seconds

// WOL packet config
#define MAGIC_PACKET_LENGTH 102
#define PORT_WAKEONLAN 9
byte magicPacket[MAGIC_PACKET_LENGTH];
unsigned int localPort = 9;
WiFiUDP udp;

// timer
const unsigned long timerInterval = 5000;
unsigned long timerPreviousTime = 0;

// proxmox state
struct WOLServerState
{
  bool IsOnline;
  uint16_t boot_time;
  bool boot_error;
  uint16_t ping;
  uint32_t previousMillis;
  uint32_t interval;
  // first start & stop event notification
  bool firstStart;
  bool firstStop;
  bool stoppedCountDown;
  int stoppedTime;
};
WOLServerState currentState = {false, 0, false, 0, 0, 5000UL, true, true, false, 0};

// MQTT config
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String clientId = "NodeMCU-Proxmox";

DynamicJsonDocument StateJSON(1024);
DynamicJsonDocument inDoc(1024);

void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.hostname("NodeMCU-Proxmox");
  WiFi.config(ip, dns, gateway, subnet);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int count = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(250);
    digitalWrite(LED_BUILTIN, LOW);
#ifdef DEBUG
    Serial.print(".");
#endif

    count++;
    if (count > 20)
    {
      delay(500);
      ESP.restart();
    }
  }

#ifdef DEBUG
  Serial.println("\nWifi Connected, Ip: " + WiFi.localIP().toString());
#endif
}

void enableOTA()
{
  AsyncElegantOTA.begin(&server);
  server.begin();
#ifdef DEBUG
  Serial.println("HTTP server started");
#endif
}

void buildMagicPacket()
{
  memset(magicPacket, 0xFF, 6);
  for (int i = 0; i < 16; i++)
  {
    int ofs = i * sizeof(macAddr) + 6;
    memcpy(&magicPacket[ofs], macAddr, sizeof(macAddr));
  }
}

void startUDP()
{
  if (udp.begin(localPort) == 1)
  {
#ifdef DEBUG
    Serial.println("UDP started");
#endif
    buildMagicPacket();
  }
  else
  {
    delay(500);
    ESP.restart();
  }
}

void startProxmox()
{
#ifdef DEBUG
  Serial.println("Proxmox started");
#endif

  if (!currentState.IsOnline && currentState.boot_time == 0)
  {
    udp.beginPacket(bcastAddr, PORT_WAKEONLAN);
    udp.write(magicPacket, MAGIC_PACKET_LENGTH);
    udp.endPacket();

    currentState.boot_time = boot_time;
    currentState.interval = 1000UL;
  }
}

void mqttMessageHandler(char *topic, byte *payload, unsigned int length)
{
  char message[length + 1];
  memcpy(message, payload, length);
  message[length] = '\0';

  if (strcmp(topic, "proxmox/on") == 0)
  {
    startProxmox();
  }

#ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println(message);
#endif
}

void connectMQTT()
{
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttMessageHandler);
}

void reconnectMQTT()
{
  // Loop until we're reconnected
  while (!mqttClient.connected())
  {
#ifdef DEBUG
    Serial.print("Attempting MQTT connection...");
#endif
    // Attempt to connect
    if (mqttClient.connect(clientId.c_str()))
    {
#ifdef DEBUG
      Serial.println("connected");
#endif
      //* MQTT startup
      mqttClient.publish("proxmox/nodeMCU-on", "true");
      mqttClient.subscribe("proxmox/on");
    }
    else
    {
#ifdef DEBUG
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
#endif
      delay(5000);
    }
  }
}

void sendState(String ping, String rssi, String serverState)
{
  StateJSON["ping"] = ping;
  StateJSON["rssi"] = rssi;
  StateJSON["serverState"] = serverState;
  String json;
  serializeJson(StateJSON, json);
  mqttClient.publish("proxmox/state", json.c_str());
}

void timerFunction()
{

  if (currentState.IsOnline)
  {
    sendState(String(currentState.ping), String(WiFi.RSSI()), "online");
  }
  else if (!currentState.IsOnline && currentState.boot_time > 0)
  {
    sendState(String(currentState.ping), String(WiFi.RSSI()), "waiting");
  }
  else if (!currentState.IsOnline && currentState.boot_time == 0 && currentState.boot_error)
  {
    sendState(String(currentState.ping), String(WiFi.RSSI()), "error");
  }
  else
  {
    sendState(String(currentState.ping), String(WiFi.RSSI()), "offline");
  }

  if (currentState.stoppedCountDown && currentState.stoppedTime < startStopTimeMargin)
  {
    currentState.stoppedTime++;
  }
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

  connectWiFi();
  enableOTA();
  connectMQTT();
  startUDP();
}

void loop()
{
  // Reconnect WiFi
  if (WiFi.status() != WL_CONNECTED)
  {
    connectWiFi();
    return;
  }

  // Reconnect MQTT
  if (!mqttClient.connected())
  {
    reconnectMQTT();
  }

  mqttClient.loop();

  // set local proxmox state
  uint32_t currentMillis = millis();
  if (currentMillis - currentState.previousMillis >= currentState.interval)
  {
    currentState.previousMillis = currentMillis;

    if (currentState.boot_time == 0)
    {
      currentState.interval = 5000UL;
    }
    else
    {
      currentState.boot_time--;
      if (currentState.boot_time == 0)
      {
        currentState.boot_error = true;
        mqttClient.publish("proxmox/event", "boot_error");
      }
    }

    if (Ping.ping(device_ip, 1))
    {
      currentState.IsOnline = true;
      currentState.boot_error = false;
      currentState.boot_time = 0;
      currentState.ping = Ping.averageTime();
      currentState.firstStop = true;
      // first start notification
      if (currentState.firstStart)
      {

        mqttClient.publish("proxmox/event", "server_on");
        currentState.firstStart = false;
      }
      currentState.stoppedCountDown = false;
      currentState.stoppedTime = 0;
    }
    else
    {
      currentState.stoppedCountDown = true;
      if (currentState.stoppedTime == startStopTimeMargin)
      {
        currentState.IsOnline = false;
        currentState.ping = 0;
        currentState.firstStart = true;
        // first stop notification
        if (currentState.firstStop)
        {
          mqttClient.publish("proxmox/event", "server_off");
          currentState.firstStop = false;
        }
      }
    }
  }

  // timer run
  unsigned long currentTime = millis();
  if (currentTime - timerPreviousTime >= timerInterval)
  {
    timerFunction();
    timerPreviousTime = currentTime;
  }
}
