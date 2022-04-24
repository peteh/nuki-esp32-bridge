/**
 * A BLE client example to connect to Nuki smartlock 2.0
 * author Jeroen
 */

#include <Arduino.h>
#include <NukiBle.h>
#include <NukiConstants.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include <MqttDevice.h>

#include "utils.h"
#include "config.h"

uint32_t deviceId = 2020001;
std::string deviceName = "Home";
Nuki::NukiBle nukiBle(deviceName, deviceId);
BleScanner scanner;
Nuki::KeyTurnerState retrievedKeyTurnerState;
Nuki::BatteryReport _batteryReport;
std::list<Nuki::LogEntry> requestedLogEntries;
std::list<Nuki::KeypadEntry> requestedKeypadEntries;
std::list<Nuki::AuthorizationEntry> requestedAuthorizationEntries;
std::list<Nuki::TimeControlEntry> requestedTimeControlEntries;

WiFiClient net;
PubSubClient client(net);
const char *HOMEASSISTANT_STATUS_TOPIC = "homeassistant/status";
const char *HOMEASSISTANT_STATUS_TOPIC_ALT = "ha/status";
MqttDevice mqttDevice(composeClientID().c_str(), "Nuki", "Nuki ESP32 Bridge", "maker_pt");
MqttLock mqttLock(&mqttDevice, "lock", "Nuki");
MqttSensor mqttBattery(&mqttDevice, "battery", "Nuki");

void batteryReport()
{
  uint8_t result = nukiBle.requestBatteryReport(&_batteryReport);
  if (result == 1)
  {
    log_d("Bat report voltage: %d Crit state: %d, start temp: %d", _batteryReport.batteryVoltage, _batteryReport.criticalBatteryState, _batteryReport.startTemperature);
  }
  else
  {
    log_d("Bat report failed: %d", result);
  }
}

bool getKeyTurnerStateFromLock()
{
  uint8_t result = nukiBle.requestKeyTurnerState(&retrievedKeyTurnerState);
  if (result == 1)
  {
    log_d("Bat crit: %d, Bat perc:%d lock state: %d %d:%d:%d",
          nukiBle.isBatteryCritical(), nukiBle.getBatteryPerc(), retrievedKeyTurnerState.lockState, retrievedKeyTurnerState.currentTimeHour,
          retrievedKeyTurnerState.currentTimeMinute, retrievedKeyTurnerState.currentTimeSecond);
  }
  else
  {
    log_d("cmd failed: %d", result);
  }
  return result;
}


void publishConfig(MqttEntity *entity)
{
  String payload = entity->getHomeAssistantConfigPayload();
  char topic[255];
  entity->getHomeAssistantConfigTopic(topic, sizeof(topic));
  client.publish(topic, payload.c_str());

  entity->getHomeAssistantConfigTopicAlt(topic, sizeof(topic));
  client.publish(topic,
                 payload.c_str());
}


void publishConfig()
{
  publishConfig(&mqttLock);
  publishConfig(&mqttBattery);
}

void publishLockState(Nuki::NukiBle &nuki)
{
  Nuki::KeyTurnerState state;
  nukiBle.retrieveKeyTunerState(&state);
  if(state.lockState == Nuki::LockState::Locked)
  {
    client.publish(mqttLock.getStateTopic(), mqttLock.getLockedState());
  }
  else
  {
    client.publish(mqttLock.getStateTopic(), mqttLock.getUnlockedState());
  }
  
}

void getConfig() {
  Nuki::Config config;
  if (nukiBle.requestConfig(&config) == 1) {
    log_d("Name: %s", config.name);
  } else {
    log_w("getConfig failed");
  }

}

void connectToMqtt()
{
  Serial.print("\nconnecting to MQTT...");
  // TODO: add security settings back to mqtt
  // while (!client.connect(mqtt_client, mqtt_user, mqtt_pass))
  while (!client.connect(composeClientID().c_str()))
  {
    Serial.print(".");
    delay(4000);
  }

  client.subscribe(mqttLock.getCommandTopic(), 1);

  client.subscribe(HOMEASSISTANT_STATUS_TOPIC);
  client.subscribe(HOMEASSISTANT_STATUS_TOPIC_ALT);

  publishConfig();
}

void connectToWifi()
{
  Serial.print("Connecting to wifi...");
  // TODO: really forever? What if we want to go back to autoconnect?
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\n Wifi connected!");
}

bool newCommandAvailable = false;
Nuki::LockAction newCommand = Nuki::LockAction::Lock;


bool notified = false;
class Handler : public Nuki::SmartlockEventHandler
{
public:
  virtual ~Handler(){};
  void notify(Nuki::EventType eventType)
  {
    if (eventType == Nuki::EventType::KeyTurnerStatusUpdated)
    {
      notified = true;
    }
  }
};

Handler handler;

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // TODO length check

  if (strcmp(topic, mqttLock.getCommandTopic()) == 0)
  {
    if (strncmp((char *)payload, mqttLock.getLockCommand(), length) == 0)
    {
      newCommand = Nuki::LockAction::Lock;
      newCommandAvailable = true;
    }
    else if (strncmp((char *)payload, mqttLock.getUnlockCommand(), length) == 0)
    {
      newCommand = Nuki::LockAction::Unlock;
      newCommandAvailable = true;
    }
    else if (strncmp((char *)payload, mqttLock.getOpenCommand(), length) == 0)
    {
      newCommand = Nuki::LockAction::Unlatch;
      newCommandAvailable = true;
    }

    // TODO: error unknown command
  }

  // publish config when homeassistant comes online and needs the configuration again
  else if (strcmp(topic, HOMEASSISTANT_STATUS_TOPIC) == 0 ||
           strcmp(topic, HOMEASSISTANT_STATUS_TOPIC_ALT) == 0)
  {
    if (strncmp((char *)payload, "online", length) == 0)
    {
      publishConfig();
    }
  }
}
void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.hostname(composeClientID().c_str());
  WiFi.begin(wifi_ssid, wifi_pass);

  connectToWifi();
    ArduinoOTA.onStart([]()
                       {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type); });
    ArduinoOTA.onEnd([]()
                     { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    } });
    ArduinoOTA.begin();

  Serial.println();
  Serial.print("Connected to SSID: ");
  Serial.println(wifi_ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  client.setBufferSize(512);
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  log_d("Starting NUKI BLE...");
  scanner.initialize();
  nukiBle.registerBleScanner(&scanner);
  nukiBle.initialize();

  if (nukiBle.isPairedWithLock())
  {
    log_d("paired");
    nukiBle.setEventHandler(&handler);
    getConfig();
    nukiBle.enableLedFlash(false);
  }
  nukiBle.saveSecurityPincode(nuki_pin);
  // nukiBle.unPairNuki();
}
unsigned long last_update = 0;

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    connectToWifi();
  }
  if (!client.connected())
  {
    connectToMqtt();
  }
  client.loop();
  ArduinoOTA.handle();
  scanner.update();
  if (!nukiBle.isPairedWithLock())
  {
    digitalWrite(LED_BUILTIN, LOW);
    if (nukiBle.pairNuki())
    {
      log_d("paired");
      nukiBle.setEventHandler(&handler);
      getConfig();
    }
  }

  if(newCommandAvailable)
  {
    Nuki::LockAction action = newCommand;
    newCommandAvailable = false;
    // TODO: use proper nonce
    if(nukiBle.lockAction(action, deviceId, 0, NULL, 0) == Nuki::CmdResult::Success)
    {
      // dirty hack to force an update
      sleep(8);
      notified = true;
    }
  }

  if (notified || millis() - last_update > 60000)
  {
    if (getKeyTurnerStateFromLock())
    {
      notified = false;
      publishLockState(nukiBle);
      last_update = millis();
    }
  }
  delay(500);
}