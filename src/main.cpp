/**
 * A BLE client example to connect to Nuki smartlock 2.0
 * author Jeroen
 */

#include <Arduino.h>
#include <NukiLock.h>
#include <NukiConstants.h>
#include <BleScanner.h>

#include <esp_task_wdt.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>

#include <MqttDevice.h>
#include <Preferences.h>
#include "mqttview.h"
#include "utils.h"
#include "config.h"
const uint WATCHDOG_TIMEOUT_S = 30;
const uint32_t PUBLISH_STATE_S = 10 * 60;

uint32_t deviceId = 2020001;
std::string deviceName = "Home";
NukiLock::NukiLock nukiBle(deviceName, deviceId);
BleScanner::Scanner scanner;
NukiLock::KeyTurnerState retrievedKeyTurnerState;
NukiLock::BatteryReport _batteryReport;
std::list<NukiLock::LogEntry> requestedLogEntries;
std::list<NukiLock::KeypadEntry> requestedKeypadEntries;
std::list<NukiLock::AuthorizationEntry> requestedAuthorizationEntries;
std::list<NukiLock::TimeControlEntry> requestedTimeControlEntries;

uint32_t restartCounter = 0;

WiFiClient net;
PubSubClient client(net);
MqttView g_mqttView(&client);
const char *HOMEASSISTANT_STATUS_TOPIC = "homeassistant/status";
const char *HOMEASSISTANT_STATUS_TOPIC_ALT = "ha/status";
MqttDevice mqttDevice(composeClientID().c_str(), "Nuki", "Nuki ESP32 Bridge", "maker_pt");

Preferences preferences;

NukiLock::KeyTurnerState g_state;

bool getKeyTurnerStateFromLock()
{
  Nuki::CmdResult result = nukiBle.requestKeyTurnerState(&retrievedKeyTurnerState);
  if (result == Nuki::CmdResult::Success)
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

void getConfig()
{
  NukiLock::Config config;
  if (nukiBle.requestConfig(&config) == Nuki::CmdResult::Success)
  {
    log_d("Name: %s", config.name);
  }
  else
  {
    log_w("getConfig failed");
  }
}

void connectToMqtt()
{
  log_i("connecting to MQTT...");
  // TODO: add security settings back to mqtt
  // while (!client.connect(mqtt_client, mqtt_user, mqtt_pass))
  for (int i = 0; i < 3 && !client.connect(composeClientID().c_str()); i++)
  {
    Serial.print(".");
    delay(3000);
  }

  client.subscribe(g_mqttView.getLock().getCommandTopic(), 1);

  client.subscribe(HOMEASSISTANT_STATUS_TOPIC);
  client.subscribe(HOMEASSISTANT_STATUS_TOPIC_ALT);

  g_mqttView.publishConfig();
}

void connectToWifi()
{
  log_i("Connecting to wifi...");
  WiFi.begin(wifi_ssid, wifi_pass);
  // TODO: really forever? What if we want to go back to autoconnect?
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(1000);
  }
  log_i("\n Wifi connected!");
}

bool newCommandAvailable = false;
NukiLock::LockAction newCommand = NukiLock::LockAction::Lock;

bool keyTurnerUpdateNotification = false;
class Handler : public Nuki::SmartlockEventHandler
{
public:
  virtual ~Handler(){};
  void notify(NukiLock::EventType eventType)
  {
    if (eventType == Nuki::EventType::KeyTurnerStatusUpdated)
    {
      keyTurnerUpdateNotification = true;
    }
  }
};

Handler handler;

void callback(char *topic, byte *payload, unsigned int length)
{
  log_d("Mqtt msg arrived [%s]", topic);
  if (strcmp(topic, g_mqttView.getLock().getCommandTopic()) == 0)
  {
    if (strncmp((char *)payload, g_mqttView.getLock().getLockCommand(), length) == 0)
    {
      log_d("New Mqtt Command: Lock");
      newCommand = NukiLock::LockAction::Lock;
      newCommandAvailable = true;
    }
    else if (strncmp((char *)payload, g_mqttView.getLock().getUnlockCommand(), length) == 0)
    {
      log_d("New Mqtt Command: Unlock");
      newCommand = NukiLock::LockAction::Unlock;
      newCommandAvailable = true;
    }
    else if (strncmp((char *)payload, g_mqttView.getLock().getOpenCommand(), length) == 0)
    {
      log_d("New Mqtt Command: Open");
      newCommand = NukiLock::LockAction::Unlatch;
      newCommandAvailable = true;
    }
    else
    {
      log_e("Unknown Mqtt command:");
      for (unsigned int i = 0; i < length; i++)
      {
        // TODO: eliminate this somehow
        Serial.print((char)payload[i]);
      }
      Serial.println();
    }
  }

  // publish config when homeassistant comes online and needs the configuration again
  else if (strcmp(topic, HOMEASSISTANT_STATUS_TOPIC) == 0 ||
           strcmp(topic, HOMEASSISTANT_STATUS_TOPIC_ALT) == 0)
  {
    if (strncmp((char *)payload, "online", length) == 0)
    {
      g_mqttView.publishConfig();
      delay(100);
      g_mqttView.publishLockState(nukiBle, restartCounter);
    }
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // on
  Serial.begin(115200);
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true); // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                      // add current thread to WDT watch

  preferences.begin("settings", false);
  restartCounter = preferences.getInt("restartCount", 0);
  log_i("Restart count: %d\n", restartCounter);
  restartCounter++;
  preferences.putInt("restartCount", restartCounter);
  preferences.end();

  WiFi.setHostname(composeClientID().c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
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
    log_i("Start updating %s", type); });

  ArduinoOTA.onEnd([]()
                   { log_i("End Update"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    log_e("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      log_e("Arduino OTA: Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      log_e("Arduino OTA: Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      log_e("Arduino OTA: Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      log_e("Arduino OTA: Receive Failed");
    } else if (error == OTA_END_ERROR) {
      log_e("Arduino OTA: End Failed");
    } });

  ArduinoOTA.begin();

  log_i("Connected to SSID: %s", wifi_ssid);
  log_i("IP address: %s", WiFi.localIP());

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
    digitalWrite(LED_BUILTIN, HIGH); // off
    nukiBle.setEventHandler(&handler);
    getConfig();
    nukiBle.enableLedFlash(false);
  }
  nukiBle.saveSecurityPincode(nuki_pin);
  // nukiBle.unPairNuki();

  // force first update
  keyTurnerUpdateNotification = true;
}
unsigned long last_update = 0;

void loop()
{
  // reset watchdog, important to be called once each loop.
  esp_task_wdt_reset();

  if (WiFi.status() != WL_CONNECTED)
  {
    log_w("WiFi not connected, trying to reconnect, state: %d", WiFi.status());
    WiFi.reconnect();
  }

  if (!client.connected())
  {
    log_w("Mqtt not connected, trying to reconnect");
    connectToMqtt();
  }

  client.loop();
  ArduinoOTA.handle();
  scanner.update();
  if (!nukiBle.isPairedWithLock())
  {
    if (nukiBle.pairNuki() == Nuki::PairingResult::Success)
    {
      digitalWrite(LED_BUILTIN, HIGH); // off
      log_d("paired");
      nukiBle.setEventHandler(&handler);
      getConfig();
    }
  }

  if (newCommandAvailable)
  {
    newCommandAvailable = false;
    for (int i = 0; i < 3 && nukiBle.lockAction(newCommand, deviceId, 0, NULL, 0) != Nuki::CmdResult::Success; i++)
    {
      log_e("Failed to send lock command to lock action: 0x%x", newCommand);
      delay(2000);
    }
  }

  if (keyTurnerUpdateNotification)
  {
    if (getKeyTurnerStateFromLock())
    {
      keyTurnerUpdateNotification = false;
      g_mqttView.publishLockState(nukiBle, restartCounter);
      last_update = millis();
    }
  }

  // we regularly send mqtt messages with the last state to make sure we are alive
  // if (millis() - last_update > PUBLISH_STATE_S * 1000)
  //{
  //  publishLockState(nukiBle);
  //}

  delay(100);
}