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

#include "utils.h"
#include "config.h"

const uint WATCHDOG_TIMEOUT_S = 30;

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
const char *HOMEASSISTANT_STATUS_TOPIC = "homeassistant/status";
const char *HOMEASSISTANT_STATUS_TOPIC_ALT = "ha/status";
MqttDevice mqttDevice(composeClientID().c_str(), "Nuki", "Nuki ESP32 Bridge", "maker_pt");
MqttLock mqttLock(&mqttDevice, "lock", "Nuki");

MqttSensor mqttBattery(&mqttDevice, "battery", "Nuki Battery");
MqttBinarySensor mqttBatteryCritical(&mqttDevice, "battery_critical", "Nuki Battery Critical");
MqttSensor mqttRestartCounter(&mqttDevice, "nuki_restart_counter", "Nuki Bridge Restart Counter");

Preferences preferences;

void batteryReport()
{
  Nuki::CmdResult result = nukiBle.requestBatteryReport(&_batteryReport);
  if (result == Nuki::CmdResult::Success)
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
  Nuki::CmdResult result = nukiBle.requestKeyTurnerState(&retrievedKeyTurnerState);
  if (result ==  Nuki::CmdResult::Success)
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
  publishConfig(&mqttBatteryCritical);
  publishConfig(&mqttRestartCounter);
}

void publishLockState(NukiLock::NukiLock &nuki)
{
  NukiLock::KeyTurnerState state;
  nukiBle.retrieveKeyTunerState(&state);
  char buffer[255];
  snprintf(buffer, sizeof(buffer), "{\"state\": \"%s\", \"battery\": %d, \"battery_critical\": \"%s\", \"restart_counter\": %u}",
           state.lockState == NukiLock::LockState::Locked ? mqttLock.getLockedState() : mqttLock.getUnlockedState(),
           nuki.getBatteryPerc(),
           nuki.isBatteryCritical() ? mqttBatteryCritical.getOnState() : mqttBatteryCritical.getOffState(),
           restartCounter);

  client.publish(mqttLock.getStateTopic(), buffer);
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

  client.subscribe(mqttLock.getCommandTopic(), 1);

  client.subscribe(HOMEASSISTANT_STATUS_TOPIC);
  client.subscribe(HOMEASSISTANT_STATUS_TOPIC_ALT);

  publishConfig();
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

bool notified = false;
class Handler : public Nuki::SmartlockEventHandler
{
public:
  virtual ~Handler(){};
  void notify(NukiLock::EventType eventType)
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
  log_d("Mqtt msg arrived [%s]", topic);
  if (strcmp(topic, mqttLock.getCommandTopic()) == 0)
  {
    if (strncmp((char *)payload, mqttLock.getLockCommand(), length) == 0)
    {
      log_d("New Mqtt Command: Lock");
      newCommand = NukiLock::LockAction::Lock;
      newCommandAvailable = true;
    }
    else if (strncmp((char *)payload, mqttLock.getUnlockCommand(), length) == 0)
    {
      log_d("New Mqtt Command: Unlock");
      newCommand = NukiLock::LockAction::Unlock;
      newCommandAvailable = true;
    }
    else if (strncmp((char *)payload, mqttLock.getOpenCommand(), length) == 0)
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
      publishConfig();
    }
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // on
  Serial.begin(115200);
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch

  preferences.begin("settings", false);
  restartCounter = preferences.getInt("restartCount", 0);
  log_i("Restart count: %d\n", restartCounter);
  restartCounter++;
  preferences.putInt("restartCount", restartCounter);
  preferences.end();

  mqttLock.setValueTemplate("{{value_json.state}}");

  // we use the state of the lock to publish battery information
  mqttBattery.setCustomStateTopic(mqttLock.getStateTopic());
  mqttBattery.setValueTemplate("{{value_json.battery}}");
  mqttBattery.setUnit("%");
  mqttBattery.setDeviceClass("battery");

  mqttBatteryCritical.setCustomStateTopic(mqttLock.getStateTopic());
  mqttBatteryCritical.setValueTemplate("{{value_json.battery_critical}}");
  mqttBatteryCritical.setDeviceClass("battery");

  mqttRestartCounter.setCustomStateTopic(mqttLock.getStateTopic());
  mqttRestartCounter.setValueTemplate("{{value_json.restart_counter}}");
  mqttRestartCounter.setStateClass(MqttSensor::StateClass::TOTAL_INCREASING);
  mqttRestartCounter.setEntityType(EntityCategory::DIAGNOSTIC);

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
      // force first state update
      notified = true;
    }
  }

  if (newCommandAvailable)
  {
    NukiLock::LockAction action = newCommand;
    newCommandAvailable = false;
    for(int i = 0; i < 3 && nukiBle.lockAction(action, deviceId, 0, NULL, 0) != Nuki::CmdResult::Success; i++)
    {
      log_e("Failed to send lock command to lock action: 0x%x", newCommand);
      sleep(2);
    }
  }

  // TODO verify impact of regular updating the lock on battery life
  if (notified) // || millis() - last_update > 60000)
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