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
#include <LittleFS.h>

#include <MqttDevice.h>
#include <Preferences.h>
#include "platform.h"
#include "mqttview.h"
#include "configstorage.h"
#include "utils.h"
#include "config.h"
const uint WATCHDOG_TIMEOUT_S = 30;
const uint32_t PUBLISH_STATE_S = 10 * 60;
const uint32_t WIFI_DISCONNECT_RESTART_S = 60;

uint32_t deviceId = 2020001;
std::string deviceName = "Home";
NukiLock::NukiLock nukiBle(deviceName, deviceId);
BleScanner::Scanner scanner;
NukiLock::KeyTurnerState g_retrievedKeyTurnerState;

Config g_config = {0, 0, 0, "", "", "", 1883};

WiFiClient net;
PubSubClient g_client(net);
MqttView g_mqttView(&g_client);
const char *HOMEASSISTANT_STATUS_TOPIC = "homeassistant/status";
const char *HOMEASSISTANT_STATUS_TOPIC_ALT = "ha/status";
Preferences preferences;

bool g_newCommandAvailable = false;
NukiLock::LockAction g_newCommand = NukiLock::LockAction::Lock;

bool g_keyTurnerUpdateNotification = false;

bool g_wifiConnected = false;
bool g_mqttConnected = false;

bool getKeyTurnerStateFromLock()
{
  Nuki::CmdResult result = nukiBle.requestKeyTurnerState(&g_retrievedKeyTurnerState);
  if (result == Nuki::CmdResult::Success)
  {
    log_d("Bat crit: %d, Bat perc:%d lock state: %d %d:%d:%d",
          nukiBle.isBatteryCritical(), nukiBle.getBatteryPerc(), g_retrievedKeyTurnerState.lockState, g_retrievedKeyTurnerState.currentTimeHour,
          g_retrievedKeyTurnerState.currentTimeMinute, g_retrievedKeyTurnerState.currentTimeSecond);
  }
  else
  {
    log_d("cmd failed: %d", result);
  }
  return result;
}

class NukiNotificationHandler : public Nuki::SmartlockEventHandler
{
public:
  virtual ~NukiNotificationHandler(){};
  void notify(NukiLock::EventType eventType)
  {
    if (eventType == Nuki::EventType::KeyTurnerStatusUpdated)
    {
      g_keyTurnerUpdateNotification = true;
    }
  }
};
NukiNotificationHandler g_nukiNotificationHandler;

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

bool connectToMqtt()
{
  if (g_client.connected())
  {
    return true;
  }

  log_i("Connecting to MQTT...");
  if (strlen(mqtt_user) == 0)
  {
    if (!g_client.connect(composeClientID().c_str()))
    {
      return false;
    }
  }
  else
  {
    if (!g_client.connect(composeClientID().c_str(), mqtt_user, mqtt_pass))
    {
      return false;
    }
  }

  g_client.subscribe(g_mqttView.getLock().getCommandTopic(), 1);
  g_client.subscribe(g_mqttView.getDiagnosticsResetButton().getCommandTopic(), 1);

  g_client.subscribe(HOMEASSISTANT_STATUS_TOPIC);
  g_client.subscribe(HOMEASSISTANT_STATUS_TOPIC_ALT);

  g_mqttView.publishConfig();

  // force an update of the state
  g_keyTurnerUpdateNotification = true;

  return true;
}

bool connectToWifi()
{
  return WiFi.status() == WL_CONNECTED;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  log_d("Mqtt msg arrived [%s]", topic);
  if (strcmp(topic, g_mqttView.getLock().getCommandTopic()) == 0)
  {
    if (strncmp((char *)payload, g_mqttView.getLock().getLockCommand(), length) == 0)
    {
      log_d("New Mqtt Command: Lock");
      g_newCommand = NukiLock::LockAction::Lock;
      g_newCommandAvailable = true;
    }
    else if (strncmp((char *)payload, g_mqttView.getLock().getUnlockCommand(), length) == 0)
    {
      log_d("New Mqtt Command: Unlock");
      g_newCommand = NukiLock::LockAction::Unlock;
      g_newCommandAvailable = true;
    }
    else if (strncmp((char *)payload, g_mqttView.getLock().getOpenCommand(), length) == 0)
    {
      log_d("New Mqtt Command: Open");
      g_newCommand = NukiLock::LockAction::Unlatch;
      g_newCommandAvailable = true;
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
  else if (strcmp(topic, g_mqttView.getDiagnosticsResetButton().getCommandTopic()) == 0)
  {
    bool pressed = strncmp((char *)payload, g_mqttView.getDiagnosticsResetButton().getPressState(), length) == 0;
    g_config.restartCounter = 0;
    g_config.mqttDisconnectCounter = 0;
    g_config.wifiDisconnectCounter = 0;
    saveSettings(g_config);
    NukiLock::KeyTurnerState state;
    // only gets the current state in the lock, does not actively query it
    nukiBle.retrieveKeyTunerState(&state);
    g_mqttView.publishLockState(nukiBle, state.lockState, g_config);
  }

  // publish config when homeassistant comes online and needs the configuration again
  else if (strcmp(topic, HOMEASSISTANT_STATUS_TOPIC) == 0 ||
           strcmp(topic, HOMEASSISTANT_STATUS_TOPIC_ALT) == 0)
  {
    if (strncmp((char *)payload, "online", length) == 0)
    {
      g_mqttView.publishConfig();
      delay(100);
      // force state update
      g_keyTurnerUpdateNotification = true;
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

  if (!LittleFS.begin())
  {
    log_e("Failed to mount file system");
    delay(5000);
    if (!formatLittleFS())
    {
      log_e("Failed to format file system - hardware issues!");
      for (;;)
      {
        delay(100);
      }
    }
  }
  loadSettings(g_config);
  g_config.restartCounter++;
  saveSettings(g_config);

  WiFi.setHostname(composeClientID().c_str());
  WiFi.mode(WIFI_STA);
  WiFi.setAutoConnect(true);
  WiFi.begin(wifi_ssid, wifi_pass);

  // TODO: this is pointless
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
                        {
    // trigger watchdog during update
    esp_task_wdt_reset();
    log_i("Progress: %u%%\r", (progress / (total / 100))); });

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

  g_client.setBufferSize(1024);
  g_client.setServer(mqtt_server, mqtt_port);
  g_client.setCallback(callback);

  log_d("Starting NUKI BLE...");
  scanner.initialize();
  nukiBle.registerBleScanner(&scanner);
  nukiBle.initialize();

  if (nukiBle.isPairedWithLock())
  {
    log_d("paired");
    digitalWrite(LED_BUILTIN, HIGH); // off
    nukiBle.setEventHandler(&g_nukiNotificationHandler);
    getConfig();
    nukiBle.enableLedFlash(false);
  }
  nukiBle.saveSecurityPincode(nuki_pin);
  // nukiBle.unPairNuki();

  // force first update
  g_keyTurnerUpdateNotification = true;
}
unsigned long g_last_update = 0;
unsigned long g_last_wifiConnect = 0;
void loop()
{
  // reset watchdog, important to be called once each loop.
  esp_task_wdt_reset();

  bool wifiConnected = connectToWifi();
  if (!wifiConnected)
  {
    if (g_wifiConnected)
    {
      // we switched to disconnected
      g_config.wifiDisconnectCounter++;
      saveSettings(g_config);
    }
    g_wifiConnected = false;
    g_mqttConnected = false;
    if (millis() - g_last_wifiConnect > WIFI_DISCONNECT_RESTART_S * 1000)
    {
      log_w("Going to do a forced restart because no wifi connection could be made");
      esp_restart();
    }
    delay(1000);
    return;
  }
  g_wifiConnected = true;
  g_last_wifiConnect = millis();
  ArduinoOTA.handle();

  bool mqttConnected = connectToMqtt();
  if (!mqttConnected)
  {
    if (g_mqttConnected)
    {
      // we switched to disconnected
      g_config.mqttDisconnectCounter++;
      saveSettings(g_config);
    }
    g_mqttConnected = false;
    delay(1000);
    return;
  }

  g_client.loop();
  ArduinoOTA.handle();
  scanner.update();
  if (!nukiBle.isPairedWithLock())
  {
    if (nukiBle.pairNuki() == Nuki::PairingResult::Success)
    {
      digitalWrite(LED_BUILTIN, HIGH); // off
      log_d("paired");
      nukiBle.setEventHandler(&g_nukiNotificationHandler);
      getConfig();
    }
  }

  if (g_newCommandAvailable)
  {
    g_newCommandAvailable = false;
    if (g_newCommand == NukiLock::LockAction::Lock)
    {
      g_mqttView.publishLockState(nukiBle, NukiLock::LockState::Locking, g_config);
      delay(100);
    }
    if (g_newCommand == NukiLock::LockAction::Unlock)
    {
      g_mqttView.publishLockState(nukiBle, NukiLock::LockState::Unlocking, g_config);
      delay(100);
    }
    for (int i = 0; i < 3 && nukiBle.lockAction(g_newCommand, deviceId, 0, NULL, 0) != Nuki::CmdResult::Success; i++)
    {
      log_e("Failed to send lock command to lock action: 0x%x", g_newCommand);
      delay(2000);
    }
  }

  if (g_keyTurnerUpdateNotification)
  {
    if (getKeyTurnerStateFromLock())
    {
      NukiLock::KeyTurnerState state;
      // only gets the current state in the lock, does not actively query it
      nukiBle.retrieveKeyTunerState(&state);
      g_keyTurnerUpdateNotification = false;
      g_mqttView.publishLockState(nukiBle, state.lockState, g_config);
      if(state.lockState == NukiLock::LockState::Locking || state.lockState == NukiLock::LockState::Unlocking)
      {
        // force state update because we are transitioning anyway
        g_keyTurnerUpdateNotification = true;
      }
      g_last_update = millis();
    }
  }

  // we regularly send mqtt messages with the last state to make sure we are alive
  // if (millis() - g_last_update > PUBLISH_STATE_S * 1000)
  //{
  //  publishLockState(nukiBle);
  //}

  delay(100);
}