#pragma once
#include <Arduino.h>
#include <PubSubClient.h>
#include <MqttDevice.h>
#include <NukiLock.h>
#include <NukiConstants.h>
#include "platform.h"
#include "utils.h"

class MqttView
{
public:
    MqttView(PubSubClient *client)
        : m_client(client),
          m_device(composeClientID().c_str(), "Nuki", "Nuki ESP32 Bridge", "maker_pt"),
          m_lock(&m_device, "lock", ""),
          m_battery(&m_device, "battery", "Battery"),
          m_batteryCritical(&m_device, "battery_critical", "Battery Critical"),

          // Diagnostics Elements
          m_bssid(&m_device, "diagnostics_bssid", "BSSID"),
          m_diagnosticsResetButton(&m_device, "diagnostics_reset_btn", "Reset Counters"),
          m_diagnosticsRestartCounter(&m_device, "diagnostics_restart_counter", "Restart Counter"),
          m_diagnosticsWifiDisconnectCounter(&m_device, "diagnostics_wifidisconnect_counter", "WiFi Disconnect Counter"),
          m_diagnosticsMqttDisconnectCounter(&m_device, "diagnostics_mqttdisconnect_counter", "MQTT Disconnect Counter")
    {
        m_device.setSWVersion(VERSION);

        m_lock.setValueTemplate("{{value_json.state}}");

        // we use the state of the lock to publish battery information
        m_battery.setCustomStateTopic(m_lock.getStateTopic());
        m_battery.setValueTemplate("{{value_json.battery}}");
        m_battery.setUnit("%");
        m_battery.setDeviceClass("battery");

        m_batteryCritical.setCustomStateTopic(m_lock.getStateTopic());
        m_batteryCritical.setValueTemplate("{{value_json.battery_critical}}");
        m_batteryCritical.setDeviceClass("battery");

        m_bssid.setCustomStateTopic(m_lock.getStateTopic());
        m_bssid.setValueTemplate("{{value_json.bssid}}");
        m_bssid.setEntityType(EntityCategory::DIAGNOSTIC);
        m_bssid.setIcon("mdi:wifi");

        m_diagnosticsResetButton.setEntityType(EntityCategory::DIAGNOSTIC);
        m_diagnosticsResetButton.setDeviceClass("restart");

        m_diagnosticsRestartCounter.setCustomStateTopic(m_lock.getStateTopic());
        m_diagnosticsRestartCounter.setValueTemplate("{{value_json.restart_counter}}");
        m_diagnosticsRestartCounter.setEntityType(EntityCategory::DIAGNOSTIC);
        m_diagnosticsRestartCounter.setStateClass(MqttSensor::StateClass::TOTAL);
        m_diagnosticsRestartCounter.setIcon("mdi:counter");

        m_diagnosticsWifiDisconnectCounter.setCustomStateTopic(m_lock.getStateTopic());
        m_diagnosticsWifiDisconnectCounter.setValueTemplate("{{value_json.wifi_disconnect_counter}}");
        m_diagnosticsWifiDisconnectCounter.setEntityType(EntityCategory::DIAGNOSTIC);
        m_diagnosticsWifiDisconnectCounter.setStateClass(MqttSensor::StateClass::TOTAL);
        m_diagnosticsWifiDisconnectCounter.setIcon("mdi:counter");

        m_diagnosticsMqttDisconnectCounter.setCustomStateTopic(m_lock.getStateTopic());
        m_diagnosticsMqttDisconnectCounter.setValueTemplate("{{value_json.mqtt_disconnect_counter}}");
        m_diagnosticsMqttDisconnectCounter.setEntityType(EntityCategory::DIAGNOSTIC);
        m_diagnosticsMqttDisconnectCounter.setStateClass(MqttSensor::StateClass::TOTAL);
        m_diagnosticsMqttDisconnectCounter.setIcon("mdi:counter");
    }

    MqttDevice &getDevice()
    {
        return m_device;
    }

    const MqttLock &getLock() const
    {
        return m_lock;
    }

    const MqttButton &getDiagnosticsResetButton() const
    {
        return m_diagnosticsResetButton;
    }

    void publishConfig()
    {
        publishConfig(m_lock);
        publishConfig(m_battery);
        publishConfig(m_batteryCritical);

        // diag elements
        publishConfig(m_diagnosticsResetButton);
        publishConfig(m_diagnosticsRestartCounter);
        publishConfig(m_diagnosticsWifiDisconnectCounter);
        publishConfig(m_diagnosticsMqttDisconnectCounter);
        publishConfig(m_bssid);
    }

    void publishLockState(NukiLock::NukiLock &nuki, NukiLock::LockState lockState, const char* bssid, Config &config)
    {
        char buffer[255];
        const char *lockStateStr;
        switch (lockState)
        {
        case NukiLock::LockState::Locked:
            lockStateStr = m_lock.getLockedState();
            break;
        case NukiLock::LockState::Locking:
            lockStateStr = m_lock.getLockingState();
            break;
        case NukiLock::LockState::Unlocked:
            lockStateStr = m_lock.getUnlockedState();
            break;
        case NukiLock::LockState::Unlocking:
        case NukiLock::LockState::Unlatching:
        case NukiLock::LockState::Unlatched:
        case NukiLock::LockState::UnlockedLnga:
            lockStateStr = m_lock.getUnlockingState();
            break;
        default:
            lockStateStr = m_lock.getUnlockedState();
            break;
        }
        snprintf(buffer, sizeof(buffer), "{\"state\": \"%s\", \"battery\": %d, \"battery_critical\": \"%s\", \"bssid\": \"%s\", \"wifi_disconnect_counter\": %u, \"mqtt_disconnect_counter\": %u, \"restart_counter\": %u}",
                 lockStateStr,
                 nuki.getBatteryPerc(),
                 nuki.isBatteryCritical() ? m_batteryCritical.getOnState() : m_batteryCritical.getOffState(),
                 bssid, 
                 config.wifiDisconnectCounter,
                 config.mqttDisconnectCounter,
                 config.restartCounter);

        m_client->publish(m_lock.getStateTopic(), buffer);
    }

private:
    PubSubClient *m_client;

    MqttDevice m_device;
    MqttLock m_lock;
    MqttSensor m_battery;
    MqttBinarySensor m_batteryCritical;

    // Diagnostics Elements
    MqttButton m_diagnosticsResetButton;
    MqttSensor m_diagnosticsRestartCounter;
    MqttSensor m_diagnosticsWifiDisconnectCounter;
    MqttSensor m_diagnosticsMqttDisconnectCounter;
    MqttSensor m_bssid;

    void publishConfig(MqttEntity &entity)
    {
        String payload = entity.getHomeAssistantConfigPayload();
        char topic[255];
        entity.getHomeAssistantConfigTopic(topic, sizeof(topic));
        if (!m_client->publish(topic, payload.c_str()))
        {
            log_e("Failed to publish config to %s", entity.getStateTopic());
        }
        entity.getHomeAssistantConfigTopicAlt(topic, sizeof(topic));
        if (!m_client->publish(topic, payload.c_str()))
        {
            log_e("Failed to publish config to %s", entity.getStateTopic());
        }
    }
};