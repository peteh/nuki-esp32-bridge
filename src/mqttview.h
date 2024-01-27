#include <Arduino.h>
#include <PubSubClient.h>
#include <MqttDevice.h>
#include <NukiLock.h>
#include <NukiConstants.h>
#include "platform.h"
#include "utils.h"

class MqttView
{
    // apartement door:
    //   doorman-[name]/apartment/bell/state -> on/off
    //   doorman-[name]/apartment/bell/pattern/state -> on/off
    // entry door:
    //   doorman-[name]/entry/bell/state -> on/off
    //   doorman-[name]/entry/bell/pattern/state -> on/off
    //   doorman-[name]/entry/opener/cmd
    //   doorman-[name]/partymode/state -> on/off

    // commands
    // 0x1100 door opener if the handset is not lifted up
    // 0x1180 door opener if the handset is lifted up

    // 0x1B8F9A41 own door bell at the flat door
    // 0x0B8F9A80 own door bell at the main door

public:
    MqttView(PubSubClient *client)
        : m_client(client),
          m_device(composeClientID().c_str(), "Nuki", "Nuki ESP32 Bridge", "maker_pt"),
          mqttLock(&m_device, "lock", ""),
          mqttBattery(&m_device, "battery", "Battery"),
          mqttBatteryCritical(&m_device, "battery_critical", "Battery Critical"),
          mqttRestartCounter(&m_device, "restart_counter", "Restart Counter")
    {
        m_device.setSWVersion(VERSION);

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
        mqttRestartCounter.setStateClass(MqttSensor::StateClass::TOTAL);
        mqttRestartCounter.setEntityType(EntityCategory::DIAGNOSTIC);
    }

    MqttDevice &getDevice()
    {
        return m_device;
    }

    const MqttLock &getLock() const
    {
        return mqttLock;
    }

    void publishOnOffEdgeSwitch(MqttSwitch &entity)
    {
        publishMqttState(entity, entity.getOnState());
        delay(1000);
        publishMqttState(entity, entity.getOffState());
    }

    void publishOnOffEdgeSiren(MqttSiren &entity)
    {
        publishMqttState(entity, entity.getOnState());
        delay(1000);
        publishMqttState(entity, entity.getOffState());
    }

    void publishOnOffEdgeBinary(MqttBinarySensor &entity)
    {
        publishMqttState(entity, entity.getOnState());
        delay(1000);
        publishMqttState(entity, entity.getOffState());
    }

    void publishConfig()
    {
        publishConfig(mqttLock);
        publishConfig(mqttBattery);
        publishConfig(mqttBatteryCritical);
        publishConfig(mqttRestartCounter);

        delay(1000);
        // TODO: publish all initial states
    }

    void publishLockState(NukiLock::NukiLock &nuki, uint32_t restartCounter)
    {
        NukiLock::KeyTurnerState state;
        nuki.retrieveKeyTunerState(&state);
        char buffer[255];
        snprintf(buffer, sizeof(buffer), "{\"state\": \"%s\", \"battery\": %d, \"battery_critical\": \"%s\", \"restart_counter\": %u}",
                 state.lockState == NukiLock::LockState::Locked ? mqttLock.getLockedState() : mqttLock.getUnlockedState(),
                 nuki.getBatteryPerc(),
                 nuki.isBatteryCritical() ? mqttBatteryCritical.getOnState() : mqttBatteryCritical.getOffState(),
                 restartCounter);

        m_client->publish(mqttLock.getStateTopic(), buffer);
    }

private:
    PubSubClient *m_client;

    MqttDevice m_device;
    MqttLock mqttLock;
    MqttSensor mqttBattery;
    MqttBinarySensor mqttBatteryCritical;
    MqttSensor mqttRestartCounter;

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

    void publishMqttConfigState(MqttEntity &entity, const uint32_t value)
    {
        char state[9];
        snprintf(state, sizeof(state), "%08x", value);
        if (!m_client->publish(entity.getStateTopic(), state))
        {
            log_e("Failed to publish state to %s", entity.getStateTopic());
        }
    }

    void publishMqttCounterState(MqttEntity &entity, const uint32_t value)
    {
        char state[9];
        snprintf(state, sizeof(state), "%u", value);
        m_client->publish(entity.getStateTopic(), state);
    }

    void publishMqttState(MqttEntity &entity, const char *state)
    {
        if (!m_client->publish(entity.getStateTopic(), state))
        {
            log_e("Failed to publish state to %s", entity.getStateTopic());
        }
    }
};