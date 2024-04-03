#pragma once
#include <Arduino.h>
#define VERSION "2024.4.0"
#define CONFIG_FILENAME "/config.txt"

struct Config
{
    uint32_t restartCounter;
    uint32_t wifiDisconnectCounter;
    uint32_t mqttDisconnectCounter;
    char wifiSsid[200];
    char wifiPassword[200];
    char mqttServer[200];
    uint16_t mqttPort;
    char mqttUser[200];
    char mqttPassword[200];
};