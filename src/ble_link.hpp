#pragma once

#include <Arduino.h>
#include <BLECharacteristic.h>
#include "networks.hpp"

void bleLinkSetCharacteristic(BLECharacteristic *c);
void bleLinkOnWrite(const std::string &value);
void bleLinkHandleCommand(const String &value);
void bleLinkPublishTrain(const struct lbj_data &l, const struct rx_info &r, bool isTest = false);
void bleLinkOnConnected();
void bleLinkOnDisconnected();
void bleLinkLoop(); // sync replay
uint32_t bleLinkBufferedCount();
uint32_t bleLinkLatestSeq();
