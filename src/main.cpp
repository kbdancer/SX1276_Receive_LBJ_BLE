/*
   SX1276 LBJ Message Receive Project
   Migrated by FLN1021 on Sep 2023.
   Modified from RadioLib Pager (POCSAG) Receive Example.
   Original file:
   https://github.com/jgromes/RadioLib/blob/master/examples/Pager/Pager_Receive/Pager_Receive.ino
*/
/*
   RadioLib Pager (POCSAG) Receive Example

   This example shows how to receive FSK packets without using
   SX127x packet engine.

   This example receives POCSAG messages using SX1278's
   FSK modem in direct mode.

   Other modules that can be used to receive POCSAG:
    - SX127x/RFM9x
    - RF69
    - SX1231
    - CC1101
    - Si443x/RFM2x

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
*/
#pragma execution_character_set("utf-8")

// include libraries
#include <RadioLib.h>
#include "coredump.h"
#include "customfont.h"
#include <esp_task_wdt.h>
#include <atomic>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLECharacteristic.h>
#include <ArduinoJson.h>
#include "ble_link.hpp"

#define WDT_TIMEOUT 20 // sec
// #define WDT_RST_PERIOD 4000 // ms
#define FD_TASK_STACK_SIZE 8000 // BLE JSON needs more stack
#define FD_TASK_TIMEOUT 750 // ms
#define FD_TASK_ATTEMPTS 3
#define LED_ON_TIME 200 // ms
//region Variables
SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DIO0_PIN, RADIO_RST_PIN, RADIO_DIO1_PIN);
// receiving packets requires connection
// to the module direct output pin
const int pin = RADIO_BUSY_PIN;
float rssi_cache = 0;
// float fer = 0;
float fers[32]{};
float actual_frequency = 0;
float freq_last = 0;
float car_fer_last = 0;
// create Pager client instance using the FSK module
PagerClient pager(&radio);
// timers
uint64_t format_task_timer = 0;
uint64_t runtime_timer = 0;
uint64_t screen_timer = 0;
uint64_t led_timer = 0;
uint64_t timer4 = 0;
// uint64_t wdt_timer = 0;
uint64_t prb_timer = 0;
uint64_t car_timer = 0;
uint32_t prb_count = 0;
uint32_t car_count = 0;
float ppm = INITIAL_PPM;

inline float actualFreq(float bias) {
    actual_frequency = (float) ((TARGET_FREQ * bias) / 1e6 + TARGET_FREQ);
    return actual_frequency;
}

bool freq_correction = AFC_ENABLE;
// bool bandwidth_altered = false;
bool exec_init_f80 = false;
// bool agc_triggered = false;
bool low_volt_warned = false;
bool oled_off = false;
bool have_cd = false;
bool ble_enabled = true;
SD_LOG sd1;

#define SERVICE_UUID           "0000FFE0-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_UUID_TX "0000FFE1-0000-1000-8000-00805F9B34FB"
#define BLE_DEVICE_NAME        "LBJReceiver"

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
std::atomic<bool> deviceConnected{false};
bool oldDeviceConnected = false;
uint64_t ble_timer = 0;

class SafeServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *server) override {
        (void)server;
        deviceConnected.store(true, std::memory_order_release);
        Serial.println("[BLE] Device is connected");
        ble_timer = millis64();
    }

    void onDisconnect(BLEServer *server) override {
        (void)server;
        deviceConnected.store(false, std::memory_order_release);
        Serial.println("[BLE] Device is disconnected");
    }
};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        bleLinkOnWrite(pCharacteristic->getValue());
    }
};

void initBLE();
void handleBleConnections();
struct rx_info rxInfo;
struct data_bond *db = nullptr;
// PagerClient::pocsag_data *pd = nullptr;
//endregion

//region Functions
void formatDataTask(void *pVoid);

void simpleFormatTask();

void initFmtVars();

void handleSerialInput();

void handleCarrier();

void handlePreamble();

void revertFrequency();


TaskHandle_t task_fd;
enum task_states {
    TASK_INIT = 0,
    TASK_CREATED = 1,
    TASK_RUNNING = 2,
    TASK_DONE = 3,
    TASK_TERMINATED = 4,
    TASK_CREATE_FAILED = 5,
    TASK_RUNNING_SCREEN = 6
};

task_states fd_state;

#ifdef HAS_DISPLAY

// Bottom-bar "L" used to mean "Local SD" (old D/L/N: SD+Net / SD-only / Net-only).
static uint64_t sd_free_cache_bytes = 0;
static uint64_t sd_free_cache_ms = 0;

static void refreshSdFreeCache(bool force = false) {
    if (!have_sd) {
        sd_free_cache_bytes = 0;
        return;
    }
    if (!force && sd_free_cache_ms != 0 && millis64() - sd_free_cache_ms < 30000) {
        return;
    }
    uint64_t total = SD.totalBytes();
    uint64_t used = SD.usedBytes();
    sd_free_cache_bytes = (total > used) ? (total - used) : 0;
    sd_free_cache_ms = millis64();
}

static void formatSdFree(char *buf, size_t n, uint64_t freeB) {
    if (freeB >= (1024ULL * 1024 * 1024)) {
        snprintf(buf, n, "%.1fG", freeB / (1024.0 * 1024.0 * 1024.0));
    } else if (freeB >= (1024ULL * 1024)) {
        snprintf(buf, n, "%lluM", (unsigned long long) (freeB / (1024ULL * 1024)));
    } else {
        snprintf(buf, n, "%lluK", (unsigned long long) (freeB / 1024ULL));
    }
}

static void drawTopStatusBar() {
    char buffer[32];
    u8g2->setFont(u8g2_font_squeezed_b7_tr);
    if (!getLocalTime(&time_info, 0)) {
        u8g2->drawStr(0, 7, "NO SNTP");
    } else {
        sprintf(buffer, "%d-%02d-%02d %02d:%02d", time_info.tm_year + 1900, time_info.tm_mon + 1, time_info.tm_mday,
                time_info.tm_hour, time_info.tm_min);
        u8g2->drawStr(0, 7, buffer);
    }
#ifdef HAS_RTC
    sprintf(buffer, "%dC", (int) rtc.getTemperature());
    u8g2->drawStr(72, 7, buffer);
#endif
    if (have_sd) {
        refreshSdFreeCache(false);
        formatSdFree(buffer, sizeof(buffer), sd_free_cache_bytes);
        int w = u8g2->getStrWidth(buffer);
        u8g2->drawStr(128 - w, 7, buffer);
    }
}

void pword(const char *msg, int xloc, int yloc) {
    int dspW = u8g2->getDisplayWidth();
    int strW = 0;
    char glyph[2];
    glyph[1] = 0;
    for (const char *ptr = msg; *ptr; *ptr++) {
        glyph[0] = *ptr;
        strW += u8g2->getStrWidth(glyph);
        ++strW;
        if (xloc + strW > dspW) {
            int sxloc = xloc;
            while (msg < ptr) {
                glyph[0] = *msg++;
                xloc += u8g2->drawStr(xloc, yloc, glyph);
            }
            strW -= xloc - sxloc;
            yloc += u8g2->getMaxCharHeight();
            xloc = 0;
        }
    }
    while (*msg) {
        glyph[0] = *msg++;
        xloc += u8g2->drawStr(xloc, yloc, glyph);
    }
}

void showInitComp() {
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_squeezed_b7_tr);
    // bottom (0,56,128,8)
    u8g2->drawStr(0, 64, "BLE");
    char buffer[32];
    sprintf(buffer, "%2u", ets_get_cpu_frequency() / 10);
    u8g2->drawStr(96, 64, buffer);
    sprintf(buffer, "%1.2f", battery.readVoltage() * 2);
    u8g2->drawStr(108, 64, buffer);
    // top (0,0,128,8) — time left, SD free right
    refreshSdFreeCache(true);
    drawTopStatusBar();
    u8g2->sendBuffer();
}

void updateInfo() {
    // update top: time / NO SNTP left, SD free capacity right (if present)
    u8g2->setDrawColor(0);
    u8g2->drawBox(0, 0, 128, 8);
    u8g2->setDrawColor(1);
    drawTopStatusBar();
    // update bottom
    char buffer[32];
    u8g2->setDrawColor(0);
    u8g2->drawBox(0, 56, 128, 8);
    u8g2->setDrawColor(1);
    u8g2->setFont(u8g2_font_squeezed_b7_tr);
    u8g2->drawStr(0, 64, "BLE");
    sprintf(buffer, "%.1f", getBias(actual_frequency));
    u8g2->drawStr(73, 64, buffer);
    sprintf(buffer, "%2u", ets_get_cpu_frequency() / 10);
    u8g2->drawStr(96, 64, buffer);
    voltage = battery.readVoltage() * 2;
    sprintf(buffer, "%1.2f", voltage); // todo: Implement average voltage reading.
    if (voltage < 3.15 && !low_volt_warned) {
        Serial.printf("Warning! Low Voltage detected, %1.2fV\n", voltage);
        sd1.append("低压警告，电池电压%1.2fV\n", voltage);
        low_volt_warned = true;
    }
    u8g2->drawStr(108, 64, buffer);
    u8g2->sendBuffer();
}

void showSTR(const String &str) {
    u8g2->setDrawColor(0);
    u8g2->drawBox(0, 8, 128, 48);
    u8g2->setDrawColor(1);
    // u8g2->setFont(FONT_12_GB2312);
    u8g2->setFont(u8g2_font_squeezed_b7_tr);
    pword(str.c_str(), 0, 19);
    u8g2->sendBuffer();
}

void showLBJ0(const struct lbj_data &l) {
    // box y 9->55
    char buffer[128];
    u8g2->setDrawColor(0);
    u8g2->drawBox(0, 8, 128, 48);
    u8g2->setDrawColor(1);
    u8g2->setFont(u8g2_font_wqy15_t_custom);
    u8g2->setCursor(0, 21);
    u8g2->printf("车  次");
    u8g2->setFont(u8g2_font_spleen8x16_mu);
    u8g2->setCursor(50, u8g2->getCursorY());
    u8g2->printf("%s", l.train);
    u8g2->setFont(u8g2_font_wqy15_t_custom);
    u8g2->setCursor(u8g2->getCursorX() + 6, u8g2->getCursorY());
    if (l.direction == FUNCTION_UP) {
        u8g2->printf("上行");
    } else if (l.direction == FUNCTION_DOWN)
        u8g2->printf("下行");
    else
        u8g2->printf("%d", l.direction);
    u8g2->setCursor(0, 37);
    u8g2->printf("速  度");
    u8g2->setCursor(50, u8g2->getCursorY());
    u8g2->setFont(u8g2_font_spleen8x16_mu);
    u8g2->printf(" %s ", l.speed);
    u8g2->setCursor(u8g2->getCursorX() + 7, u8g2->getCursorY());
    u8g2->setFont(u8g2_font_profont15_mr);
    u8g2->printf("KM/H");
    u8g2->setFont(u8g2_font_wqy15_t_custom);
    // sprintf(buffer, "公里标 %s KM", l.position);
    u8g2->setCursor(0, 53);
    u8g2->printf("公里标");
    u8g2->setCursor(50, u8g2->getCursorY());
    u8g2->setFont(u8g2_font_spleen8x16_mu);
    u8g2->printf("%s ", l.position);
    u8g2->setCursor(u8g2->getCursorX() + 4, u8g2->getCursorY());
    u8g2->setFont(u8g2_font_profont15_mr);
    u8g2->printf("KM");
    // draw RSSI
    u8g2->setDrawColor(0);
    u8g2->drawBox(98, 0, 30, 8);
    u8g2->setDrawColor(1);
    u8g2->setFont(u8g2_font_squeezed_b7_tr);
    sprintf(buffer, "%3.1f", rxInfo.rssi);
    u8g2->drawStr(99, 7, buffer);
    u8g2->sendBuffer();
}

void showLBJ1(const struct lbj_data &l) {
    char buffer[128];
    u8g2->setDrawColor(0);
    u8g2->drawBox(0, 8, 128, 48);
    u8g2->setDrawColor(1);
    u8g2->setFont(FONT_12_GB2312);
    // line 1
    u8g2->setCursor(0, 19);
    u8g2->printf("车:");
    u8g2->setCursor(u8g2->getCursorX() + 1, u8g2->getCursorY());
    u8g2->setFont(u8g2_font_profont12_custom_tf);
    for (int i = 0, c = 0; i < 6; i++) {
        if (i == 5) {
            buffer[c] = 0;
            break;
        }
        if (l.train[i] == ' ')
            continue;
        buffer[c] = l.train[i];
        ++c;
    }
    u8g2->printf("%s%s", l.lbj_class, buffer);
    u8g2->setFont(FONT_12_GB2312);
    u8g2->setCursor(68, 19);
    u8g2->printf("速:");
    u8g2->setCursor(u8g2->getCursorX() + 2, u8g2->getCursorY());
    u8g2->setFont(u8g2_font_profont12_custom_tf);
    u8g2->printf("%s", l.speed);
    u8g2->setCursor(u8g2->getCursorX(), u8g2->getCursorY());
    u8g2->printf("KM/H");
    u8g2->setFont(FONT_12_GB2312);
    // line 2
    u8g2->setCursor(0, 31);
    u8g2->printf("线:");
    u8g2->setCursor(u8g2->getCursorX() + 2, u8g2->getCursorY());
    u8g2->printf("%s", l.route_utf8);
    u8g2->drawBox(67, 21, 13, 12);
    u8g2->setDrawColor(0);
    if (l.direction == FUNCTION_UP)
        u8g2->drawUTF8(68, 31, "上");
    else if (l.direction == FUNCTION_DOWN)
        u8g2->drawUTF8(68, 31, "下");
    else {
        sprintf(buffer, "%d", l.direction);
        u8g2->drawStr(71, 31, buffer);
    }
    u8g2->setDrawColor(1);
    u8g2->setCursor(84, 31);
    u8g2->setFont(u8g2_font_profont12_custom_tf);
    u8g2->printf("%s", l.position);
    u8g2->setCursor(u8g2->getCursorX(), u8g2->getCursorY());
    u8g2->printf("K");
    u8g2->setFont(FONT_12_GB2312);
    // line 3
    u8g2->setCursor(0, 43);
    u8g2->printf("号:");
    u8g2->setCursor(u8g2->getCursorX() + 1, u8g2->getCursorY());
    u8g2->setFont(u8g2_font_profont12_custom_tf);
    u8g2->printf("%s", l.loco);
    if (String(l.loco) != "<NUL>" && l.info2_hex.length() > 14 && l.info2_hex[12] == '3') {
        if (l.info2_hex[13] == '1')
            u8g2->printf("A");
        else if (l.info2_hex[13] == '2')
            u8g2->printf("B");
    }
    u8g2->setFont(FONT_12_GB2312);
    if (l.loco_type.length())
        u8g2->drawUTF8(72, 43, l.loco_type.c_str());
    // line 4
    String pos;
    if (l.pos_lat_deg[1] && l.pos_lat_min[1]) {
        sprintf(buffer, "%s°%s'", l.pos_lat_deg, l.pos_lat_min);
        pos += String(buffer);
    } else {
        sprintf(buffer, "%s ", l.pos_lat);
        pos += String(buffer);
    }
    if (l.pos_lon_deg[1] && l.pos_lon_min[1]) {
        sprintf(buffer, "%s°%s'", l.pos_lon_deg, l.pos_lon_min);
        pos += String(buffer);
    } else {
        sprintf(buffer, "%s ", l.pos_lon);
        pos += String(buffer);
    }
//    sprintf(buffer,"%s°%s'%s°%s'",l.pos_lat_deg,l.pos_lat_min,l.pos_lon_deg,l.pos_lon_min);
    u8g2->setFont(u8g2_font_profont12_custom_tf);
    u8g2->drawUTF8(0, 54, pos.c_str());
    // draw RSSI
    u8g2->setDrawColor(0);
    u8g2->drawBox(98, 0, 30, 8);
    u8g2->setDrawColor(1);
    u8g2->setFont(u8g2_font_squeezed_b7_tr);
    sprintf(buffer, "%3.1f", rxInfo.rssi);
    u8g2->drawStr(99, 7, buffer);
    u8g2->sendBuffer();
}

void showLBJ2(const struct lbj_data &l) {
    char buffer[128];
    u8g2->setDrawColor(0);
    u8g2->drawBox(0, 8, 128, 48);
    u8g2->setDrawColor(1);
    u8g2->setFont(u8g2_font_wqy15_t_custom);
    u8g2->setCursor(0, 23);
    u8g2->printf("当前时间");
    u8g2->setFont(u8g2_font_spleen8x16_mu);
    u8g2->setCursor(u8g2->getCursorX() + 3, u8g2->getCursorY() - 1);
    u8g2->printf("%s ", l.time);
    // draw RSSI
    u8g2->setDrawColor(0);
    u8g2->drawBox(98, 0, 30, 8);
    u8g2->setDrawColor(1);
    u8g2->setFont(u8g2_font_squeezed_b7_tr);
    sprintf(buffer, "%3.1f", rxInfo.rssi);
    u8g2->drawStr(99, 7, buffer);
    u8g2->sendBuffer();
}

#endif

void dualPrintf(bool time_stamp, const char *format, ...) {
    (void)time_stamp;
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.print(buffer);
}

void dualPrint(const char *fmt) {
    Serial.print(fmt);
}

void dualPrintln(const char *fmt) {
    Serial.println(fmt);
}


String printResetReason(esp_reset_reason_t reset) {
    String str;
    switch (reset) {
        case ESP_RST_UNKNOWN:
            str = "ESP_RST_UNKNOWN, Reset reason can not be determined";
            break;
        case ESP_RST_POWERON:
            str = "ESP_RST_POWERON, Reset due to power-on event";
            break;
        case ESP_RST_EXT:
            str = "ESP_RST_EXT, Reset by external pin (not applicable for ESP32)";
            break;
        case ESP_RST_SW:
            str = "ESP_RST_SW, Software reset via esp_restart";
            break;
        case ESP_RST_PANIC:
            str = "ESP_RST_PANIC, Software reset due to exception/panic";
            break;
        case ESP_RST_INT_WDT:
            str = "ESP_RST_INT_WDT, Reset (software or hardware) due to interrupt watchdog";
            break;
        case ESP_RST_TASK_WDT:
            str = "ESP_RST_TASK_WDT, Reset due to task watchdog";
            break;
        case ESP_RST_WDT:
            str = "ESP_RST_WDT, Reset due to other watchdogs";
            break;
        case ESP_RST_DEEPSLEEP:
            str = "ESP_RST_DEEPSLEEP, Reset after exiting deep sleep mode";
            break;
        case ESP_RST_BROWNOUT:
            str = "ESP_RST_BROWNOUT, Brownout reset (software or hardware)";
            break;
        case ESP_RST_SDIO:
            str = "ESP_RST_SDIO, Reset over SDIO";
            break;
    }
    return str;
}

void LBJTEST() {
    PagerClient::pocsag_data pocdat[16];
    pocdat[0].str = "37012";
    pocdat[0].addr = 1234000;
    pocdat[0].func = 1;
    pocdat[0].is_empty = false;
    pocdat[0].len = 15;
    pocdat[1].str = "30479100018530U)*9UU*6 (-(202011719040139058291000";
    pocdat[1].addr = 1234002;
    pocdat[1].func = 1;
    pocdat[1].is_empty = false;
    pocdat[1].len = 0;
//    Serial.println("[LBJ] 测试输出 机车编号 位置 XX°XX′XX″ ");
//    dualPrintf(false,"[LBJ] 测试输出 机车编号 位置 XX°XX′XX″ \n");
    struct lbj_data lbj;

    // db = new data_bond;
    // db->pocsagData[0].addr = 1234000;
    // db->pocsagData[0].str = "37012  15  1504";
    // db->pocsagData[0].func = 1;
    // db->pocsagData[0].is_empty = false;
    // db->pocsagData[0].len = 15;
    // db->pocsagData[1].str = "20202350018530U)*9UU*6 (-(202011719040139058291000";
    // db->pocsagData[1].addr = 1234002;
    // db->pocsagData[1].func = 1;
    // db->pocsagData[1].is_empty = false;
    // db->pocsagData[1].len = 0;
    readDataLBJ(pocdat, &lbj);
    printDataSerial(pocdat, lbj, rxInfo);
    // // appendDataLog(pocdat, lbj, rxInfo);
    // simpleFormatTask();
    // rxInfo.rssi = 0;
    // rxInfo.fer = 0;
    // delete db;
}

int initPager() {// initialize SX1276 with default settings

    int state = radio.beginFSK(434.0, 4.8, 5.0, 12.5);
    RADIOLIB_ASSERT(state)

    state = radio.setGain(1);
    RADIOLIB_ASSERT(state)

    // state = radio.setRxBandwidth(25);
    // RADIOLIB_ASSERT(state)
    // initialize Pager client
    // Serial.print(F("[Pager] Initializing ... "));
    // base (center) frequency: 821.2375 MHz + ppm
    // speed:                   1200 bps
    state = pager.begin(actualFreq(ppm), 1200, false, 2500);
    RADIOLIB_ASSERT(state)

    freq_last = actual_frequency;

    // start receiving POCSAG messages
    // Serial.print(F("[Pager] Starting to listen ... "));
    // address of this "pager": 12340XX
    state = pager.startReceive(pin, 1234000, 0xFFFF0);
    //TODO Enhancement: try to keep a open address filter, we might find something unknown.
    RADIOLIB_ASSERT(state)

    // state = radio.setFrequency(actual_freq);
    // RADIOLIB_ASSERT(state)

    return (state);
}
//endregion


void initBLE() {
    Serial.println("[BLE] Initializing BLE...");

    if (BLEDevice::getInitialized()) {
        Serial.println("[BLE] Warning BLE already initialized, deinitializing first");
        BLEDevice::deinit(true);
        delay(500);
    }

    pServer = nullptr;
    pTxCharacteristic = nullptr;

    BLEDevice::init(BLE_DEVICE_NAME);
    pServer = BLEDevice::createServer();
    if (pServer == nullptr) {
        Serial.println("[BLE] Error unable to create server");
        return;
    }
    pServer->setCallbacks(new SafeServerCallbacks());

    BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    if (pTxCharacteristic == nullptr) {
        Serial.println("[BLE] Error unable to create TX characteristic");
        return;
    }

    BLE2902 *p2902Descriptor = new BLE2902();
    p2902Descriptor->setNotifications(true);
    pTxCharacteristic->addDescriptor(p2902Descriptor);
    pTxCharacteristic->setCallbacks(new MyCallbacks());
    pTxCharacteristic->setValue("LBJ Ready");
    bleLinkSetCharacteristic(pTxCharacteristic);

    pService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    pAdvertising->start();

    Serial.printf("[BLE] Device name: %s\n", BLE_DEVICE_NAME);
    Serial.printf("[BLE] Service UUID: %s\n", SERVICE_UUID);
    Serial.println("[BLE] Advertising started");
}

void handleBleConnections() {
    if (pServer == nullptr || !ble_enabled) {
        return;
    }

    bool connected = deviceConnected.load(std::memory_order_acquire);
    if (!connected && oldDeviceConnected) {
        bleLinkOnDisconnected();
        Serial.println("[BLE] Device disconnected, restart advertising");
        delay(100);
        BLEAdvertising *pAdvertising = pServer->getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        pAdvertising->start();
        oldDeviceConnected = connected;
    }
    if (connected && !oldDeviceConnected) {
        oldDeviceConnected = connected;
        if (pTxCharacteristic != nullptr) {
            pTxCharacteristic->setValue("LBJ Train Warning Ready");
            pTxCharacteristic->notify();
        }
        bleLinkOnConnected();
        Serial.println("[BLE] New device connected successfully");
    }
}

// SETUP
void setup() {
    esp_core_dump_init();
    runtime_timer = millis64();
    esp_reset_reason_t reset_reason = esp_reset_reason();
    initBoard();
    sd1.setFS(SD);
    delay(150);

    // Configure time sync.
    sntp_set_time_sync_notification_cb(timeAvailable);
    sntp_servermode_dhcp(1);
    configTzTime(time_zone, ntpServer1, ntpServer2);

#ifdef HAS_RTC
    // rtc.begin();
    // rtc.getDateTime(time_info);
    time_info = rtcLibtoC(rtc.now());
    Serial.println(&time_info, "[eRTC] RTC Time %Y-%m-%d %H:%M:%S ");
    timeSync(time_info); // sync system time from rtc
    Serial.printf("SYS Time %s\n", fmtime(time_info));
#endif

    Serial.printf("RST: %s\n", printResetReason(reset_reason).c_str());
    if (have_sd) {
        sd1.begin("/LOGTEST");
        sd1.beginCSV("/CSVTEST");
        sd1.append("电池电压 %1.2fV\n", battery.readVoltage() * 2);
        sd1.append(2, "调试等级 %d\n", LOG_VERBOSITY);
        sd1.append("复位原因 %s\n", printResetReason(reset_reason).c_str());
#ifdef HAS_RTC
        sd1.append("RTC时间 %d-%02d-%02d %02d:%02d:%02d\n", time_info.tm_year + 1900, time_info.tm_mon + 1,
                   time_info.tm_mday, time_info.tm_hour, time_info.tm_min, time_info.tm_sec);
#endif
    }

    // Process core dump.
    readCoreDump();

    if (u8g2) {
        showInitComp();
        u8g2->setFont(FONT_12_GB2312);
        u8g2->setCursor(0, 52);
        u8g2->println("Initializing...");
        u8g2->sendBuffer();
    }

    // Initialize SX1276
    dualPrint("[SX1276] Initializing ... ");
    int state = initPager();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success."));
        Serial.printf("[SX1276] Actual Frequency %f MHz, ppm %.1f\n", actualFreq(ppm), ppm);
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    // start thread watchdog
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(nullptr);
    // wdt_timer = millis64();

    digitalWrite(BOARD_LED, LED_OFF);
    Serial.printf("Booting time %llu ms\n", millis64() - runtime_timer);
    sd1.append("启动用时 %llu ms\n", millis64() - runtime_timer);
    runtime_timer = 0;

    if (u8g2) {
        u8g2->setDrawColor(0);
        u8g2->drawBox(0, 42, 128, 14);
        u8g2->setDrawColor(1);
        u8g2->drawStr(0, 52, "Listening...");
        u8g2->sendBuffer();
        Serial.printf("Mem left: %d Bytes\n", esp_get_free_heap_size());
    }

    // test stuff
    // LBJTEST();
    // auto *test = new uint8_t[32];
    // Serial.printf("[D] test addr %p\n",test);
    // delete[] test;
    // Serial.printf("[D] test addr %p\n",test);
    // test = nullptr;
    // Serial.printf("[D] test addr %p\n",test);
    // delete[] test;
    // delete[] test;
//     Serial.printf("CPU FREQ %d MHz\n",ets_get_cpu_frequency());

    if (ble_enabled) {
        initBLE();
    }

}

// Loop functions
void handleSync() {
    if (pager.gotSyncState()) {
        // if (!bandwidth_altered) {
        //     int16_t state = radio.swapRxBandwidth(12.5);
        //     Serial.printf("[D] Channelize, code %d\n",state);
        //     radio.restartReceive(true);
        //     bandwidth_altered = true;
        // }
//        sd1.append("[PGR][DEBUG] SYNC DETECTED.\n");
        if (rxInfo.cnt < 5 && (rxInfo.timer == 0 || esp_timer_get_time() - rxInfo.timer < 11000)) {
            float rssi = radio.getRSSI(false, true);
            rxInfo.timer = esp_timer_get_time();
            // rxInfo.rssi += rssi;
            rssi_cache += rssi;
            rxInfo.cnt++;
            Serial.printf("[D] RXI %.2f\n", rssi_cache / (float) rxInfo.cnt);
        }
        if (rxInfo.fer == 0)
            rxInfo.fer = radio.getFrequencyError();
    }
}



// LOOP
void loop() {
    // reset watchdog
    esp_task_wdt_reset();
    // if (millis64() - wdt_timer >= WDT_RST_PERIOD) {
    //     uint64_t t = esp_timer_get_time() ;
    //     auto r = esp_task_wdt_reset();
    //     t = esp_timer_get_time() - t;
    //     wdt_timer = millis64();
    //     Serial.printf("WDT_RST %d [%llu]\n",r,t);
    // }

    // freqCorrection();
    // Handle carrier timout.
    if (car_timer != 0 && millis64() - car_timer > 700 && prb_timer == 0 && rxInfo.timer == 0) {
        car_count = 0;
        revertFrequency();
        car_fer_last = 0;
        car_timer = 0;
        Serial.println("[D] CARRIER TIMEOUT.");
    }

    // Handle preamble timeout.
    if (prb_timer != 0 && millis64() - prb_timer > 600 && rxInfo.timer == 0) {
        prb_count = 0;
        revertFrequency();
        for (auto &i: fers) {
            i = 0;
        }
        prb_timer = 0;
        Serial.println("[D] PREAMBLE TIMEOUT.");
    }

    // if task complete, de-initialize
    if (fd_state == TASK_DONE) {
        if (task_fd != nullptr) {
            // Serial.printf("[D] NULLPTR EXCE [%llu]\n", millis64() - format_task_timer);
            vTaskDelete(task_fd);
            // Serial.printf("[D] TASK DEL [%llu]\n", millis64() - format_task_timer);
            task_fd = nullptr;
        }
        // Serial.printf("[D] NULLPTR [%llu]\n", millis64() - format_task_timer);
        initFmtVars();
        // Serial.printf("[D] INIT VARS [%llu]\n", millis64() - format_task_timer);
        // digitalWrite(BOARD_LED, LED_OFF);
        // Serial.printf("[D] LED LOW [%llu]\n", millis64() - format_task_timer);
        // changeCpuFreq(240);
        // Serial.printf("[D] FREQ CHANGED [%llu]\n", millis64() - format_task_timer);
        fd_state = TASK_INIT;
        format_task_timer = 0;
    } else if (fd_state == TASK_CREATE_FAILED) { // Handle create failure.
        initFmtVars();
        // changeCpuFreq(240);
        format_task_timer = 0;
        fd_state = TASK_INIT;
    }

    if (millis64() - led_timer > LED_ON_TIME && led_timer != 0 && fd_state == TASK_INIT) {
        digitalWrite(BOARD_LED, LED_OFF);
        led_timer = 0;
        changeCpuFreq(240);
    }


    handleSerialInput();
    handleBleConnections();
    bleLinkLoop();

    if (millis64() > 60000 && format_task_timer == 0 && !exec_init_f80) {
        setCpuFrequencyMhz(80);
        exec_init_f80 = true;
    }
#ifdef HAS_DISPLAY
    // update information on screen.
    if (screen_timer == 0) {
        screen_timer = millis64();
    } else if (millis64() - screen_timer > 3000) { // Set to 3000 to reduce interference.
#ifdef HAS_OLED_TIMEOUT
        if (!oled_off)
#endif
            updateInfo();
        screen_timer = millis64();
    }
#ifdef HAS_OLED_TIMEOUT
    if (millis64() - timer4 >= OLED_TIMEOUT && timer4 != 0 && !oled_off) {
        u8g2->clearBuffer();
        oled_off = true;
        u8g2->setPowerSave(true);
    }
#endif
#endif
    // if (millis64()%5000 == 0){
    //     sd1.append("[D] 当前运行时间 %lu ms.\n",millis64());
    //     sd1.append("[D] 测试输出：\n");
    //     LBJTEST();
    // }

    // if (millis64() - format_task_timer >= 200 && format_task_timer != 0) {
    //     Serial.printf("LED LOW [%llu]\n", millis64() - format_task_timer);
    //     digitalWrite(BOARD_LED, LED_OFF);
    //     if (fd_state == TASK_DONE || fd_state == TASK_INIT) {
    //         format_task_timer = 0;
    //         // changeCpuFreq(240);
    //     }
    // }

    // handle task timeout
    // timeout & running | created
    // todo: simplify this judgement.
    if (millis64() - format_task_timer >= FD_TASK_TIMEOUT && (fd_state == TASK_RUNNING || fd_state == TASK_CREATED)
        && task_fd != nullptr && format_task_timer != 0) {
        vTaskDelete(task_fd);
        task_fd = nullptr;
        // fd_state = TASK_TERMINATED;
        dualPrintln("[Pager] FD_TASK Timeout.");
        sd1.append("[Pager] FD_TASK Timeout.\n");
        initFmtVars();
        Serial.printf("LED LOW [%llu]\n", millis64() - format_task_timer);
        digitalWrite(BOARD_LED, LED_OFF);
        format_task_timer = 0;
        led_timer = 0;
        changeCpuFreq(240);
        fd_state = TASK_INIT;
    }
    // else if (millis64() - format_task_timer >= FD_TASK_TIMEOUT && fd_state != TASK_INIT && format_task_timer != 0 &&
    //            fd_state != TASK_RUNNING_SCREEN) { // terminate task while u8g2 operation causes main loop stuck.
    //     Serial.printf("[Pager] Task state %d \n", fd_state);
    //     if (task_fd != nullptr) {
    //         vTaskDelete(task_fd);
    //         task_fd = nullptr;
    //     }
    //     dualPrintln("[Pager] FD_TASK Timeout.");
    //     sd1.append("[Pager] FD_TASK Timeout.\n");
    //     initFmtVars();
    //     Serial.printf("LED LOW [%llu]\n", millis64() - format_task_timer);
    //     digitalWrite(BOARD_LED, LED_OFF);
    //     format_task_timer = 0;
    //     led_timer = 0;
    //     changeCpuFreq(240);
    //     fd_state = TASK_INIT;
    // }

    if (millis64() - timer4 >= 60000 && timer4 != 0 && ets_get_cpu_frequency() != 80) // fCPU to 80 after 60s in idle.
//        setCpuFrequencyMhz(80);
        changeCpuFreq(80);

    handleCarrier();
    handlePreamble();

    handleSync();

    // the number of batches to wait for
    // 2 batches will usually be enough to fit short and medium messages
    if (pager.available() >= 2 && fd_state == TASK_INIT) { // todo add session timeout exception to prevent stuck here.
        // Serial.println("[PHY-LAYER][D] AVAILABLE > 2.");
        setCpuFrequencyMhz(240);
        db = new data_bond;
        runtime_timer = millis64();
        timer4 = millis64();
        int state = pager.readDataMSA(db->pocsagData, 0);
//        sd1.append("[PHY-LAYER][D] AVAILABLE > 2.\n");
        rxInfo.rssi = rssi_cache / (float) rxInfo.cnt;
        rssi_cache = 0;
        rxInfo.cnt = 0;
        rxInfo.timer = 0;
        prb_timer = 0;
        car_timer = 0;
        // radio.setRxBandwidth(20.8);
        // bandwidth_altered = false;

//        Serial.printf("CPU FREQ TO %d MHz\n",ets_get_cpu_frequency());

        // PagerClient::pocsag_data pocdat[POCDAT_SIZE];
        // struct lbj_data lbj;
        // pd = new PagerClient::pocsag_data[POCDAT_SIZE];

        Serial.printf("[D] Prb_count %d\n", prb_count);
        Serial.printf("[D] Car_count %d\n", car_count);
        if (prb_count >= 32)
            prb_count = 31;
        if (prb_count > 0)
            rxInfo.fer = fers[prb_count - 1];
        for (int i = 0; i < prb_count; ++i) {
            Serial.printf("[D] Fer %.2f Hz\n", fers[i]);
            fers[i] = 0;
        }
        // Serial.printf("[D] Fer %.2f Hz\n", fer);
        // fer = 0;
        prb_count = 0;
        car_count = 0;
        car_fer_last = 0;
        rxInfo.ppm = getBias(actual_frequency);

        Serial.println(F("[Pager] Received pager data, decoding ... "));
        sd1.append(2, "正在解码信号...\n");

        // you can read the data as an Arduino String
        // String str = {};

        if (state == RADIOLIB_ERR_NONE) {
            freq_last = actual_frequency;
//            Serial.printf("success.\n");
            digitalWrite(BOARD_LED, LED_ON);
            format_task_timer = millis64();
            led_timer = millis64();

            sd1.append(2, "正在格式化输出...\n");
            // formatDataTask();
            auto x_ret = xTaskCreatePinnedToCore(formatDataTask, "task_fd",
                                                 FD_TASK_STACK_SIZE, nullptr,
                                                 2, &task_fd, ARDUINO_RUNNING_CORE);
            if (x_ret == pdPASS) {
                fd_state = TASK_CREATED;
                delay(1);
            } else if (x_ret == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) {
                int x_ret1;
                for (int i = 0; i < FD_TASK_ATTEMPTS; ++i) {
                    x_ret1 = xTaskCreatePinnedToCore(formatDataTask, "task_fd",
                                                     FD_TASK_STACK_SIZE, nullptr,
                                                     2, &task_fd, ARDUINO_RUNNING_CORE);
                    if (x_ret1 == pdPASS) {
                        fd_state = TASK_CREATED;
                        delay(1);
                        break;
                    }
                    Serial.printf("[Pager] FTask failed memory allocation, error %d, mem left %d B, retry %d\n",
                                  x_ret1, esp_get_free_heap_size(), i);
                    sd1.append("[Pager] FTask failed memory allocation, error %d, mem left %d B, retry %d\n",
                               x_ret1, esp_get_free_heap_size(), i);
                }
                if (x_ret1 != pdPASS) {
                    Serial.printf("Mem left: %d Bytes\n", esp_get_free_heap_size());
                    dualPrintf(true, "[Pager] Format task memory allocation failure\n");
                    sd1.append("[Pager] Format task memory allocation failure, Mem left %d Bytes\n",
                               esp_get_free_heap_size());
                    fd_state = TASK_CREATE_FAILED;
                    simpleFormatTask();
                    digitalWrite(BOARD_LED, LED_OFF);
                }
            } else {
                dualPrintf(true, "[Pager] Failed to create format task, errcode %d\n", x_ret);
                sd1.append("[Pager] Failed to create format task, errcode %d\n", x_ret);
                fd_state = TASK_CREATE_FAILED;
                digitalWrite(BOARD_LED, LED_OFF);
            }

        } else if (state == RADIOLIB_ERR_MSG_CORRUPT) {
//            Serial.printf("failed.\n");
//            Serial.println("[Pager] Reception failed, too many errors.");
            dualPrintf(true, "[Pager] Reception failed, too many errors. \n");
            revertFrequency();
//            sd1.append("[Pager] Reception failed, too many errors. \n");
        } else {
            // some error occurred
            sd1.append("[Pager] Reception failed, code %d \n", state);
            dualPrintf(true, "[Pager] Reception failed, code %d \n", state);
        }

        // if task was not called.
        if (fd_state == TASK_INIT) {
            initFmtVars();
        } else if (fd_state == TASK_CREATED && task_fd == nullptr) {
            fd_state = TASK_DONE;
        }
    }
}

void revertFrequency() {
    if (actual_frequency != freq_last) {
        actual_frequency = freq_last;
        int state = radio.setFrequency(actual_frequency);
        if (state != RADIOLIB_ERR_NONE) {
            Serial.printf("[D] Revert freq failed %d\n", state);
        } else {
            Serial.printf("[D] Revert to last freq %f MHz, ppm %.2f\n", actual_frequency, getBias(actual_frequency));
        }
    }
}

void handleCarrier() {
    if (pager.gotCarrierState() && !pager.gotPreambleState() && !pager.gotSyncState() && freq_correction &&
        prb_timer == 0) {
        if (car_count == 0)
            car_timer = millis64();
        ++car_count;
        if (car_count < 64) {
            float fei = radio.getFrequencyError();
            // Serial.printf("[D] Carrier FEI %.2f Hz, count %d\n",fei,car_count);
            if (abs(fei) > 1000.0 && car_count != 1 &&
                abs(fei - car_fer_last) < 500) {
                // Perform frequency correction
                auto target_freq = (float) (actual_frequency + fei * 1e-6);
                int state = radio.setFrequency(target_freq);
                if (state != RADIOLIB_ERR_NONE) {
                    Serial.printf("[D][C] Freq Alter failed %d, target freq %f\n", state, target_freq);
                    sd1.append("[D][C] Freq Alter failed %d, target freq %f\n", state, target_freq);
                } else {
                    actual_frequency = target_freq;
                    Serial.printf("[D][C] Freq Altered %f MHz, FEI %.2f Hz, PPM %.2f\n", actual_frequency, fei,
                                  getBias(actual_frequency));
                }
            }
            car_fer_last = fei;
        }
    }
}

void handlePreamble() {
    if (pager.gotPreambleState() && !pager.gotSyncState() && freq_correction) {
        if (prb_count == 0)
            prb_timer = millis64();
        // if (millis64() - prb_timer > 500) {
        //     prb_count = 0;
        //     if (actual_frequency != freq_last) {
        //         actual_frequency = freq_last;
        //         int state = radio.setFrequency(actual_frequency);
        //         if (state != RADIOLIB_ERR_NONE) {
        //             Serial.printf("[D] Freq Alter failed %d\n", state);
        //         }
        //     }
        //     for (auto &i: fers) {
        //         i = 0;
        //     }
        // }
        ++prb_count;
        if (prb_count < 32) {
            // todo: Implement automatic bandwidth adjustment.
            // if (prb_count > 2 && !bandwidth_altered) {
            //     int16_t state = radio.swapRxBandwidth(12.5);
            //     Serial.printf("[D] Bandwidth to 12.5,code %d\n",state);
            //     radio.restartReceive(true);
            //     bandwidth_altered = true;
            // }
            // else if (prb_count > 6 && bandwidth_altered) {
            //     radio.swapRxBandwidth(12.5);
            //     bandwidth_altered = false;
            // }
            // radio.swapRxBandwidth(12.5);
            fers[prb_count - 1] = radio.getFrequencyError();
            if (abs(fers[prb_count - 1]) > 1000.0 && prb_count != 1 &&
                abs(fers[prb_count - 1] - fers[prb_count - 2]) < 500) {
                // Perform frequency correction
                auto target_freq = (float) (actual_frequency + fers[prb_count - 1] * 1e-6);
                int state = radio.setFrequency(target_freq);
                if (state != RADIOLIB_ERR_NONE) {
                    Serial.printf("[D][P] Freq Alter failed %d, target freq %f\n", state, target_freq);
                    sd1.append("[D][P] Freq Alter failed %d, target freq %f\n", state, target_freq);
                } else {
                    actual_frequency = target_freq;
                    Serial.printf("[D][P] Freq Altered %f MHz, FEI %.2f Hz, PPM %.2f\n", actual_frequency,
                                  fers[prb_count - 1], getBias(actual_frequency));
                }
            }
        }
    }
}

void getCoreFreq(void *pVoid) {
    Serial.printf("Core %d Frequency %d MHz\n", xPortGetCoreID(), ets_get_cpu_frequency());
    vTaskDelete(nullptr);
}

void handleSerialInput() {
    if (Serial.available()) {
        String in = Serial.readStringUntil('\r');
        if (in == "ble on") {
            if (!ble_enabled) {
                ble_enabled = true;
                initBLE();
                Serial.println("[BLE] Enabled");
            } else {
                BLEDevice::deinit(true);
                delay(500);
                initBLE();
                Serial.println("[BLE] Reinitialized");
            }
        } else if (in == "ble off") {
            if (ble_enabled) {
                ble_enabled = false;
                BLEDevice::deinit(true);
                Serial.println("[BLE] Disabled");
            } else {
                Serial.println("[BLE] Already disabled");
            }
        } else if (in == "ble status") {
            Serial.printf("[BLE] Status: %s, Connection: %s\n",
                          ble_enabled ? "enabled" : "disabled",
                          deviceConnected.load(std::memory_order_acquire) ? "Connected" : "Disconnected");
        } else if (in == "ping")
            Serial.println("$ Pong");
        else if (in == "task state")
            Serial.println("$ Task state " + String(fd_state));
        else if (in == "rtc") {
#ifdef HAS_RTC
            // rtc.getDateTime(time_info);
            // DateTime now = rtc.now();
            time_info = rtcLibtoC(rtc.now());
            float temp = rtc.getTemperature();
            Serial.print(&time_info, "$ [eRTC] %Y-%m-%d %H:%M:%S ");
            Serial.printf("Temp: %.2f °C\n", temp);
#endif
        } else if (in == "time") {
            getLocalTime(&time_info, 1);
            Serial.printf("$ SYS Time %s, Up time %llu ms (%s)\n", fmtime(time_info), millis64(), fmtms(millis64()));
        } else if (in == "cd") {
            if (have_cd)
                Serial.println("$ Core dump exported.");
            else
                Serial.println("$ No core dump.");
        } else if (in == "sd end") {
            if (!sd1.status())
                Serial.println("$ [SDLOG] No SD.");
            else {
                sd1.append("[SDLOG] SD卡将被卸载\n");
                sd1.end();
                Serial.println("$ [SDLOG] SD end.");
            }
        } else if (in == "sd begin") {
            if (sd1.status())
                Serial.println("$ End SD First.");
            else {
                SD_LOG::reopenSD();
                sd1.begin("/LOGTEST");
                sd1.beginCSV("/CSVTEST");
                sd1.append("[SDLOG] SD卡已重新挂载\n");
                Serial.println("$ [SDLOG] SD reopen.");
            }
        } else if (in == "mem") {
            Serial.printf("$ Mem left: %d Bytes\n", esp_get_free_heap_size());
        } else if (in == "rst") {
            esp_reset_reason_t reason = esp_reset_reason();
            Serial.printf("$ RST: %s\n", printResetReason(reason).c_str());
        } else if (in == "ppm") {
            if (runtime_timer == 0 && !pager.gotSyncState()) {
                ppm = 3;
                int16_t state = radio.setFrequency(actualFreq(ppm));
                if (state == RADIOLIB_ERR_NONE)
                    Serial.printf("$ Actual Frequency %f MHz\n", actualFreq(ppm));
                else
                    Serial.printf("$ Failure, Code %d\n", state);
            } else {
                Serial.println("$ Unable to change frequency due to occupation");
                if (pager.available())
                    Serial.println("$ pager.available == true");
                if (runtime_timer)
                    Serial.printf("$ runtime_timer = %llu, running %llu\n", runtime_timer, millis64() - runtime_timer);
            }
        } else if (in == "ppm read") {
            Serial.printf("$ ppm %.1f\n", ppm);
        } else if (in == "afc off") {
            prb_count = 0;
            prb_timer = 0;
            car_count = 0;
            car_timer = 0;
            freq_correction = false;
            Serial.println("$ Frequency Correction Disabled");
        } else if (in == "afc on") {
            freq_correction = true;
            Serial.println("$ Frequency Correction Enabled");
        } else if (in == "rssi") {
            Serial.printf("$ RSSI %3.2f dBm.\n", radio.getRSSI(false, true));
        } else if (in == "gain") {
            Serial.printf("$ Gain Pos %d \n", radio.getGain());
        } else if (in == "cpu") {
            xTaskCreatePinnedToCore(getCoreFreq, "get_freq", 2048, nullptr,
                                    1, nullptr, 0);
            Serial.printf("Core %d Frequency %d MHz\n", xPortGetCoreID(), ets_get_cpu_frequency());
        }
    }
}

void initFmtVars() {
    Serial.printf("[Pager] Processing time %llu ms.\n", millis64() - runtime_timer);
    runtime_timer = 0;
    rxInfo.rssi = 0;
    rxInfo.fer = 0;
    rxInfo.ppm = 0;
    // prb_count = 0;
    // for (auto &i: fers) {
    //     i = 0;
    // }
    if (db != nullptr) {
        delete db;
        db = nullptr;
    }
}

void formatDataTask(void *pVoid) {
    fd_state = TASK_RUNNING;
    // Serial.printf("[FD-Task] Stack High Mark Begin %u\n", uxTaskGetStackHighWaterMark(nullptr));
    sd1.append(2, "格式化任务已创建\n");
    for (auto &i: db->pocsagData) {
        if (i.is_empty)
            continue;
        Serial.printf("[D-pDATA] %d/%d: %s\n", i.addr, i.func, i.str.c_str());
        sd1.append(2, "[D-pDATA] %d/%d: %s\n", i.addr, i.func, i.str.c_str());
        db->str = db->str + "  " + i.str;
    }

    // Serial.printf("[FD-Task] Stack High Mark pDATA %u\n", uxTaskGetStackHighWaterMark(nullptr));
    sd1.append(2, "原始数据输出完成，用时[%llu]\n", millis64() - runtime_timer);
    Serial.printf("decode complete.[%llu]", millis64() - runtime_timer);
    readDataLBJ(db->pocsagData, &db->lbjData);
    sd1.append(2, "LBJ读取完成，用时[%llu]\n", millis64() - runtime_timer);
    Serial.printf("Read complete.[%llu]", millis64() - runtime_timer);
    // Serial.printf("[FD-Task] Stack High Mark rLBJ %u\n", uxTaskGetStackHighWaterMark(nullptr));

    printDataSerial(db->pocsagData, db->lbjData, rxInfo);
    sd1.append(2, "串口输出完成，用时[%llu]\n", millis64() - runtime_timer);
    if (ble_enabled) {
        bleLinkPublishTrain(db->lbjData, rxInfo);
    }
    Serial.printf("SPRINT complete.[%llu]", millis64() - runtime_timer);

    // sd1.disableSizeCheck();
    appendDataLog(db->pocsagData, db->lbjData, rxInfo);
    Serial.printf("sdprint complete.[%llu]", millis64() - runtime_timer);
    appendDataCSV(db->pocsagData, db->lbjData, rxInfo);
    Serial.printf("csvprint complete.[%llu]", millis64() - runtime_timer);
    // sd1.enableSizeCheck();

    // Serial.printf("[FD-Task] Stack High Mark TRI-OUT %u\n", uxTaskGetStackHighWaterMark(nullptr));
// Serial.printf("type %d \n",lbj.type);

#ifdef HAS_DISPLAY
    fd_state = TASK_RUNNING_SCREEN;
    if (u8g2) {
#ifdef HAS_OLED_TIMEOUT
        if (oled_off) {
            oled_off = false;
            u8g2->setPowerSave(false);
            u8g2->clearBuffer();
            updateInfo();
        }
#endif
        if (db->lbjData.type == 0)
            showLBJ0(db->lbjData);
        else if (db->lbjData.type == 1) {
            showLBJ1(db->lbjData);
        } else if (db->lbjData.type == 2) {
            showLBJ2(db->lbjData);
        }
        Serial.printf("Complete u8g2 [%llu]\n", millis64() - runtime_timer);
    }
#endif
    Serial.printf("[FD-Task] Stack High Mark %u\n", uxTaskGetStackHighWaterMark(nullptr));
    sd1.append(2, "任务堆栈标 %u\n", uxTaskGetStackHighWaterMark(nullptr));
    // sd1.append("[FD-Task] Stack High Mark %u\n", uxTaskGetStackHighWaterMark(nullptr));
    sd1.append(2, "格式化输出任务完成，用时[%llu]\n", millis64() - runtime_timer);
    fd_state = TASK_DONE;
    task_fd = nullptr;
    vTaskDelete(nullptr);
}

void simpleFormatTask() { // only output initially phrased data in case of memory shortage
    for (auto &i: db->pocsagData) {
        if (i.is_empty)
            continue;
        Serial.printf("[D-pDATA] %d/%d: %s\n", i.addr, i.func, i.str.c_str());
        sd1.append("[D-pDATA] %d/%d: %s\n", i.addr, i.func, i.str.c_str());
        // db->str = db->str + "  " + i.str;
        db->str += String(i.addr) + "/" + String(i.func) + ":" + i.str + "\n ";
    }
    // pword(db->str.c_str(),20,50);
#ifdef HAS_OLED_TIMEOUT
    if (oled_off) {
        oled_off = false;
        u8g2->setPowerSave(false);
        u8g2->clearBuffer();
        updateInfo();
    }
#endif
    showSTR(db->str);
}
// END OF FILE.