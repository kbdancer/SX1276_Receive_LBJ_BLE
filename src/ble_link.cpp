#include "ble_link.hpp"
#include "boards.hpp"

#include <ArduinoJson.h>
#include <SD.h>
#include <atomic>

extern bool ble_enabled;
extern std::atomic<bool> deviceConnected;
extern bool have_sd;

static BLECharacteristic *txChar = nullptr;

static const char *SYNC_DIR = "/SYNC";
static const char *SYNC_FILE = "/SYNC/records.jsonl";
static const char *SYNC_META = "/SYNC/meta.json";
static const size_t SYNC_MAX_BYTES = 2 * 1024 * 1024; // 2MB rotate

static uint32_t nextSeq = 1;
static uint32_t ackedSeq = 0;
static bool syncActive = false;
static uint32_t syncSendFrom = 0;
static File syncReadFile;
static bool syncFileOpen = false;
static uint64_t lastSyncSendMs = 0;
static const uint32_t SYNC_SEND_INTERVAL_MS = 40;

static bool pendingAutoSync = false;

static void bleNotifyRaw(const String &payload) {
    if (!ble_enabled || txChar == nullptr) {
        return;
    }
    if (!deviceConnected.load(std::memory_order_acquire)) {
        return;
    }
    txChar->setValue(payload.c_str());
    txChar->notify();
}

static void linkEmit(const String &payload) {
    bleNotifyRaw(payload);
}

static bool linkHasLiveClient() {
    return deviceConnected.load(std::memory_order_acquire);
}

static void bleSendResp(const char *cmd, bool ok, const JsonObject &extra) {
    DynamicJsonDocument doc(768);
    doc["type"] = "resp";
    doc["cmd"] = cmd;
    doc["ok"] = ok;
    for (JsonPair kv : extra) {
        doc[kv.key()] = kv.value();
    }
    String out;
    serializeJson(doc, out);
    linkEmit(out);
    Serial.printf("[LINK] RESP %s\n", out.c_str());
}

static void bleSendRespSimple(const char *cmd, bool ok) {
    DynamicJsonDocument empty(64);
    bleSendResp(cmd, ok, empty.as<JsonObject>());
}

static void loadSyncMeta() {
    nextSeq = 1;
    ackedSeq = 0;
    if (!have_sd) {
        return;
    }
    if (!SD.exists(SYNC_META)) {
        return;
    }
    File f = SD.open(SYNC_META, FILE_READ);
    if (!f) {
        return;
    }
    DynamicJsonDocument doc(256);
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
        nextSeq = doc["next_seq"] | 1;
        ackedSeq = doc["acked_seq"] | 0;
        if (nextSeq == 0) {
            nextSeq = 1;
        }
    }
    f.close();
}

static void saveSyncMeta() {
    if (!have_sd) {
        return;
    }
    SD.mkdir(SYNC_DIR);
    File f = SD.open(SYNC_META, FILE_WRITE);
    if (!f) {
        return;
    }
    DynamicJsonDocument doc(128);
    doc["next_seq"] = nextSeq;
    doc["acked_seq"] = ackedSeq;
    serializeJson(doc, f);
    f.close();
}

static String buildTrainJson(const struct lbj_data &l, const struct rx_info &r, bool isTest,
                             uint32_t seq, bool isSync) {
    DynamicJsonDocument doc(1024);
    doc["train"] = l.train;
    doc["dir"] = l.direction;
    doc["speed"] = l.speed;
    doc["pos"] = l.position;
    doc["time"] = l.time;
    doc["loco"] = l.loco;
    doc["loco_type"] = l.loco_type;
    doc["lbj_class"] = l.lbj_class;
    doc["route"] = l.route_utf8;

    String position_info;
    if (l.pos_lat_deg[0] && l.pos_lat_min[0] && l.pos_lon_deg[0] && l.pos_lon_min[0]) {
        position_info = String(l.pos_lat_deg) + "°" + String(l.pos_lat_min) + "′ "
                      + String(l.pos_lon_deg) + "°" + String(l.pos_lon_min) + "′";
    } else if (l.pos_lat[0] && l.pos_lon[0]) {
        position_info = String(l.pos_lat) + " " + String(l.pos_lon);
    }
    doc["position_info"] = position_info;
    doc["rssi"] = r.rssi;
    doc["test_flag"] = isTest;
    doc["seq"] = seq;
    doc["device_ts"] = (uint32_t) (millis() / 1000);
    if (isSync) {
        doc["sync"] = true;
    }

    String output;
    serializeJson(doc, output);
    return output;
}

static bool appendSyncLine(const String &line) {
    if (!have_sd) {
        return false;
    }
    SD.mkdir(SYNC_DIR);

    if (SD.exists(SYNC_FILE)) {
        File probe = SD.open(SYNC_FILE, FILE_READ);
        if (probe) {
            size_t sz = probe.size();
            probe.close();
            if (sz >= SYNC_MAX_BYTES) {
                String bak = String(SYNC_FILE) + ".old";
                SD.remove(bak);
                SD.rename(SYNC_FILE, bak.c_str());
            }
        }
    }

    File f = SD.open(SYNC_FILE, FILE_APPEND);
    if (!f) {
        return false;
    }
    f.println(line);
    f.close();
    return true;
}

static uint32_t countUnreadApprox() {
    if (!have_sd || nextSeq <= ackedSeq + 1) {
        return nextSeq > ackedSeq ? (nextSeq - ackedSeq - 1) : 0;
    }
    return nextSeq - ackedSeq - 1;
}

static void fillSdStatus(JsonObject obj) {
    obj["present"] = have_sd;
    if (!have_sd) {
        obj["total_mb"] = 0;
        obj["used_mb"] = 0;
        obj["free_mb"] = 0;
        obj["sync_pending"] = 0;
        obj["next_seq"] = nextSeq;
        obj["acked_seq"] = ackedSeq;
        return;
    }
    uint64_t total = SD.cardSize();
    uint64_t used = SD.usedBytes();
    uint64_t freeb = total > used ? total - used : 0;
    obj["total_mb"] = (uint32_t) (total / (1024 * 1024));
    obj["used_mb"] = (uint32_t) (used / (1024 * 1024));
    obj["free_mb"] = (uint32_t) (freeb / (1024 * 1024));
    obj["sync_pending"] = countUnreadApprox();
    obj["next_seq"] = nextSeq;
    obj["acked_seq"] = ackedSeq;
    obj["sync_file"] = SD.exists(SYNC_FILE);
}

static void closeSyncReader() {
    if (syncFileOpen) {
        syncReadFile.close();
        syncFileOpen = false;
    }
}

static void startSyncReplay() {
    closeSyncReader();
    syncActive = false;
    if (!have_sd) {
        DynamicJsonDocument extra(256);
        JsonObject obj = extra.to<JsonObject>();
        obj["reason"] = "no_sd";
        obj["sent"] = 0;
        obj["done"] = true;
        bleSendResp("sync", false, obj);
        return;
    }
    if (!SD.exists(SYNC_FILE)) {
        DynamicJsonDocument extra(256);
        JsonObject obj = extra.to<JsonObject>();
        obj["sent"] = 0;
        obj["done"] = true;
        obj["pending"] = 0;
        bleSendResp("sync", true, obj);
        return;
    }
    syncReadFile = SD.open(SYNC_FILE, FILE_READ);
    if (!syncReadFile) {
        bleSendRespSimple("sync", false);
        return;
    }
    syncFileOpen = true;
    syncSendFrom = ackedSeq;
    syncActive = true;
    lastSyncSendMs = 0;
    DynamicJsonDocument extra(256);
    JsonObject obj = extra.to<JsonObject>();
    obj["pending"] = countUnreadApprox();
    obj["from_seq"] = ackedSeq;
    obj["done"] = false;
    bleSendResp("sync", true, obj);
}

static void processSyncReplay() {
    if (!syncActive || !syncFileOpen) {
        return;
    }
    if (!linkHasLiveClient()) {
        closeSyncReader();
        syncActive = false;
        return;
    }
    uint64_t now = millis();
    if (lastSyncSendMs != 0 && now - lastSyncSendMs < SYNC_SEND_INTERVAL_MS) {
        return;
    }

    while (syncReadFile.available()) {
        String line = syncReadFile.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) {
            continue;
        }
        DynamicJsonDocument doc(1024);
        if (deserializeJson(doc, line) != DeserializationError::Ok) {
            continue;
        }
        uint32_t seq = doc["seq"] | 0;
        if (seq <= syncSendFrom) {
            continue;
        }
        doc["sync"] = true;
        String out;
        serializeJson(doc, out);
        linkEmit(out);
        lastSyncSendMs = now;
        syncSendFrom = seq;
        return; // one record per tick
    }

    // finished
    closeSyncReader();
    syncActive = false;
    ackedSeq = syncSendFrom;
    if (ackedSeq + 1 > nextSeq) {
        // keep meta consistent
    }
    saveSyncMeta();
    DynamicJsonDocument extra(256);
    JsonObject obj = extra.to<JsonObject>();
    obj["done"] = true;
    obj["acked_seq"] = ackedSeq;
    obj["pending"] = countUnreadApprox();
    bleSendResp("sync", true, obj);
}

static void clearSyncHistory() {
    if (!have_sd) {
        DynamicJsonDocument extra(128);
        JsonObject obj = extra.to<JsonObject>();
        obj["reason"] = "no_sd";
        bleSendResp("sd_clear", false, obj);
        return;
    }
    closeSyncReader();
    syncActive = false;

    // Clear sync queue only (not whole card). Also remove rotated backup.
    SD.remove(SYNC_FILE);
    SD.remove(String(SYNC_FILE) + ".old");
    SD.remove(SYNC_META);

    // Optional: clear LBJ CSV/LOG history dirs used by existing logger.
    auto wipeDirFiles = [](const char *dir) {
        File root = SD.open(dir);
        if (!root || !root.isDirectory()) {
            return;
        }
        File file = root.openNextFile();
        while (file) {
            String name = file.name();
            file.close();
            // file.name() may be full path or basename depending on core
            String path = name.startsWith("/") ? name : String(dir) + "/" + name;
            if (!path.endsWith("/")) {
                SD.remove(path);
            }
            file = root.openNextFile();
        }
        root.close();
    };
    wipeDirFiles("/CSVTEST");
    wipeDirFiles("/LOGTEST");

    nextSeq = 1;
    ackedSeq = 0;
    saveSyncMeta();

    DynamicJsonDocument extra(256);
    JsonObject obj = extra.to<JsonObject>();
    fillSdStatus(obj);
    bleSendResp("sd_clear", true, obj);
}

void bleLinkSetCharacteristic(BLECharacteristic *c) {
    txChar = c;
    if (have_sd) {
        SD.mkdir(SYNC_DIR);
        loadSyncMeta();
    }
}

void bleLinkOnWrite(const std::string &value) {
    if (value.empty()) {
        return;
    }
    bleLinkHandleCommand(String(value.c_str()));
}

void bleLinkHandleCommand(const String &value) {
    if (value.isEmpty()) {
        return;
    }
    Serial.printf("[LINK] CMD: %s\n", value.c_str());

    DynamicJsonDocument doc(768);
    DeserializationError err = deserializeJson(doc, value.c_str());
    if (err) {
        // also accept plain text commands
        String s = value;
        s.trim();
        if (s == "sync") {
            startSyncReplay();
        } else if (s == "sd_status") {
            DynamicJsonDocument extra(384);
            JsonObject obj = extra.to<JsonObject>();
            fillSdStatus(obj);
            bleSendResp("sd_status", true, obj);
        } else {
            bleSendRespSimple("unknown", false);
        }
        return;
    }

    const char *cmd = doc["cmd"] | "";
    if (strcmp(cmd, "sync") == 0) {
        if (doc.containsKey("since_seq")) {
            ackedSeq = doc["since_seq"] | ackedSeq;
            saveSyncMeta();
        }
        startSyncReplay();
    } else if (strcmp(cmd, "sync_ack") == 0) {
        uint32_t seq = doc["seq"] | 0;
        if (seq > ackedSeq) {
            ackedSeq = seq;
            saveSyncMeta();
        }
        DynamicJsonDocument extra(128);
        JsonObject obj = extra.to<JsonObject>();
        obj["acked_seq"] = ackedSeq;
        bleSendResp("sync_ack", true, obj);
    } else if (strcmp(cmd, "sd_status") == 0) {
        DynamicJsonDocument extra(384);
        JsonObject obj = extra.to<JsonObject>();
        fillSdStatus(obj);
        bleSendResp("sd_status", true, obj);
    } else if (strcmp(cmd, "sd_clear") == 0) {
        clearSyncHistory();
    } else {
        bleSendRespSimple("unknown", false);
    }
}

void bleLinkPublishTrain(const struct lbj_data &l, const struct rx_info &r, bool isTest) {
    uint32_t seq = nextSeq++;
    String json = buildTrainJson(l, r, isTest, seq, false);

    if (have_sd) {
        if (appendSyncLine(json)) {
            saveSyncMeta();
        } else {
            Serial.println("[SYNC] append failed");
        }
    }

    if (linkHasLiveClient()) {
        linkEmit(json);
        Serial.println("[LINK] Data sent");
        if (have_sd && !syncActive) {
            ackedSeq = seq;
            saveSyncMeta();
        }
    }
}

void bleLinkOnConnected() {
    pendingAutoSync = have_sd;
    DynamicJsonDocument extra(384);
    JsonObject obj = extra.to<JsonObject>();
    fillSdStatus(obj);
    bleSendResp("sd_status", true, obj);
}


void bleLinkOnDisconnected() {
    closeSyncReader();
    syncActive = false;
    pendingAutoSync = false;
}

void bleLinkLoop() {
    if (pendingAutoSync && linkHasLiveClient() && have_sd) {
        pendingAutoSync = false;
        if (countUnreadApprox() > 0) {
            startSyncReplay();
        }
    }
    processSyncReplay();
}

uint32_t bleLinkBufferedCount() {
    return countUnreadApprox();
}

uint32_t bleLinkLatestSeq() {
    return nextSeq > 0 ? nextSeq - 1 : 0;
}
