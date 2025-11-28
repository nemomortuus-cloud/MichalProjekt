/*
 * triac_controller_remote_mode.ino
 *
 * Обновлённая прошивка ESP32 для управления триаками Tuya.
 * - Более гладкий PID с фильтрацией и защитой от насыщения.
 * - REMOTE-режим изолирован: ESP не шлёт команды в Tuya, пока Python управляет лампами.
 * - Режим TEST использует тот же PID, но с поправкой targetLux - correction.
 * - Поддержка checkbox'ов из MATLAB: каждый канал можно отключить в AUTO/TEST.
 * - Коррекции, приходящие раз в секунду, буферизуются до следующего обновления.
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <mbedtls/md.h>
#include <map>
#include <math.h>
#include <time.h>

// ===================== Wi-Fi =====================
const char* WIFI_SSID = "SD111";
const char* WIFI_PASS = "11114422";

// ===================== Tuya Cloud =====================
const String CLIENT_ID     = "p3hea9cxpe8wxegcdmyd";
const String CLIENT_SECRET = "770933630c54493c847f0e4ca19ed7de";
const String REGION_HOST   = "https://openapi.tuyaeu.com";

const String TRIAC_1 = "bfa302be3e853f564aaaoz";
const String TRIAC_2 = "bf0794d9f4981c9715zygg";
const String TRIAC_3 = "bfc0ad553a9d0a8c07tcov";

// ===================== Server =====================
const char* SERVER_IP   = "100.105.5.95";
const int   SERVER_PORT = 5000;

// ===================== Sensors =====================
#define SENSOR_PIN_1 34
#define SENSOR_PIN_2 35
#define SENSOR_PIN_3 32

// ===================== Timing / Control =====================
const uint32_t LOOP_TS_MS              = 10;
const float    SENSOR_ALPHA            = 0.25f;
const float    PID_OUTPUT_ALPHA        = 0.35f;
const float    PID_HOLD_BAND           = 40.0f;
const float    PID_INTEGRAL_CLAMP      = 2500.0f;
const float    PID_D_MIN_DT            = 0.001f;
const unsigned long MIN_SEND_INTERVAL_MS = 90;
const int LEVEL_CHANGE_THRESHOLD       = 3;

// ===================== Modes =====================
const int MODE_AUTO   = 0;
const int MODE_RECORD = 1;
const int MODE_TEST   = 2;
const int MODE_MANUAL = 3;
const int MODE_REMOTE = 4;

int mode = MODE_AUTO;

// ===================== State =====================
int   manualLevel[3]      = {300, 300, 300};
int   lastCommandedLevel[3] = {300, 300, 300};
bool  triacEnabled[3]     = {true, true, true};
float targetLux[3]        = {1000.0f, 1000.0f, 1000.0f};
float correctionFeed[3]   = {0.0f, 0.0f, 0.0f};
unsigned long correctionStamp[3] = {0, 0, 0};

float pidCommand[3]       = {300.0f, 300.0f, 300.0f};
float Kp_pid = 0.045f;
float Ki_pid = 0.0018f;
float Kd_pid = 0.0f;

struct ChannelPidState {
    float integral;
    float prevErr;
    float filtered;
};

ChannelPidState pidState[3] = {
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f}
};

int prevSentLevel[3]      = {-1, -1, -1};
unsigned long lastSendTime[3] = {0, 0, 0};

volatile bool manualApplyPending = false;
bool manualForceSend = false;

// ===================== Tuya token =====================
String access_token = "";
unsigned long long token_expire_ms = 0;
long long time_offset_ms = 0;

// ===================== WebSocket =====================
WebSocketsClient webSocket;

// ===================== Utils =====================
String sha256_hex_lower(const String& data) {
    byte hash[32];
    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, info, 0);
    mbedtls_md_starts(&ctx);
    mbedtls_md_update(&ctx, (const unsigned char*)data.c_str(), data.length());
    mbedtls_md_finish(&ctx, hash);
    mbedtls_md_free(&ctx);

    char hex[65];
    for (int i = 0; i < 32; i++) sprintf(hex + i * 2, "%02x", hash[i]);
    hex[64] = 0;
    return String(hex);
}

String hmac_sha256_hex_upper(const String& key, const String& msg) {
    byte hash[32];
    mbedtls_md_context_t ctx;
    const mbedtls_md_info_t* info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);

    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, info, 1);
    mbedtls_md_hmac_starts(&ctx, (const unsigned char*)key.c_str(), key.length());
    mbedtls_md_hmac_update(&ctx, (const unsigned char*)msg.c_str(), msg.length());
    mbedtls_md_hmac_finish(&ctx, hash);
    mbedtls_md_free(&ctx);

    char hex[65];
    for (int i = 0; i < 32; i++) sprintf(hex + i * 2, "%02X", hash[i]);
    hex[64] = 0;
    return String(hex);
}

unsigned long long now_epoch_ms() {
    time_t sec = time(NULL);
    if (sec <= 0) return millis();
    return ((unsigned long long)sec) * 1000ULL;
}

bool httpGetWithHeaders(const String& host, const String& path,
                        const std::map<String, String>& headers, String& out) {
    HTTPClient http;
    String url = host + path;
    http.begin(url);
    for (auto const &kv : headers)
        http.addHeader(kv.first, kv.second);

    int code = http.GET();
    out = http.getString();
    Serial.printf("[HTTP GET] %s -> code=%d\n", url.c_str(), code);
    http.end();
    return (code > 0);
}

bool httpPostWithHeaders(const String& host, const String& path,
                         const std::map<String, String>& headers,
                         const String& body, String& out) {
    HTTPClient http;
    String url = host + path;
    http.begin(url);
    for (auto const &kv : headers)
        http.addHeader(kv.first, kv.second);

    int code = http.POST(body);
    out = http.getString();
    Serial.printf("[HTTP POST] %s -> code=%d\n", url.c_str(), code);
    Serial.print("[HTTP POST] body: ");
    Serial.println(body);
    Serial.print("[HTTP POST] resp: ");
    Serial.println(out);
    http.end();
    return (code > 0);
}

bool get_token_and_sync();
std::map<String,String> build_headers(const String& method,
                                      const String& path,
                                      const String& body_json);

bool get_token_and_sync() {
    String path = "/v1.0/token?grant_type=1";
    unsigned long long t_epoch_ms = now_epoch_ms();
    String t = String(t_epoch_ms);
    String nonce = "";
    const String empty_sha =
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";

    String sign_str = "GET\n" + empty_sha + "\n\n" + path;
    String string_to_sign = CLIENT_ID + t + nonce + sign_str;
    String sign = hmac_sha256_hex_upper(CLIENT_SECRET, string_to_sign);

    std::map<String, String> headers;
    headers["client_id"]   = CLIENT_ID;
    headers["sign"]        = sign;
    headers["t"]           = t;
    headers["sign_method"] = "HMAC-SHA256";

    String resp;
    if (!httpGetWithHeaders(REGION_HOST, path, headers, resp)) {
        Serial.println("[TUYA] HTTP GET token failed");
        return false;
    }

    DynamicJsonDocument doc(4096);
    DeserializationError err = deserializeJson(doc, resp);
    if (err) {
        Serial.print("[TUYA] token JSON parse error: ");
        Serial.println(err.c_str());
        return false;
    }

    if (doc.containsKey("t"))
        time_offset_ms = doc["t"].as<long long>() - (long long)now_epoch_ms();

    if (!doc["success"].as<bool>()) {
        Serial.println("[TUYA] token request success=false");
        serializeJsonPretty(doc, Serial);
        return false;
    }

    access_token = doc["result"]["access_token"].as<String>();
    int expire_time = doc["result"]["expire_time"].as<int>();
    token_expire_ms = now_epoch_ms()
                      + (unsigned long long)expire_time * 1000ULL
                      - 600000ULL;

    Serial.printf("[TUYA] Token OK, expires in %d s\n", expire_time);
    return true;
}

std::map<String,String> build_headers(const String& method,
                                      const String& path,
                                      const String& body_json) {
    if (access_token == "" || now_epoch_ms() > token_expire_ms)
        get_token_and_sync();

    unsigned long long t_epoch_ms = now_epoch_ms() + time_offset_ms;
    String t = String(t_epoch_ms);
    String nonce = "";
    String body_hash = sha256_hex_lower(body_json);
    String sign_url  = method + "\n" + body_hash + "\n\n" + path;
    String string_to_sign = CLIENT_ID + access_token + t + nonce + sign_url;
    String sign = hmac_sha256_hex_upper(CLIENT_SECRET, string_to_sign);

    std::map<String,String> headers;
    headers["client_id"]    = CLIENT_ID;
    headers["access_token"] = access_token;
    headers["sign"]         = sign;
    headers["t"]            = t;
    headers["sign_method"]  = "HMAC-SHA256";
    headers["Content-Type"] = "application/json";
    return headers;
}

int normalizeLevel(int inLevel) {
    if (inLevel <= 0) return -1;
    if (inLevel <= 100) {
        int out = map(inLevel, 1, 100, 20, 1000);
        return constrain(out, 20, 1000);
    }
    return constrain(inLevel, 0, 1000);
}

bool pushTriacToTuya(int index, int inputLevel) {
    const String dev = (index == 0 ? TRIAC_1 : (index == 1 ? TRIAC_2 : TRIAC_3));
    int level = normalizeLevel(inputLevel);
    bool switchOn = (level > 0);
    if (level < 0) {
        level = 0;
        switchOn = false;
    }

    int tuyaLevel = level;
    if (tuyaLevel > 1000) tuyaLevel = 1000;
    if (tuyaLevel > 0 && tuyaLevel < 10) tuyaLevel = 10;

    DynamicJsonDocument doc(256);
    JsonArray cmds = doc.createNestedArray("commands");

    JsonObject c0 = cmds.createNestedObject();
    c0["code"]  = "switch_led";
    c0["value"] = switchOn;

    JsonObject c1 = cmds.createNestedObject();
    c1["code"]  = "work_mode";
    c1["value"] = "white";

    if (switchOn) {
        JsonObject c2 = cmds.createNestedObject();
        c2["code"]  = "bright_value_v2";
        c2["value"] = tuyaLevel;
    }

    String body;
    serializeJson(doc, body);
    Serial.printf("[TUYA] triac_set -> device=%s input=%d mapped=%d on=%d\n",
                  dev.c_str(), inputLevel, tuyaLevel, switchOn ? 1 : 0);
    Serial.print("[TUYA] will POST body: ");
    Serial.println(body);

    String path = "/v1.0/devices/" + dev + "/commands";
    std::map<String,String> headers = build_headers("POST", path, body);
    String resp;
    bool ok = false;

    if (!httpPostWithHeaders(REGION_HOST, path, headers, body, resp)) {
        Serial.println("[TUYA] HTTP POST fail");
        ok = false;
    } else {
        DynamicJsonDocument rdoc(2048);
        DeserializationError err = deserializeJson(rdoc, resp);
        if (err) {
            Serial.print("[TUYA] parse resp error: ");
            Serial.println(err.c_str());
            ok = false;
        } else {
            ok = rdoc.containsKey("success") && rdoc["success"].as<bool>();
            if (!ok) serializeJsonPretty(rdoc, Serial);
        }
    }

    Serial.printf("[TUYA] triac_set result: %d\n", ok ? 1 : 0);
    return ok;
}

bool applyTriacLevel(int idx, int inputLevel, bool forceSend) {
    if (mode == MODE_REMOTE) return false;
    int target = constrain(inputLevel, 0, 1000);
    int normalized = normalizeLevel(target);
    int cmp = normalized < 0 ? 0 : normalized;
    unsigned long nowMs = millis();
    bool timeReady = (nowMs - lastSendTime[idx]) >= MIN_SEND_INTERVAL_MS;

    if (!forceSend) {
        if (!timeReady) return false;
        if (prevSentLevel[idx] >= 0 && abs(cmp - prevSentLevel[idx]) < LEVEL_CHANGE_THRESHOLD)
            return false;
    }

    bool ok = pushTriacToTuya(idx, target);
    if (ok) {
        prevSentLevel[idx] = cmp;
        lastSendTime[idx] = nowMs;
    }
    return ok;
}

void setMode(int newMode) {
    if (newMode < MODE_AUTO) newMode = MODE_AUTO;
    if (newMode > MODE_REMOTE) newMode = MODE_REMOTE;
    if (mode == newMode) return;

    Serial.printf("[MODE] %d -> %d\n", mode, newMode);
    mode = newMode;

    if (mode == MODE_REMOTE) {
        manualApplyPending = false;
        manualForceSend = false;
        for (int i = 0; i < 3; ++i) {
            prevSentLevel[i] = -1;
        }
    }

    if (mode == MODE_RECORD) {
        for (int i = 0; i < 3; ++i) {
            pidCommand[i] = 0.0f;
            lastCommandedLevel[i] = 0;
            applyTriacLevel(i, 0, true);
        }
    }
}

void setTriacEnabled(int idx, bool value) {
    if (idx < 0 || idx > 2) return;
    if (triacEnabled[idx] == value) return;
    triacEnabled[idx] = value;
    Serial.printf("[ENABLE] triac %d -> %d\n", idx, value ? 1 : 0);

    if (!value) {
        pidCommand[idx] = 0.0f;
        lastCommandedLevel[idx] = 0;
        manualLevel[idx] = 0;
        if (mode != MODE_REMOTE) {
            applyTriacLevel(idx, 0, true);
        }
    }
}

void queueManualApply(bool forceAll) {
    if (mode == MODE_REMOTE) return;
    manualApplyPending = true;
    manualForceSend |= forceAll;
}

void applyManualLevels() {
    if (mode == MODE_REMOTE) return;
    for (int i = 0; i < 3; ++i) {
        pidCommand[i] = manualLevel[i];
        lastCommandedLevel[i] = manualLevel[i];
        applyTriacLevel(i, manualLevel[i], manualForceSend);
    }
    manualApplyPending = false;
    manualForceSend = false;
}

void handleManualArray(const JsonArray& arr, bool isRemoteContext) {
    if (arr.isNull()) return;
    for (size_t i = 0; i < 3 && i < arr.size(); ++i) {
        manualLevel[i] = arr[i].as<int>();
    }
    Serial.printf("[WS] manual payload -> %d %d %d\n", manualLevel[0], manualLevel[1], manualLevel[2]);

    if (isRemoteContext) {
        for (int i = 0; i < 3; ++i)
            lastCommandedLevel[i] = manualLevel[i];
        return;
    }

    if (mode != MODE_MANUAL)
        setMode(MODE_MANUAL);

    queueManualApply(true);
}

void handleTriacLevelCommand(int idx, int lvl, bool isRemoteContext) {
    if (idx < 0 || idx > 2) return;
    manualLevel[idx] = lvl;
    Serial.printf("[WS] triac direct idx=%d level=%d\n", idx, lvl);

    if (isRemoteContext) {
        lastCommandedLevel[idx] = lvl;
        return;
    }

    if (mode != MODE_MANUAL)
        setMode(MODE_MANUAL);

    queueManualApply(true);
}

void handleEnableVariant(const JsonVariant& en, bool hasTriac, int triacIdx) {
    if (en.is<JsonArray>()) {
        JsonArray arr = en.as<JsonArray>();
        for (size_t i = 0; i < 3 && i < arr.size(); ++i)
            setTriacEnabled(i, arr[i].as<bool>());
    } else {
        bool v = en.as<bool>();
        if (hasTriac && triacIdx >= 0 && triacIdx < 3)
            setTriacEnabled(triacIdx, v);
        else
            for (int i = 0; i < 3; ++i)
                setTriacEnabled(i, v);
    }
}

void handleCorrectionArray(const JsonArray& arr) {
    if (arr.isNull()) return;
    unsigned long stamp = millis();
    for (size_t i = 0; i < 3 && i < arr.size(); ++i) {
        correctionFeed[i] = arr[i].as<float>();
        correctionStamp[i] = stamp;
    }
    Serial.printf("[WS] correction -> %.1f %.1f %.1f\n",
                  correctionFeed[0], correctionFeed[1], correctionFeed[2]);
}

int computePidLevel(int idx, float measurement, float target, float dt) {
    if (!triacEnabled[idx]) {
        pidState[idx].integral = 0.0f;
        pidState[idx].prevErr = 0.0f;
        pidCommand[idx] = 0.0f;
        return 0;
    }

    float err = target - measurement;
    float absErr = fabs(err);

    if (absErr < PID_HOLD_BAND) {
        pidState[idx].integral *= 0.98f;
        pidState[idx].prevErr = err;
        return (int)round(pidCommand[idx]);
    }

    pidState[idx].integral += err * dt;
    pidState[idx].integral = constrain(pidState[idx].integral,
                                       -PID_INTEGRAL_CLAMP, PID_INTEGRAL_CLAMP);

    float derivative = (err - pidState[idx].prevErr) / max(dt, PID_D_MIN_DT);
    pidState[idx].prevErr = err;

    float kp = Kp_pid;
    if (absErr > 400.0f) kp *= 2.5f;
    else if (absErr > 200.0f) kp *= 1.6f;

    float delta = kp * err + Ki_pid * pidState[idx].integral + Kd_pid * derivative;
    float candidate = pidCommand[idx] + delta;
    candidate = constrain(candidate, 0.0f, 1000.0f);

    pidCommand[idx] = pidCommand[idx] + (candidate - pidCommand[idx]) * PID_OUTPUT_ALPHA;
    return (int)round(pidCommand[idx]);
}

void sendLog(float s[3], int lvl[3]) {
    DynamicJsonDocument doc(256);
    for (int i = 0; i < 3; i++) {
        doc["s"][i]          = s[i];
        doc["level"][i]      = lvl[i];
        doc["correction"][i] = correctionFeed[i];
    }
    doc["mode"] = mode;

    String body;
    serializeJson(doc, body);

    HTTPClient http;
    String url = String("http://") + SERVER_IP + ":" + String(SERVER_PORT) + "/log";
    http.begin(url);
    http.addHeader("Content-Type", "application/json");
    int code = http.POST(body);
    String resp = http.getString();
    Serial.printf("[LOG POST] %s -> code=%d\n", url.c_str(), code);
    Serial.print("[LOG POST] resp: ");
    Serial.println(resp);
    http.end();
}

void dimmingLoop() {
    static unsigned long lastLoop = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastLoop < LOOP_TS_MS) return;
    float dt = (lastLoop == 0) ? (LOOP_TS_MS / 1000.0f)
                               : (nowMs - lastLoop) / 1000.0f;
    lastLoop = nowMs;

    float rawSensors[3] = {
        (float)analogRead(SENSOR_PIN_1),
        (float)analogRead(SENSOR_PIN_2),
        (float)analogRead(SENSOR_PIN_3)
    };

    for (int i = 0; i < 3; ++i) {
        if (pidState[i].filtered == 0.0f)
            pidState[i].filtered = rawSensors[i];
        else
            pidState[i].filtered += SENSOR_ALPHA * (rawSensors[i] - pidState[i].filtered);
    }

    if (mode == MODE_REMOTE) {
        sendLog(rawSensors, lastCommandedLevel);
        return;
    }

    if (manualApplyPending)
        applyManualLevels();

    for (int i = 0; i < 3; ++i) {
        int levelOut = lastCommandedLevel[i];

        if (!triacEnabled[i]) {
            levelOut = 0;
            pidCommand[i] = 0.0f;
        }
        else if (mode == MODE_MANUAL) {
            levelOut = manualLevel[i];
        }
        else if (mode == MODE_AUTO) {
            levelOut = computePidLevel(i, pidState[i].filtered, targetLux[i], dt);
        }
        else if (mode == MODE_TEST) {
            float effectiveTarget = targetLux[i] - correctionFeed[i];
            if (effectiveTarget < 0.0f) effectiveTarget = 0.0f;
            levelOut = computePidLevel(i, pidState[i].filtered, effectiveTarget, dt);
        }
        else if (mode == MODE_RECORD) {
            levelOut = 0;
        }

        levelOut = constrain(levelOut, 0, 1000);
        lastCommandedLevel[i] = levelOut;

        if (mode == MODE_AUTO || mode == MODE_TEST || mode == MODE_MANUAL) {
            bool force = (mode == MODE_MANUAL);
            applyTriacLevel(i, levelOut, force);
        } else if (mode == MODE_RECORD) {
            applyTriacLevel(i, 0, true);
        }
    }

    sendLog(rawSensors, lastCommandedLevel);
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        Serial.print("[WS RX] ");
        Serial.println((const char*)payload);

        DynamicJsonDocument doc(2048);
        DeserializationError err = deserializeJson(doc, payload);
        if (err) {
            Serial.print("[WS RX] JSON parse error: ");
            Serial.println(err.c_str());
            return;
        }

        if (doc.containsKey("mode")) {
            setMode(doc["mode"].as<int>());
        }

        bool isRemote = (mode == MODE_REMOTE);

        if (doc.containsKey("manual")) {
            JsonArray arr = doc["manual"].as<JsonArray>();
            handleManualArray(arr, isRemote);
        }

        if (doc.containsKey("triac") && doc.containsKey("level")) {
            int idx = doc["triac"].as<int>();
            int lvl = doc["level"].as<int>();
            handleTriacLevelCommand(idx, lvl, isRemote);
        }

        if (doc.containsKey("switch_all")) {
            bool on = doc["switch_all"].as<bool>();
            for (int i = 0; i < 3; ++i)
                manualLevel[i] = on ? 1000 : 0;
            if (!isRemote) {
                setMode(MODE_MANUAL);
                queueManualApply(true);
            }
        }

        if (doc.containsKey("enable")) {
            bool hasTriac = doc.containsKey("triac");
            int triacIdx = hasTriac ? doc["triac"].as<int>() : -1;
            handleEnableVariant(doc["enable"], hasTriac, triacIdx);
        }

        if (doc.containsKey("correction")) {
            JsonArray arr = doc["correction"].as<JsonArray>();
            handleCorrectionArray(arr);
        }

        if (doc.containsKey("targetLux")) {
            JsonVariant t = doc["targetLux"];
            if (t.is<JsonArray>()) {
                JsonArray arr = t.as<JsonArray>();
                for (size_t i = 0; i < 3 && i < arr.size(); ++i)
                    targetLux[i] = arr[i].as<float>();
            } else {
                float v = t.as<float>();
                for (int i = 0; i < 3; ++i)
                    targetLux[i] = v;
            }
            Serial.printf("[WS] targetLux -> %.1f %.1f %.1f\n", targetLux[0], targetLux[1], targetLux[2]);
        }

        if (doc.containsKey("Kp")) {
            Kp_pid = doc["Kp"].as<float>();
            Serial.printf("[WS] Kp=%f\n", Kp_pid);
        }
        if (doc.containsKey("Ki")) {
            Ki_pid = doc["Ki"].as<float>();
            Serial.printf("[WS] Ki=%f\n", Ki_pid);
        }
        if (doc.containsKey("Kd")) {
            Kd_pid = doc["Kd"].as<float>();
            Serial.printf("[WS] Kd=%f\n", Kd_pid);
        }

    } else if (type == WStype_CONNECTED) {
        Serial.println("[WS] connected");
    } else if (type == WStype_DISCONNECTED) {
        Serial.println("[WS] disconnected");
    }
}

void setupWebSocket() {
    webSocket.begin(SERVER_IP, SERVER_PORT, "/ws");
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);
}

void syncTimeWithNTP() {
    configTime(0, 0, "pool.ntp.org", "time.google.com");
    delay(1500);
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi OK");

    syncTimeWithNTP();

    if (!get_token_and_sync()) {
        Serial.println("[TUYA] Warning: token retrieval failed");
    }

    setupWebSocket();

    for (int i = 0; i < 3; ++i) {
        prevSentLevel[i] = -1;
        lastSendTime[i] = 0;
    }
}

void loop() {
    webSocket.loop();
    dimmingLoop();
}
