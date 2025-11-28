/* triac_controller_remote_mode.ino
   Режимы:
   0 = AUTO   (PID, triacEnabled[i] учитывается)
   1 = RECORD (только лог)
   2 = TEST   (triacEnabled[i] учитывается)
   3 = MANUAL (по WebSocket)
   4 = REMOTE (ESP не трогает Tuya, управляет Python)
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <mbedtls/md.h>
#include <map>
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
const String TRIAC_3 = "bfc0ad553a9d0a8c07tcov";   // выровнял с Python-сервером

// ===================== Server =====================
const char* SERVER_IP   = "100.105.5.95";
const int   SERVER_PORT = 5000;

// ===================== Sensors =====================
#define SENSOR_PIN_1 34
#define SENSOR_PIN_2 35
#define SENSOR_PIN_3 32

// ===================== Timing =====================
const int Ts = 10;  // ms loop period
unsigned long lastUpdate = 0;

// ===================== Modes =====================
const int MODE_AUTO   = 0;
const int MODE_RECORD = 1;
const int MODE_TEST   = 2;
const int MODE_MANUAL = 3;
const int MODE_REMOTE = 4;

// ✅ Стартуем в REMOTE, чтобы ESP сам не трогал Tuya при включении
int mode = MODE_REMOTE;

// Состояние
int   manualLevel[3]  = {300,300,300};
bool  triacEnabled[3] = {true,true,true};
float correction[3]   = {0,0,0};

volatile bool needApplyManual = false;

// ===================== Tuya token =====================
String access_token = "";
unsigned long long token_expire_ms = 0;
long long time_offset_ms = 0;

// ===================== WebSocket =====================
WebSocketsClient webSocket;

// ===================== Send filters & control =====================
int prevLevel[3]    = {-1,-1,-1};
int currentLevel[3] = {300,300,300};
const int LEVEL_CHANGE_THRESHOLD = 2;
unsigned long lastSendTime[3] = {0,0,0};
const unsigned long MIN_SEND_INTERVAL_MS = 80;

// ===================== PID variables =====================
float Kp_pid = 0.05;
float Ki_pid = 0.0;
float Kd_pid = 0.0;

float targetLux[3] = {1000.0, 1000.0, 1000.0};
float integral_pid[3] = {0.0,0.0,0.0};
float prevErr_pid[3]  = {0.0,0.0,0.0};

// 🔧 PID настройки
const float PID_DEADBAND       = 94.0f;   // мёртвая зона по ошибке
const float PID_ERR_MED        = 200.0f;  // порог средней ошибки
const float PID_ERR_BIG        = 500.0f;  // порог большой ошибки
const float PID_INTEGRAL_MAX   = 5000.0f; // ограничение интеграла
const int   PID_LEVEL_MIN      = 0;
const int   PID_LEVEL_MAX      = 1000;
const int   PID_LEVEL_STEP_MAX = 30;      // макс. изменение уровня за 1 цикл (Ts=10мс)

// ===================== Utilities: SHA/HMAC =====================
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
    for (int i=0; i<32; i++) sprintf(hex+i*2, "%02x", hash[i]);
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
    for (int i=0; i<32; i++) sprintf(hex+i*2, "%02X", hash[i]);
    hex[64] = 0;
    return String(hex);
}

unsigned long long now_epoch_ms() {
    time_t sec = time(NULL);
    if (sec <= 0) return millis();
    return ((unsigned long long)sec)*1000ULL;
}

// ===================== HTTP helpers =====================
bool httpGetWithHeaders(const String& host, const String& path,
                        const std::map<String,String>& headers, String& out) {
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
                         const std::map<String,String>& headers,
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

// ===================== Tuya token =====================
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

    std::map<String,String> headers;
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
                      + (unsigned long long)expire_time*1000ULL
                      - 600000ULL;

    Serial.printf("[TUYA] Token OK, expires in %d s\n", expire_time);
    return true;
}

std::map<String,String> build_headers(const String& method,
                                      const String& path,
                                      const String& body_json) {
    if (access_token=="" || now_epoch_ms() > token_expire_ms)
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

// ===================== helper: normalize level =====================
int normalizeLevel(int inLevel) {
    if (inLevel <= 0) return -1; // OFF

    if (inLevel <= 100) { // percent
        int out = map(inLevel, 1, 100, 20, 1000);
        out = constrain(out, 20, 1000);
        return out;
    }
    return constrain(inLevel, 0, 1000);
}

// ===================== triac_set =====================
bool triac_set(int index, int inputLevel) {
    const String dev = (index == 0 ? TRIAC_1 :
                       (index == 1 ? TRIAC_2 : TRIAC_3));

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
            if (rdoc.containsKey("success"))
                ok = rdoc["success"].as<bool>();
            else
                ok = false;

            if (!ok) {
                String codeStr = "";
                String msgStr  = "";
                if (rdoc.containsKey("code"))
                    try { codeStr = rdoc["code"].as<String>(); } catch(...) {}
                if (rdoc.containsKey("msg"))
                    try { msgStr = rdoc["msg"].as<String>(); } catch(...) {}

                if (codeStr.length()) {
                    Serial.print("[TUYA] code=");
                    Serial.println(codeStr);
                }
                if (msgStr.length()) {
                    Serial.print("[TUYA] msg=");
                    Serial.println(msgStr);
                }
                serializeJsonPretty(rdoc, Serial);
            }
        }
    }

    Serial.printf("[TUYA] triac_set result: %d\n", ok ? 1 : 0);
    return ok;
}

// ===================== WebSocket handling =====================
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

        // --- режим ---
        if (doc.containsKey("mode")) {
            int m = doc["mode"].as<int>();
            if (m < 0) m = 0;
            if (m > 4) m = 4;
            mode = m;
            Serial.printf("[WS] set mode=%d\n", mode);
        }

        // --- manual ---
        if (doc.containsKey("manual")) {
            JsonArray arr = doc["manual"].as<JsonArray>();
            if (!arr.isNull()) {
                for (size_t i=0; i<3 && i < arr.size(); ++i) {
                    manualLevel[i]  = arr[i].as<int>();
                    currentLevel[i] = manualLevel[i];
                    prevLevel[i]    = -1;
                    lastSendTime[i] = 0;
                }
                if (mode != MODE_REMOTE) {
                    mode = MODE_MANUAL;
                }
                needApplyManual = true;
                Serial.printf("[WS] manual stored: %d %d %d, mode=%d\n",
                              manualLevel[0], manualLevel[1], manualLevel[2], mode);
            }
        }

        // --- triac + level ---
        if (doc.containsKey("triac") && doc.containsKey("level")) {
            int idx = doc["triac"].as<int>();
            int lvl = doc["level"].as<int>();
            if (idx>=0 && idx<3) {
                manualLevel[idx]  = lvl;
                currentLevel[idx] = lvl;
                prevLevel[idx]    = -1;
                lastSendTime[idx] = 0;
                if (mode != MODE_REMOTE) {
                    mode = MODE_MANUAL;
                }
                needApplyManual = true;
                Serial.printf("[WS] triac direct stored idx=%d level=%d, mode=%d\n",
                              idx, lvl, mode);
            }
        }

        // --- enable (исправлено: учитываем triac, если он передан) ---
        if (doc.containsKey("enable")) {
            JsonVariant en = doc["enable"];

            if (en.is<JsonArray>()) {
                // Массив enable для всех каналов
                JsonArray arr = en.as<JsonArray>();
                for (size_t i = 0; i < 3 && i < arr.size(); ++i) {
                    triacEnabled[i] = arr[i].as<bool>();
                }
            } else {
                // Одиночный enable + возможно triac
                bool v = en.as<bool>();

                if (doc.containsKey("triac")) {
                    int idx = doc["triac"].as<int>();
                    if (idx >= 0 && idx < 3) {
                        triacEnabled[idx] = v;
                    }
                } else {
                    // Если triac не указан, меняем все
                    for (int i = 0; i < 3; i++) {
                        triacEnabled[i] = v;
                    }
                }
            }

            Serial.printf("[WS] enable set: %d %d %d\n",
                          triacEnabled[0], triacEnabled[1], triacEnabled[2]);
        }

        // --- correction ---
        if (doc.containsKey("correction")) {
            JsonArray arr = doc["correction"].as<JsonArray>();
            if (!arr.isNull()) {
                for (size_t i = 0; i < 3 && i < arr.size(); ++i) {
                    correction[i] = arr[i].as<float>();
                }
                Serial.printf("[WS] correction set: %.3f %.3f %.3f\n",
                              correction[0], correction[1], correction[2]);
            }
        }

        // --- targetLux ---
        if (doc.containsKey("targetLux")) {
            JsonVariant t = doc["targetLux"];
            if (t.is<JsonArray>()) {
                JsonArray arr = t.as<JsonArray>();
                for (size_t i=0; i<3 && i<arr.size(); ++i) {
                    targetLux[i] = arr[i].as<float>();
                }
            } else {
                float v = t.as<float>();
                for (int i=0; i<3; i++)
                    targetLux[i] = v;
            }
            Serial.printf("[WS] targetLux set: %.2f %.2f %.2f\n",
                          targetLux[0], targetLux[1], targetLux[2]);
        }

        // --- PID params ---
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

// ===================== Log send to server =====================
void sendLog(float s[3], int lvl[3]) {
    DynamicJsonDocument doc(256);
    for (int i=0; i<3; i++) {
        doc["s"][i]          = s[i];
        doc["level"][i]      = lvl[i];
        doc["correction"][i] = correction[i];
    }
    doc["mode"] = mode;

    String body;
    serializeJson(doc, body);

    HTTPClient http;
    String url = String("http://") + SERVER_IP + ":" + String(SERVER_PORT) + "/log";
    http.begin(url);
    http.addHeader("Content-Type","application/json");
    int code = http.POST(body);
    String resp = http.getString();
    Serial.printf("[LOG POST] %s -> code=%d\n", url.c_str(), code);
    Serial.print("[LOG POST] resp: ");
    Serial.println(resp);
    http.end();
}

// ===================== Main loop / dimming =====================
void dimmingLoop() {
    unsigned long now = millis();
    if (now - lastUpdate < Ts) return;
    lastUpdate = now;

    float s[3] = {
        (float)analogRead(SENSOR_PIN_1),
        (float)analogRead(SENSOR_PIN_2),
        (float)analogRead(SENSOR_PIN_3)
    };
    int level[3];
    float dt = Ts / 1000.0f;

    // REMOTE: не трогаем Tuya, только шлём лог
    if (mode == MODE_REMOTE) {
        for (int i=0; i<3; i++) {
            level[i] = currentLevel[i];
        }
        sendLog(s, level);
        return;
    }

    // Одноразовое применение manual
    if (needApplyManual && mode != MODE_REMOTE) {
        Serial.println("[APPLY MANUAL] Applying manual levels for all triacs");
        for (int i=0; i<3; i++) {
            int mlevel     = manualLevel[i];
            int normalized = normalizeLevel(mlevel);
            int toCompare  = (normalized < 0) ? 0 : normalized;

            bool ok = triac_set(i, mlevel);
            if (ok) {
                prevLevel[i]    = toCompare;
                lastSendTime[i] = millis();
                currentLevel[i] = mlevel;
            }
            Serial.printf("[APPLY MANUAL] triac %d -> level=%d norm=%d ok=%d\n",
                          i, mlevel, toCompare, ok ? 1 : 0);
        }
        needApplyManual = false;
    }

    // Расчёт уровня
    for (int i=0; i<3; i++) {

        if (mode == MODE_AUTO) {
            // если чекбокс выключен – гасим канал и сбрасываем PID
            if (!triacEnabled[i]) {
                level[i]        = 0;
                currentLevel[i] = 0;
                integral_pid[i] = 0.0f;
                prevErr_pid[i]  = 0.0f;
            } else {
                // 👉 используем коррекцию: sensor - correction
                float measured = s[i] - correction[i];
                if (measured < 0.0f) measured = 0.0f;

                // ошибка: target - measured
                float err    = targetLux[i] - measured;
                float absErr = fabs(err);

                // 🔹 Мёртвая зона: близко к цели – держим уровень, плавно сдуваем интеграл
                if (absErr < PID_DEADBAND) {
                    integral_pid[i] *= 0.9f;    // чуть уменьшаем, чтобы не накапливался
                    level[i] = currentLevel[i];
                } else {
                    // 🔹 Динамический Kp: при большом diff реагируем быстрее
                    float Kp_eff = Kp_pid;
                    if (absErr > PID_ERR_BIG) {
                        Kp_eff = Kp_pid * 3.0f;
                    } else if (absErr > PID_ERR_MED) {
                        Kp_eff = Kp_pid * 1.5f;
                    }

                    // 🔹 anti-windup для интеграла
                    bool atUpper = (currentLevel[i] >= PID_LEVEL_MAX - 5);
                    bool atLower = (currentLevel[i] <= PID_LEVEL_MIN + 5);
                    if (!((atUpper && err > 0.0f) || (atLower && err < 0.0f))) {
                        integral_pid[i] += err * dt;
                        if (integral_pid[i] >  PID_INTEGRAL_MAX) integral_pid[i] =  PID_INTEGRAL_MAX;
                        if (integral_pid[i] < -PID_INTEGRAL_MAX) integral_pid[i] = -PID_INTEGRAL_MAX;
                    }

                    // производная
                    float derivative = (err - prevErr_pid[i]) / (dt > 0.0f ? dt : 1.0f);
                    prevErr_pid[i]   = err;

                    float delta = Kp_eff * err
                                  + Ki_pid * integral_pid[i]
                                  + Kd_pid * derivative;

                    // желаемый новый уровень (пока без ограничений скорости)
                    float newLevelF = (float)currentLevel[i] + delta;

                    // 🔹 ограничение скорости изменения яркости
                    float step = newLevelF - currentLevel[i];
                    if (step >  PID_LEVEL_STEP_MAX) step =  PID_LEVEL_STEP_MAX;
                    if (step < -PID_LEVEL_STEP_MAX) step = -PID_LEVEL_STEP_MAX;
                    newLevelF = (float)currentLevel[i] + step;

                    int newLevel = constrain((int)round(newLevelF), PID_LEVEL_MIN, PID_LEVEL_MAX);
                    currentLevel[i] = newLevel;
                    level[i]        = newLevel;
                }
            }
        }
        else if (mode == MODE_MANUAL) {
            level[i]        = manualLevel[i];
            currentLevel[i] = manualLevel[i];
        }
        else if (mode == MODE_RECORD) {
            level[i]        = 0;
            currentLevel[i] = 0;
        }
        else if (mode == MODE_TEST) {
            float val = s[i] - correction[i];
            level[i] = triacEnabled[i]
                       ? constrain(map((int)val, 0, 4095, 20, 1000), 0, 1000)
                       : 0;
            currentLevel[i] = level[i];
        }
        else {
            level[i] = currentLevel[i];
        }

        // Отправка в Tuya для AUTO/MANUAL/TEST
        if (mode == MODE_AUTO || mode == MODE_MANUAL || mode == MODE_TEST) {
            int normalized = normalizeLevel(level[i]);
            int toCompare  = (normalized < 0) ? 0 : normalized;
            unsigned long nowMs = millis();
            bool timeOk = (nowMs - lastSendTime[i]) >= MIN_SEND_INTERVAL_MS;

            if ((prevLevel[i] < 0 || abs(toCompare - prevLevel[i]) > LEVEL_CHANGE_THRESHOLD) && timeOk) {
                bool ok = triac_set(i, level[i]);
                if (ok) {
                    prevLevel[i]    = (normalized < 0) ? 0 : normalized;
                    lastSendTime[i] = nowMs;
                }
                Serial.printf("[LOOP] triac %d -> input=%d normalized=%d sent=%d\n",
                              i, level[i], prevLevel[i], ok ? 1 : 0);
            }
        }
    }

    sendLog(s, level);
}

// ===================== Time sync =====================
void syncTimeWithNTP() {
    configTime(0,0,"pool.ntp.org","time.google.com");
    delay(1500);
}

// ===================== Setup / Loop =====================
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

    // ✅ Стартуем с нуля по уровням и сбросом PID
    for (int i=0; i<3; i++) {
        manualLevel[i]  = 0;
        currentLevel[i] = 0;
        prevLevel[i]    = -1;
        lastSendTime[i] = 0;

        integral_pid[i] = 0.0f;
        prevErr_pid[i]  = 0.0f;
    }
}

void loop() {
    webSocket.loop();
    dimmingLoop();
}
