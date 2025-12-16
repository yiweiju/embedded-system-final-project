#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include "qrcode.h"
#include <ArduinoJson.h>
#include <time.h>
#include <vector>
#include <algorithm>
#include <ctype.h>
#include <stdlib.h>

// ---------- Config ----------
static const char *AP_SSID = "ESP32_SETUP";
static const char *AP_PASSWORD = "88888888";  // 用于手机扫码加入热点
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GATEWAY(192, 168, 4, 1);
static const IPAddress AP_NETMASK(255, 255, 255, 0);
static const int UART_RX_PIN = 25;  // ESP32 -> TM4C UART0_TX
static const int UART_TX_PIN = 26;  // ESP32 <- TM4C UART0_RX
static const uint32_t UART_BAUD = 115200;

// ---------- Display ----------
TFT_eSPI tft = TFT_eSPI();
static const uint16_t COLOR_BG = 0xFFDD;    // RGB565 of #FFF8F0
static const uint16_t COLOR_TEXT = 0x6A68;  // RGB565 of #6D4C41
static const uint16_t COLOR_BLACK = TFT_BLACK;

// Dashboard palette (RGB565)
static const uint16_t DASH_BG = 0xFFDF;      // #FFF8F0 close
static const uint16_t DASH_PRIMARY = 0xFDB6; // #FFB7B2
static const uint16_t DASH_ALERT = 0xFA4A;   // #FF5252
static const uint16_t DASH_WHITE = TFT_WHITE;
static const uint16_t DASH_TEXT = COLOR_TEXT;

// ---------- QR generation ----------
static constexpr uint8_t QR_VERSION = 6;
static constexpr size_t QR_BUF_SIZE = ((4 * QR_VERSION + 17) * (4 * QR_VERSION + 17) + 7) / 8;

struct QRSet {
    QRCode qr;
    uint8_t buffer[QR_BUF_SIZE];
    String payload;
};

QRSet qrJoinAp;
QRSet qrConfigPage;
QRSet qrMainPage;

void makeQr(QRSet &out, const String &payload) {
    out.payload = payload;
    memset(out.buffer, 0, sizeof(out.buffer));
    qrcode_initText(&out.qr, out.buffer, QR_VERSION, ECC_MEDIUM, out.payload.c_str());
}

// ---------- State ----------
enum class Stage { JOIN_AP, CONFIG_URL, ONLINE };
Stage currentStage = Stage::JOIN_AP;

enum class DisplayMode { QR, DASH_STATUS, DASH_SCHEDULE };
DisplayMode displayMode = DisplayMode::QR;

String homeSsid;
String homePass;
IPAddress homeIp;
bool hasStoredCreds = false;
Preferences prefs;
std::vector<String> cachedSsids;
bool scanInProgress = false;
bool scanRequested = false;

AsyncWebServer server(80);
HardwareSerial &tm4cSerial = Serial1;
uint32_t lastStatusPollMs = 0;
uint32_t lastSchedulePollMs = 0;
DisplayMode lastDisplayMode = DisplayMode::QR;
bool statusPollActive = false;
bool scheduleNeedsFetch = false;
uint32_t lastAutoSettimeMs = 0;
bool timeDesyncWarning = false;
constexpr uint32_t TIME_DRIFT_THRESHOLD_SEC = 120;      // allowed diff between device and ESP32 local time
uint32_t lastNtpSyncMs = 0;
String lastQrTimeShown;
bool lastQrTimeWarn = false;

// Button to cycle display pages (active LOW, internal pull-up)
constexpr int DISPLAY_BTN_PIN = 15;  // default BOOT button; adjust if needed
bool lastBtnState = true;
unsigned long lastBtnMs = 0;
bool displayDirty = false;

// ---------- Device data (updated via TM4C UART) ----------
struct ScheduleItem {
    String time;
    String amount;
};

// Async schedule update task
enum class ScheduleTaskState { IDLE, PENDING, PROCESSING, SUCCESS, FAILED };
struct ScheduleTask {
    ScheduleTaskState state = ScheduleTaskState::IDLE;
    std::vector<ScheduleItem> pendingSchedule;
    String errorMessage;
    uint32_t startMs = 0;
    int retryCount = 0;
};
ScheduleTask scheduleTask;

struct StatusData {
    int foodBowlG = 45;   // grams
    int waterBowlG = 120; // grams
};

StatusData statusData;
std::vector<ScheduleItem> scheduleData = {
        {"08:00", "M"},
        {"18:00", "H"}
};

int32_t timezoneOffsetSeconds = 0;  // default UTC

// ---------- TM4C UART AT command client ----------
struct Tm4cLink {
    String asyncBuf;

    static bool parseLocalTimeString(const String &ts, time_t &outLocalEpoch) {
        int year, mon, day, hh, mm, ss;
        if (sscanf(ts.c_str(), "%d-%d-%d %d:%d:%d", &year, &mon, &day, &hh, &mm, &ss) != 6) {
            return false;
        }
        struct tm t{};
        t.tm_year = year - 1900;
        t.tm_mon = mon - 1;
        t.tm_mday = day;
        t.tm_hour = hh;
        t.tm_min = mm;
        t.tm_sec = ss;
        time_t utc = mktime(&t);
        if (utc == (time_t) -1) return false;
        outLocalEpoch = utc + timezoneOffsetSeconds;
        return true;
    }

    void begin() {
        // Increase RX buffer to 512 bytes to prevent data loss
        tm4cSerial.setRxBufferSize(512);
        tm4cSerial.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    }

    void sendCurrentTime() {
        time_t unixUtc = time(nullptr);
        uint32_t unixLocal = static_cast<uint32_t>(unixUtc + timezoneOffsetSeconds);
        tm4cSerial.printf("AT+SETTIME=%lu\r\n", static_cast<unsigned long>(unixLocal));
        Serial.printf("[UART] -> AT+SETTIME=%lu (tz=%d)\n", static_cast<unsigned long>(unixLocal), timezoneOffsetSeconds);
    }

    void handleAsyncLine(const String &line) {
        if (line.startsWith("AT+GETTIME")) {
            Serial.println("[UART] <- AT+GETTIME (async)");
            sendCurrentTime();
        } else if (line.length()) {
            Serial.printf("[UART] Async line ignored: %s\n", line.c_str());
        }
    }

    void poll() {
        while (tm4cSerial.available()) {
            char c = static_cast<char>(tm4cSerial.read());
            if (c == '\r') continue;
            if (c == '\n') {
                if (asyncBuf.length()) {
                    Serial.printf("[UART] <- %s\n", asyncBuf.c_str());
                    handleAsyncLine(asyncBuf);
                    asyncBuf = "";
                }
            } else {
                if (asyncBuf.length() < 256) asyncBuf += c;
            }
        }
    }

    bool sendAtCommand(const String &cmd, String &payload, String &err, uint32_t timeoutMs = 600, bool slowSend = false, uint8_t slowDelayMs = 2, uint8_t slowDelaySepMs = 4) {
        while (tm4cSerial.available()) tm4cSerial.read();  // clear stale bytes
        Serial.printf("[UART] -> %s\n", cmd.c_str());
        String wire = cmd;
        if (!wire.endsWith("\r\n")) wire += "\r\n";

        if (slowSend) {
            for (size_t i = 0; i < wire.length(); ++i) {
                char c = wire.charAt(i);
                tm4cSerial.write(static_cast<uint8_t>(c));
                if (c == ';') {
                    delay(slowDelaySepMs);  // brief pause between schedule entries
                } else {
                    delay(slowDelayMs);  // gentle pacing to avoid UART overruns
                }
            }
        } else {
            tm4cSerial.print(wire);
        }

        String cmdEcho = cmd;
        while (cmdEcho.endsWith("\r") || cmdEcho.endsWith("\n")) {
            cmdEcho.remove(cmdEcho.length() - 1);
        }

        unsigned long start = millis();
        String line;
        while (millis() - start < timeoutMs) {
            while (tm4cSerial.available()) {
                char c = static_cast<char>(tm4cSerial.read());
                if (c == '\r') continue;
                if (c == '\n') {
                    if (line.length() == 0) continue;
                    if (line.startsWith("AT+GETTIME")) {
                        Serial.println("[UART] <- AT+GETTIME (during wait)");
                        sendCurrentTime();
                        line = "";
                        continue;
                    }
                    if (line == cmdEcho) {
                        Serial.println("[UART] <- (echo)");
                        line = "";
                        continue;
                    }
                    if (line.startsWith("+OK")) {
                        int colon = line.indexOf(':');
                        payload = (colon >= 0) ? line.substring(colon + 1) : "";
                        payload.trim();
                        Serial.printf("[UART] <- +OK%s%s\n", colon >= 0 ? ": " : "", payload.c_str());
                        return true;
                    }
                    if (line.startsWith("+ERR")) {
                        err = line.substring(line.indexOf(':') + 1);
                        err.trim();
                        if (err.length() == 0) err = "ERR";
                        Serial.printf("[UART] <- +ERR: %s\n", err.c_str());
                        return false;
                    }
                    Serial.printf("[UART] <- %s (ignored)\n", line.c_str());
                    // Unknown line, ignore and keep waiting
                    line = "";
                } else {
                    if (line.length() < 256) line += c;
                }
            }
            delay(2);  // yield to avoid starving watchdog while waiting
        }
        err = "timeout";
        return false;
    }

    bool getStatus(StatusData &out, String &err) {
        String payload;
        if (!sendAtCommand("AT+STATUS", payload, err)) return false;
        // Expected: TIME=YYYY-MM-DD HH:MM:SS,BOWL=<g>,WATER=<g>,ALARM=<b>,BUSY=<b>
        int last = 0;
        String deviceTimeStr;
        while (true) {
            int comma = payload.indexOf(',', last);
            String part = (comma >= 0) ? payload.substring(last, comma) : payload.substring(last);
            part.trim();
            int eq = part.indexOf('=');
            if (eq > 0) {
                String key = part.substring(0, eq);
                String val = part.substring(eq + 1);
                key.trim(); val.trim();
                if (key == "TIME") deviceTimeStr = val;
                if (key == "BOWL") out.foodBowlG = val.toInt();
                if (key == "WATER") out.waterBowlG = val.toInt();
            }
            if (comma < 0) break;
            last = comma + 1;
        }
        if (deviceTimeStr.length()) {
            time_t devLocal, nowLocal;
            if (parseLocalTimeString(deviceTimeStr, devLocal)) {
                nowLocal = time(nullptr) + timezoneOffsetSeconds;
                long diff = labs((long) (nowLocal - devLocal));
                if (diff > TIME_DRIFT_THRESHOLD_SEC) {
                    Serial.printf("[UART] Detected time drift %ld sec, resyncing...\n", diff);
                    sendCurrentTime();
                }
                timeDesyncWarning = diff > TIME_DRIFT_THRESHOLD_SEC;
                if (!timeDesyncWarning) lastAutoSettimeMs = millis();
            } else {
                timeDesyncWarning = true;
            }
        } else {
            timeDesyncWarning = true;
        }
        return true;
    }

    static String amountToCode(const String &amt) {
        if (amt.length() == 0) return "";
        char c = toupper(amt.charAt(0));
        if (c == 'L' || c == 'M' || c == 'H') return String(c);
        return "";
    }

    bool getSchedule(std::vector<ScheduleItem> &out, String &err) {
        String payload;
        if (!sendAtCommand("AT+GETSCHED", payload, err)) return false;
        out.clear();
        payload.trim();
        if (payload.equalsIgnoreCase("NONE")) return true;
        if (payload.length() == 0) return true;

        // New format: 0700M;1200L;1900H
        int last = 0;
        while (true) {
            int semi = payload.indexOf(';', last);
            String entry = (semi >= 0) ? payload.substring(last, semi) : payload.substring(last);
            entry.trim();

            // Expected format: HHMM + L/M/H (e.g., "0700M", "1200L")
            if (entry.length() >= 5) {
                String timeStr = entry.substring(0, 4);  // HHMM
                String amountCode = amountToCode(entry.substring(4));  // L/M/H

                if (amountCode.length() > 0) {
                    // Convert HHMM to HH:MM
                    String time = timeStr.substring(0, 2) + ":" + timeStr.substring(2, 4);
                    out.push_back({time, amountCode});
                }
            }

            if (semi < 0) break;
            last = semi + 1;
        }
        return true;
    }

    bool setSchedule(const std::vector<ScheduleItem> &items, String &err, bool extraSlow = false) {
        String schedStr;
        for (size_t i = 0; i < items.size(); ++i) {
            const String code = amountToCode(items[i].amount);
            if (code.length() == 0) continue;
            if (schedStr.length()) schedStr += ";";

            // Convert HH:MM to HHMM format
            String timeStr = items[i].time;
            timeStr.replace(":", "");  // Remove colon: "07:00" -> "0700"

            // New format: HHMM + L/M/H (e.g., "0700M")
            schedStr += timeStr + code;
        }
        String payload;
        if (schedStr.length() == 0) {
            return sendAtCommand("AT+SCHED=NONE", payload, err);
        }
        // Long schedule strings can overrun TM4C UART; send slowly with extra timeout.
        uint8_t perCharDelay = extraSlow ? 6 : 2;
        uint8_t perSemiDelay = extraSlow ? 12 : 4;
        return sendAtCommand("AT+SCHED=" + schedStr, payload, err, 3000, true, perCharDelay, perSemiDelay);
    }

    bool feedNow(const String &level, String &err) {
        String code = amountToCode(level);
        if (code.length() == 0) {
            err = "invalid level";
            return false;
        }
        String payload;
        return sendAtCommand("AT+FEED=" + code, payload, err);
    }

    bool timeSync(uint32_t unixTs, int32_t /*tzOffset*/, String &err) {
        String payload;
        return sendAtCommand("AT+SETTIME=" + String(unixTs), payload, err);
    }
};

Tm4cLink tm4c;

String scheduleToJson() {
    String json = "[";
    for (size_t i = 0; i < scheduleData.size(); ++i) {
        if (i) json += ",";
        json += "{\"time\":\"" + scheduleData[i].time + "\",\"amount\":\"" + scheduleData[i].amount + "\"}";
    }
    json += "]";
    return json;
}

int clampReading(int v) {
    if (v < 0) return 0;
    if (v > 999) return 999;
    return v;
}

String statusToJson() {
    String json = "{";
    json += "\"foodBowl\":" + String(clampReading(statusData.foodBowlG)) + ",";
    json += "\"waterBowl\":" + String(clampReading(statusData.waterBowlG)) + ",";
    json += "\"timeWarn\":" + String(timeDesyncWarning ? "true" : "false");
    json += "}";
    return json;
}

// ---------- Display helper ----------
void drawQr(const char *title, const QRSet *qr, uint16_t qrColor, bool clearScreen = true) {
    if (clearScreen) {
        tft.fillScreen(COLOR_BG);
    }

    uint8_t size = qr ? qr->qr.size : 0;
    uint8_t scale = qr ? std::max<uint8_t>(1, std::min<uint8_t>((tft.width() - 20) / size, (tft.height() - 40) / size)) : 1;
    int qrPix = qr ? size * scale : 0;
    int x0 = (tft.width() - qrPix) / 2;
    int y0 = (tft.height() - qrPix) / 2 + 10;

    // Title directly above QR code
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setTextFont(1);
    tft.setTextSize(1);
    // Place title a bit above the QR with extra spacing
    int titleY = std::max(4, y0 - 22);
    tft.drawString(title, tft.width() / 2, titleY);

    if (!qr) {
        tft.drawString("Waiting...", tft.width() / 2, tft.height() / 2);
        return;
    }

    for (uint8_t y = 0; y < size; y++) {
        for (uint8_t x = 0; x < size; x++) {
            if (qrcode_getModule(const_cast<QRCode *>(&qr->qr), x, y)) {
                tft.fillRect(x0 + x * scale, y0 + y * scale, scale, scale, qrColor);
            }
        }
    }
}

void drawText(const char *line1, const char *line2 = nullptr) {
    tft.fillScreen(COLOR_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setTextFont(1);
    tft.setTextSize(2);
    int centerY = tft.height() / 2 - (line2 ? 12 : 0);
    tft.drawString(line1, tft.width() / 2, centerY);
    if (line2) tft.drawString(line2, tft.width() / 2, centerY + 24);
}

String currentTimeString() {
    struct tm timeinfo{};
    if (getLocalTime(&timeinfo, 500)) {
        char buf[6];
        strftime(buf, sizeof(buf), "%H:%M", &timeinfo);
        return String(buf);
    }
    return String("--:--");
}

constexpr int WATER_BOWL_LOW_G = 80;
bool hasAlertState() {
    int water = clampReading(statusData.waterBowlG);
    return water < WATER_BOWL_LOW_G;
}

void drawWifiIcon(int x, int y, bool connected, uint16_t color, uint16_t bg) {
    if (connected) {
        // Use right-top quadrant arcs for compact Wi-Fi icon
        tft.drawCircleHelper(x, y, 6, 0x1, color);
        tft.drawCircleHelper(x, y, 4, 0x1, color);
        tft.drawCircleHelper(x, y, 2, 0x1, color);
        tft.fillCircle(x, y, 1, color);
    } else {
        tft.fillRect(x - 5, y - 5, 10, 10, bg);
        tft.setTextColor(color, bg);
        tft.setCursor(x - 3, y - 4);
        tft.print("x");
    }
}

void drawStatusBarDash(bool alert) {
    uint16_t barColor = alert ? DASH_ALERT : DASH_PRIMARY;
    tft.fillRect(0, 0, tft.width(), 18, barColor);
    tft.setTextColor(DASH_WHITE, barColor);
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setCursor(4, 4);
    String now = currentTimeString();
    tft.print(now);
    if (timeDesyncWarning) {
        tft.setCursor(tft.width() / 2 - 16, 4);
        tft.print("TIME?");
    }
    drawWifiIcon(tft.width() - 12, 9, WiFi.status() == WL_CONNECTED, DASH_WHITE, barColor);
    Serial.printf("[UI] StatusBar time=%s wifi=%s alert=%d\n", now.c_str(),
                  WiFi.status() == WL_CONNECTED ? "on" : "off", alert);
}

void drawPageDotsGeneric(uint8_t total, uint8_t active) {
    int y = tft.height() - 6;
    int cx = tft.width() / 2;
    int spread = (total - 1) * 8;
    for (int i = 0; i < total; ++i) {
        int x = cx - spread / 2 + i * 8;
        tft.drawCircle(x, y, 2, DASH_PRIMARY);
        if (i == active) tft.fillCircle(x, y, 2, DASH_TEXT);
    }
}

void drawDashStatusPage() {
    tft.fillScreen(DASH_BG);
    bool alert = hasAlertState();
    drawStatusBarDash(alert);
    int foodVal = clampReading(statusData.foodBowlG);
    int waterVal = clampReading(statusData.waterBowlG);
    tft.setTextColor(DASH_TEXT, DASH_BG);
    tft.drawFastVLine(tft.width() / 2, 20, tft.height() - 28, DASH_PRIMARY);

    // Food Bowl
    tft.setTextFont(1); tft.setTextSize(1);
    tft.setCursor(4, 28); tft.print("Bowl(F)");
    tft.setTextSize(2);
    tft.setCursor(4, 48); tft.print(foodVal); tft.setTextSize(1); tft.print("g");

    // Water Bowl
    tft.setTextSize(1);
    int rightX = tft.width() / 2 + 4;
    tft.setCursor(rightX, 28); tft.print("Bowl(W)");
    tft.setTextSize(2);
    tft.setCursor(rightX, 48); tft.print(waterVal); tft.setTextSize(1); tft.print("g");
    if (waterVal < WATER_BOWL_LOW_G) {
        tft.setTextColor(DASH_ALERT, DASH_BG);
        tft.setCursor(rightX, 74); tft.print("REFILL");
        tft.setTextColor(DASH_TEXT, DASH_BG);
    }

    drawPageDotsGeneric(3, 1);
    Serial.printf("[UI] Page STATUS fb=%dg wb=%dg\n",
                  statusData.foodBowlG, statusData.waterBowlG);
}

void drawDashSchedulePage() {
    tft.fillScreen(DASH_BG);
    bool alert = hasAlertState();
    drawStatusBarDash(alert);
    tft.setTextColor(DASH_TEXT, DASH_BG);
    tft.setTextFont(1); tft.setTextSize(2);
    tft.setCursor(6, 30); tft.print("Plan");
    tft.setTextSize(1);

    auto toMinutes = [](const String &ts) -> int {
        int colon = ts.indexOf(':');
        if (colon < 0) return -1;
        int h = ts.substring(0, colon).toInt();
        int m = ts.substring(colon + 1).toInt();
        if (h < 0 || h > 23 || m < 0 || m > 59) return -1;
        return h * 60 + m;
    };

    int nowMin = toMinutes(currentTimeString());
    std::vector<std::pair<int, ScheduleItem>> ordered;
    ordered.reserve(scheduleData.size());
    for (auto &item : scheduleData) {
        int mins = toMinutes(item.time);
        if (mins >= 0) ordered.push_back({mins, item});
    }
    std::sort(ordered.begin(), ordered.end(),
              [](const std::pair<int, ScheduleItem> &a, const std::pair<int, ScheduleItem> &b) {
                  return a.first < b.first;
              });

    ScheduleItem prevItem;
    bool hasPrev = false;

    // find prev (last <= now) and start index for next
    size_t startNext = 0;
    if (!ordered.empty()) {
        for (size_t i = 0; i < ordered.size(); ++i) {
            if (ordered[i].first <= nowMin) {
                prevItem = ordered[i].second;
                hasPrev = true;
                startNext = (i + 1) % ordered.size();
            }
        }
    }

    std::vector<ScheduleItem> nextItems;
    nextItems.reserve(3);

    if (!ordered.empty()) {
        // Pass 1: unique items from startNext, wrapping once
        size_t idx = startNext;
        size_t visited = 0;
        while (nextItems.size() < 3 && visited < ordered.size()) {
            const ScheduleItem &cand = ordered[idx].second;
            bool dup = false;
            for (auto &n : nextItems) {
                if (n.time == cand.time && n.amount == cand.amount) { dup = true; break; }
            }
            if (!dup) nextItems.push_back(cand);
            idx = (idx + 1) % ordered.size();
            ++visited;
        }
        // Pass 2: fill remaining slots by cycling even if repeats are needed
        idx = startNext;
        while (nextItems.size() < 3) {
            nextItems.push_back(ordered[idx].second);
            idx = (idx + 1) % ordered.size();
        }
    }

    int y = 60;
    if (hasPrev) {
        tft.fillRoundRect(6, y, tft.width() - 12, 20, 4, DASH_WHITE);
        tft.setCursor(12, y + 6);
        tft.setTextColor(DASH_PRIMARY, DASH_WHITE);
        tft.print("Prev ");
        tft.setTextColor(DASH_TEXT, DASH_WHITE);
        tft.print(prevItem.time + " " + prevItem.amount);
        y += 24;
    }

    if (!nextItems.empty()) {
        tft.setTextColor(DASH_PRIMARY, DASH_BG);
        tft.setCursor(8, y - 2);
        tft.print("Next");
        tft.setTextColor(DASH_TEXT, DASH_BG);
    }

    for (size_t i = 0; i < nextItems.size() && y < tft.height() - 16; ++i) {
        tft.fillRoundRect(6, y, tft.width() - 12, 20, 4, DASH_WHITE);
        tft.setCursor(12, y + 6);
        tft.setTextColor(DASH_TEXT, DASH_WHITE);
        tft.print(nextItems[i].time + " " + nextItems[i].amount);
        y += 24;
    }

    if (!hasPrev && nextItems.empty()) {
        tft.setTextColor(DASH_TEXT, DASH_BG);
        tft.setCursor(10, y + 6);
        tft.print("No schedule");
    }

    drawPageDotsGeneric(3, 2);
    Serial.printf("[UI] Page SCHEDULE prev=%s nextCount=%u total=%u nowMin=%d\n",
                  hasPrev ? prevItem.time.c_str() : "none",
                  (unsigned)nextItems.size(), (unsigned)scheduleData.size(), nowMin);
}

void renderDashboard() {
    switch (displayMode) {
        case DisplayMode::DASH_STATUS:
            drawDashStatusPage();
            break;
        case DisplayMode::DASH_SCHEDULE:
            drawDashSchedulePage();
            break;
        default:
            break;
    }
    displayDirty = false;
}

void drawCurrentQrStage();

void renderCurrentDisplay() {
    if (displayMode == DisplayMode::QR) {
        drawCurrentQrStage();
    } else {
        renderDashboard();
    }
}

void cycleDisplayMode() {
    int mode = static_cast<int>(displayMode);
    mode = (mode + 1) % 3;
    displayMode = static_cast<DisplayMode>(mode);
    Serial.printf("[UI] Cycle display -> mode=%d\n", mode);
    renderCurrentDisplay();
}

void drawCurrentQrStage() {
    uint16_t qrColor = (currentStage == Stage::ONLINE) ? COLOR_TEXT : COLOR_BLACK;
    bool showTitle = currentStage != Stage::ONLINE;  // hide text when online
    bool showBarAndDots = currentStage == Stage::ONLINE;

    tft.fillScreen(COLOR_BG);

    if (showBarAndDots) {
        // Status bar on top of QR screens too
        uint16_t barColor = hasAlertState() ? DASH_ALERT : DASH_PRIMARY;
        tft.fillRect(0, 0, tft.width(), 18, barColor);
        tft.setTextColor(DASH_WHITE, barColor);
        tft.setTextFont(1);
        tft.setTextSize(1);
        tft.setCursor(4, 4);
        String now = currentTimeString();
        tft.print(now);
        drawWifiIcon(tft.width() - 12, 9, WiFi.status() == WL_CONNECTED, DASH_WHITE, barColor);

        // Clear content area below bar
        tft.fillRect(0, 18, tft.width(), tft.height() - 24, COLOR_BG);

        // Page dots: 3 pages (0:QR, 1:status, 2:schedule)
        drawPageDotsGeneric(3, 0);
    }

    // Draw QR (without clearing screen)
    switch (currentStage) {
        case Stage::JOIN_AP:
            drawQr(showTitle ? "1) Join AP" : "", &qrJoinAp, qrColor, false);
            Serial.println("[UI] Show Join-AP QR");
            break;
        case Stage::CONFIG_URL:
            drawQr(showTitle ? "2) Config Page" : "", &qrConfigPage, qrColor, false);
            Serial.println("[UI] Show Config URL QR");
            break;
        case Stage::ONLINE:
            drawQr("", &qrMainPage, qrColor, false);
            Serial.println("[UI] Show Online URL QR");
            break;
    }
    if (showBarAndDots) {
        lastQrTimeShown = currentTimeString();
        lastQrTimeWarn = timeDesyncWarning;
    }
}

void showStage(Stage stage) {
    currentStage = stage;
    if (displayMode == DisplayMode::QR) {
        drawCurrentQrStage();
    } else {
        displayDirty = true;  // refresh dash later
        Serial.printf("[UI] Stage changed -> queued refresh, stage=%d\n", static_cast<int>(stage));
    }
}

// ---------- Wi-Fi helpers ----------
void startAP() {
    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_NETMASK);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    Serial.printf("[WiFi] AP started SSID=%s PASS=%s\n", AP_SSID, AP_PASSWORD);
    // Keep Wi-Fi auto-connect format but short SSID/PWD; ensure SSID/PWD remain concise to maximize QR error tolerance.
    makeQr(qrJoinAp, String("WIFI:T:WPA;S:") + AP_SSID + ";P:" + AP_PASSWORD + ";;");
    makeQr(qrConfigPage, "http://192.168.4.1/");
    showStage(Stage::JOIN_AP);
}

void connectHome(const String &ssid, const String &pass) {
    homeSsid = ssid;
    homePass = pass;
    WiFi.begin(homeSsid.c_str(), homePass.c_str());
}

void fetchStatusOnce() {
    String err;
    if (tm4c.getStatus(statusData, err)) {
        displayDirty = true;
    } else {
        Serial.printf("[UART] get_status fail: %s\n", err.c_str());
    }
}

void fetchScheduleOnce() {
    String err;
    if (tm4c.getSchedule(scheduleData, err)) {
        displayDirty = true;
    } else {
        Serial.printf("[UART] get_schedule fail: %s\n", err.c_str());
    }
}

// ---------- Storage ----------
bool loadCreds() {
    prefs.begin("wifi", true);
    homeSsid = prefs.getString("ssid", "");
    homePass = prefs.getString("pass", "");
    timezoneOffsetSeconds = prefs.getInt("tzOffset", 0);
    prefs.end();
    hasStoredCreds = homeSsid.length() > 0;
    return hasStoredCreds;
}

void saveCreds(const String &ssid, const String &pass, int32_t tz) {
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.putInt("tzOffset", tz);
    prefs.end();
    hasStoredCreds = true;
    timezoneOffsetSeconds = tz;
}

void clearCreds() {
    prefs.begin("wifi", false);
    prefs.clear();
    prefs.end();
    hasStoredCreds = false;
    timezoneOffsetSeconds = 0;
}

void tryConnectStored() {
    if (loadCreds()) {
        Serial.printf("[WiFi] Try stored SSID=%s\n", homeSsid.c_str());
        connectHome(homeSsid, homePass);
        drawText("Connecting saved", homeSsid.c_str());
    } else {
        startAP();
    }
}

bool syncTimeAndTm4c() {
    unsigned long startAttempt = millis();
    while (WiFi.status() == WL_CONNECTED) {
        configTime(timezoneOffsetSeconds, 0, "pool.ntp.org", "time.nist.gov", "ntp.aliyun.com");
        struct tm timeinfo{};
        bool ok = false;
        for (int i = 0; i < 3 && !ok; ++i) {
            if (getLocalTime(&timeinfo, 3000)) ok = true;
        }
        if (!ok) {
            Serial.println("[NTP] Sync fail, retrying...");
            delay(1000);
            continue;
        }

        lastNtpSyncMs = millis();
        char buf[64];
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
        Serial.printf("[NTP] Synced time: %s (tz=%d)\n", buf, timezoneOffsetSeconds);
        time_t nowTs = time(nullptr) + timezoneOffsetSeconds;
        String err;
        if (!tm4c.timeSync(static_cast<uint32_t>(nowTs), timezoneOffsetSeconds, err)) {
            Serial.printf("[UART] time_sync fail: %s\n", err.c_str());
            timeDesyncWarning = true;
        } else {
            timeDesyncWarning = false;
        }
        return true;
    }
    Serial.println("[NTP] No WiFi, cannot sync");
    timeDesyncWarning = true;
    return false;
}

#if 0
bool syncTimeAndTm4c() {
    configTime(timezoneOffsetSeconds, 0, "pool.ntp.org", "time.nist.gov", "ntp.aliyun.com");
    struct tm timeinfo{};
    bool ok = false;
    for (int i = 0; i < 3 && !ok; ++i) {
        if (getLocalTime(&timeinfo, 3000)) ok = true;
    }
    lastNtpSyncMs = millis();
    if (!ok) {
        Serial.println("[NTP] Failed to sync time");
        return false;
    }
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.printf("[NTP] Synced time: %s (tz=%d)\n", buf, timezoneOffsetSeconds);
    time_t nowTs = time(nullptr) + timezoneOffsetSeconds;
    String err;
    if (!tm4c.timeSync(static_cast<uint32_t>(nowTs), timezoneOffsetSeconds, err)) {
        Serial.printf("[UART] time_sync fail: %s\n", err.c_str());
    }
    return true;
}
#endif

// ---------- Web routes ----------
const char *CONFIG_PAGE = R"HTML(
<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>WiFi Setup</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;outline:none;-webkit-tap-highlight-color:transparent;}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;background:#FFF8F0;color:#6D4C41;display:flex;align-items:center;justify-content:center;min-height:100vh;padding:20px;}
.container{background:#FFF;width:100%;max-width:360px;padding:40px 30px;border-radius:30px;box-shadow:0 10px 25px rgba(255,183,178,0.2);text-align:center;position:relative;overflow:hidden;}
.container::before{content:'';position:absolute;top:0;left:0;width:100%;height:8px;background:#FFB7B2;}
.kaomoji{font-size:50px;font-weight:bold;color:#FFB7B2;margin-bottom:10px;display:inline-block;white-space:nowrap;animation:bounce 2s infinite ease-in-out;text-shadow:2px 2px 0px #FFF,4px 4px 0px rgba(255,183,178,0.3);}
h1{font-size:24px;font-weight:700;margin-bottom:8px;color:#5D4037;}
p{font-size:14px;color:#A1887F;margin-bottom:30px;}
.form-group{margin-bottom:20px;text-align:left;}
label{display:block;font-size:13px;font-weight:600;margin-bottom:8px;margin-left:10px;color:#8D6E63;text-transform:uppercase;letter-spacing:0.5px;}
select,input{width:100%;padding:14px 20px;border:2px solid #FBE9E7;border-radius:20px;font-size:16px;background:#FFFDFB;color:#5D4037;transition:all .3s ease;appearance:none;}
.select-wrapper{position:relative;}
.select-wrapper::after{content:'▼';font-size:12px;color:#FFB7B2;position:absolute;right:20px;top:50%;transform:translateY(-50%);pointer-events:none;}
select:focus,input:focus{border-color:#FFB7B2;background:#FFF;box-shadow:0 0 0 4px rgba(255,183,178,0.2);}
button{width:100%;padding:16px;margin-top:10px;background:#FFB7B2;color:#FFF;border:none;border-radius:20px;font-size:16px;font-weight:bold;cursor:pointer;transition:transform .1s ease,background-color .3s;box-shadow:0 4px 10px rgba(255,183,178,0.4);}
button:active{transform:scale(0.96);background:#FF8A65;}
button.loading{background:#D7CCC8;pointer-events:none;cursor:default;}
@keyframes bounce{0%,100%{transform:translateY(0);}50%{transform:translateY(-10px);}}
.footer{margin-top:25px;font-size:12px;color:#D7CCC8;}
</style>
</head>
<body>
  <div class="container">
    <div class="kaomoji">(=^･ω･^=)</div>
    <h1>Hello Human!</h1>
    <p>Please connect me to the internet ~</p>
    <form id="wifiForm" action="/configure" method="POST">
      <div class="form-group">
        <label for="ssid">Network Name</label>
        <div class="select-wrapper" id="ssid-wrapper">
          <select id="ssid" name="ssid" required>
            <option value="" disabled selected>Searching...</option>
          </select>
        </div>
      </div>
      <div class="form-group">
        <label for="password">Password</label>
        <input type="password" id="password" name="password" placeholder="Leave empty if open">
      </div>
      <div class="form-group">
        <label for="tz">Timezone</label>
        <div class="select-wrapper">
          <select id="tz" name="tzOffset" required>
            <option value="28800" selected>GMT+08:00 (Beijing)</option>
            <option value="0">GMT+00:00 (UTC)</option>
            <option value="19800">GMT+05:30 (India)</option>
            <option value="25200">GMT+07:00 (Bangkok)</option>
            <option value="32400">GMT+09:00 (Tokyo)</option>
            <option value="3600">GMT+01:00 (Berlin)</option>
            <option value="-18000">GMT-05:00 (New York)</option>
            <option value="-25200">GMT-07:00 (Denver)</option>
            <option value="-28800">GMT-08:00 (San Francisco)</option>
          </select>
        </div>
      </div>
      <button type="submit" id="saveBtn">Save & Connect</button>
      <button type="button" id="resetBtn" style="margin-top:24px;background:#E0E0E0;color:#6D4C41">Reset Wi-Fi</button>
    </form>
    <div class="footer">Pet Device Configuration</div>
  </div>
    <script>
    async function loadNetworks(attempt=0){
      try{
        const res = await fetch('/scan');
        const list = await res.json();
        const wrapper = document.getElementById('ssid-wrapper');
        const sel = document.getElementById('ssid');
        if (list.length === 0) {
          sel.innerHTML = '<option value=\"\" disabled selected>Searching...</option>';
          if (attempt < 5) {
            setTimeout(()=>loadNetworks(attempt+1), 1200);
          } else {
            wrapper.innerHTML = '<input id=\"ssid\" name=\"ssid\" required placeholder=\"Enter SSID manually\" style=\"width:100%;padding:14px 20px;border:2px solid #FBE9E7;border-radius:20px;font-size:16px;background:#FFFDFB;color:#5D4037;\" />';
          }
        } else {
          sel.innerHTML = '<option value=\"\" disabled selected>Select WiFi</option>';
          list.forEach(ssid=>{
            const opt=document.createElement('option');
            opt.value=ssid; opt.textContent=ssid;
            sel.appendChild(opt);
          });
        }
      }catch(e){
        const wrapper = document.getElementById('ssid-wrapper');
        wrapper.innerHTML='<input id=\"ssid\" name=\"ssid\" required placeholder=\"Enter SSID manually\" style=\"width:100%;padding:14px 20px;border:2px solid #FBE9E7;border-radius:20px;font-size:16px;background:#FFFDFB;color:#5D4037;\" />';
      }
    }

    async function loadTz(){
      try{
        const res = await fetch('/api/tz');
        const json = await res.json();
        if (typeof json.tzOffset !== 'undefined') {
          document.getElementById('tz').value = json.tzOffset;
        }
      }catch(e){
        console.warn('loadTz failed', e);
      }
    }

    document.getElementById('wifiForm').addEventListener('submit', async (e)=>{
      e.preventDefault();
      const btn=document.getElementById('saveBtn');
      btn.textContent='Connecting...';
      btn.classList.add('loading');
      const fd=new FormData(e.target);
      await fetch('/configure',{method:'POST',body:fd});
      btn.textContent='Saved';
    });
    document.getElementById('resetBtn').addEventListener('click', async ()=>{
      if (!confirm('Confirm reset Wi-Fi settings?')) return;
      const btn=document.getElementById('resetBtn');
      btn.textContent='Resetting...';
      btn.classList.add('loading');
      await fetch('/reset');
      btn.textContent='Done';
    });
    window.onload=()=>{
      loadNetworks();
      loadTz();
    };
  </script>
</body>
</html>
)HTML";

const char *HOME_PAGE = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>Pet Dashboard</title>
    <style>
        /* Warm & Cute Palette */
        :root {
            --bg-color: #FFF8F0;
            --card-bg: #FFFFFF;
            --primary: #FFB7B2;    /* Pinkish */
            --primary-dark: #FF8A65; /* Orange */
            --text-main: #6D4C41;  /* Chocolate */
            --text-sub: #A1887F;
            --alert: #FF5252;      /* Red for warnings */
            --success: #81C784;    /* Green for High/OK */
        }

        * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; outline: none; }

        body {
            font-family: "Nunito", -apple-system, sans-serif; /* Rounder font if available */
            background-color: var(--bg-color);
            color: var(--text-main);
            padding: 20px;
            max-width: 480px;
            margin: 0 auto;
        }

        /* --- Header --- */
        #alert-banner {
            background: var(--alert);
            color: #fff;
            padding: 12px 14px;
            border-radius: 14px;
            margin-bottom: 14px;
            font-weight: 800;
            box-shadow: 0 6px 16px rgba(255, 82, 82, 0.35);
            text-align: center;
        }

        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 25px;
        }

        h1 { font-size: 22px; font-weight: 800; }
        .kaomoji-header { color: var(--primary); font-size: 14px; display: block; }
        
        .settings-btn {
            background: none;
            border: none;
            font-size: 24px;
            cursor: pointer;
            color: var(--text-sub);
            transition: transform 0.3s;
        }
        .settings-btn:active { transform: rotate(90deg); }

        .quick-actions {
            display: flex;
            gap: 12px;
            margin-bottom: 16px;
        }

        .feed-btn {
            flex: 1;
            background-color: var(--primary);
            color: #fff;
            border: none;
            border-radius: 20px;
            padding: 15px;
            font-weight: bold;
            font-size: 16px;
            box-shadow: 0 4px 10px rgba(255, 183, 178, 0.4);
            cursor: pointer;
            transition: transform 0.15s ease, box-shadow 0.15s ease, background-color 0.2s ease;
        }
        .feed-btn:active { transform: scale(0.96); background-color: var(--primary-dark); box-shadow: none; }

        /* --- Status Grid --- */
        .status-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-bottom: 25px;
        }

        .card {
            background: var(--card-bg);
            border-radius: 20px;
            padding: 15px;
            box-shadow: 0 4px 15px rgba(255, 183, 178, 0.15);
            text-align: center;
            position: relative;
            transition: transform 0.2s;
        }
        
        .card:active { transform: scale(0.98); }

        .card-label { font-size: 12px; color: var(--text-sub); font-weight: 700; text-transform: uppercase; letter-spacing: 0.5px; margin-bottom: 5px; }
        .card-value { font-size: 20px; font-weight: 800; color: var(--text-main); }
        .card-icon { font-size: 24px; margin-bottom: 5px; display: block; }
        .card-chip {
            position: absolute;
            bottom: 12px;
            right: 12px;
            padding: 4px 10px;
            border-radius: 999px;
            font-size: 10px;
            font-weight: 800;
            letter-spacing: 0.6px;
            background: #FF7043;
            color: #fff;
            box-shadow: 0 4px 10px rgba(255,112,67,0.25);
            text-transform: uppercase;
            display: none;
        }
        
        /* Alert States */
        .is-alert { color: var(--alert) !important; animation: pulse 1.5s infinite; }
        .is-ok { color: var(--success); }
        
        @keyframes pulse { 0% { opacity: 1; } 50% { opacity: 0.6; } 100% { opacity: 1; } }

        /* --- Sections --- */
        .section-title {
            font-size: 16px;
            font-weight: 700;
            margin-bottom: 12px;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .info-list {
            background: var(--card-bg);
            border-radius: 20px;
            padding: 15px;
            margin-bottom: 25px;
            box-shadow: 0 4px 15px rgba(255, 183, 178, 0.15);
        }

        .info-item {
            display: flex;
            justify-content: space-between;
            padding: 8px 0;
            border-bottom: 1px solid #FFF0EB;
            font-size: 14px;
        }
        .info-item:last-child { border-bottom: none; }
        .info-time { color: var(--text-sub); }
        .info-val { font-weight: 700; }

        /* --- Schedule --- */
        .schedule-list {
            list-style: none;
        }
        
        .schedule-item {
            background: var(--card-bg);
            border-radius: 15px;
            padding: 12px 15px;
            margin-bottom: 10px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            box-shadow: 0 2px 8px rgba(0,0,0,0.03);
        }

        .sch-time { font-weight: 800; font-size: 18px; color: var(--text-main); }
        .sch-amount { 
            font-size: 12px; 
            padding: 4px 10px; 
            border-radius: 10px; 
            background: #FFF0EB; 
            color: var(--primary-dark); 
            font-weight: bold;
            margin-left: 10px;
        }
        .del-btn {
            background: none;
            border: none;
            color: #FFCDD2;
            font-size: 18px;
            padding: 0 5px;
            cursor: pointer;
        }
        .del-btn:hover { color: var(--alert); }

        .add-btn {
            width: 100%;
            padding: 15px;
            background-color: var(--primary);
            color: white;
            border: none;
            border-radius: 20px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            box-shadow: 0 4px 10px rgba(255, 183, 178, 0.4);
            margin-bottom: 40px;
        }
        .add-btn:active { transform: scale(0.96); background-color: var(--primary-dark); }
        .add-btn:disabled { background-color: #E0E0E0; color: #999; box-shadow: none; }

        /* --- Modal --- */
        .modal-overlay {
            position: fixed; top: 0; left: 0; width: 100%; height: 100%;
            background: rgba(109, 76, 65, 0.4);
            backdrop-filter: blur(2px);
            display: none;
            justify-content: center;
            align-items: center;
            z-index: 100;
        }
        .modal {
            background: #FFF;
            padding: 25px;
            border-radius: 25px;
            width: 85%;
            max-width: 320px;
            text-align: center;
            box-shadow: 0 10px 30px rgba(0,0,0,0.1);
        }
        .modal h3 { margin-bottom: 15px; color: var(--primary-dark); }
        
        .modal-input-group { margin-bottom: 15px; text-align: left; }
        .modal label { display: block; font-size: 12px; color: var(--text-sub); margin-bottom: 5px; }
        .modal input, .modal select {
            width: 100%; padding: 10px; border: 2px solid #FBE9E7; border-radius: 12px;
            font-size: 16px; color: var(--text-main); background: #FFFDFB;
        }

        .modal-actions { display: flex; gap: 10px; margin-top: 20px; }
        .btn-cancel { background: #EFEBE9; color: var(--text-sub); }
        .btn-save { background: var(--primary); color: white; }
        .btn-modal { flex: 1; padding: 12px; border: none; border-radius: 15px; font-weight: bold; cursor: pointer; }

        .feed-actions { display: flex; gap: 10px; margin-top: 10px; }
        .feed-choice {
            flex: 1;
            padding: 12px;
            border: none;
            border-radius: 12px;
            font-weight: 800;
            color: #fff;
            cursor: pointer;
            box-shadow: 0 6px 14px rgba(0,0,0,0.08);
        }
        .feed-choice.l { background: #FFCC80; color: #6D4C41; }
        .feed-choice.m { background: #FFB7B2; }
        .feed-choice.h { background: #FF7043; }

    </style>
</head>
<body>

    <div id="alert-banner" style="display:none"></div>

    <!-- Loading overlay -->
    <div id="loadingOverlay" style="position:fixed; top:0; left:0; width:100%; height:100%; background:rgba(0,0,0,0.5); z-index:200; display:none; align-items:center; justify-content:center;">
        <div style="background:#fff; padding:30px; border-radius:20px; text-align:center;">
            <div style="font-size:40px; margin-bottom:10px;">⏳</div>
            <div id="loadingText" style="color:#6D4C41; font-weight:bold;">Saving...</div>
        </div>
    </div>

    <header>
        <div>
            <span class="kaomoji-header">(=^･ω･^=) Meow!</span>
            <h1>Pet Feeder</h1>
        </div>
        <button class="settings-btn" onclick="openSettings()">⚙️</button>
    </header>

    <!-- Status Grid -->
    <div class="status-grid">
        <!-- Food Bowl -->
        <div class="card">
            <span class="card-icon">🥣</span>
            <div class="card-label">Food Bowl</div>
            <div class="card-value" id="val-food-bowl">--g</div>
            <div class="card-chip" id="food-low-chip">LOW</div>
        </div>
        <!-- Water Bowl -->
        <div class="card">
            <span class="card-icon">💧</span>
            <div class="card-label">Water Bowl</div>
            <div class="card-value" id="val-water-bowl">--g</div>
        </div>
    </div>

    <div class="quick-actions">
        <button class="feed-btn" id="feedBtn" onclick="openFeedModal()">Feed Now</button>
    </div>

    <div class="section-title">
        <span>📅</span> Feeding Schedule 
        <span style="font-size:12px; color:var(--text-sub); margin-left:auto" id="schedule-count">0/8</span>
    </div>
    
    <div id="schedule-container" class="schedule-list">
        <!-- Items will be injected here -->
    </div>

    <button class="add-btn" id="addBtn" onclick="openModal()">+ Add Schedule</button>


    <!-- Add Schedule Modal -->
    <div class="modal-overlay" id="modalOverlay">
        <div class="modal">
            <h3>Add Feeding Time</h3>
            <div class="modal-input-group">
                <label>Time</label>
                <input type="time" id="inputTime">
            </div>
            <div class="modal-input-group">
                <label>Amount</label>
                <select id="inputAmount">
                    <option value="L">L (Little)</option>
                    <option value="M">M (Normal)</option>
                    <option value="H">H (Feast)</option>
                </select>
            </div>
            <div class="modal-actions">
                <button class="btn-modal btn-cancel" onclick="closeModal()">Cancel</button>
                <button class="btn-modal btn-save" onclick="saveSchedule()">Save</button>
            </div>
        </div>
    </div>

    <!-- Feed Now Modal -->
    <div class="modal-overlay" id="feedOverlay">
        <div class="modal">
            <h3>Manual Feed</h3>
            <p style="color:var(--text-sub); margin-bottom:8px;">Pick a portion</p>
            <div class="feed-actions">
                <button class="feed-choice l" onclick="sendFeed('L')">L</button>
                <button class="feed-choice m" onclick="sendFeed('M')">M</button>
                <button class="feed-choice h" onclick="sendFeed('H')">H</button>
            </div>
            <div class="modal-actions" style="margin-top:14px;">
                <button class="btn-modal btn-cancel" onclick="closeFeedModal()">Close</button>
            </div>
        </div>
    </div>

    <script>
        // --- 1. DEVICE DATA ---
        // Pulls from ESP32 mock API for now; swap to TM4C UART later.
        let deviceData = {
            foodBowl: 0,
            waterBowl: 0,
            timeWarn: false,
            schedule: []
        };
        const FOOD_LOW_BADGE = 50;
        const WATER_LOW = 80;

        const MAX_SCHEDULES = 8;
        const AMOUNT_LABEL = { L: "L · Small", M: "M · Mid", H: "H · High" };

        function labelForAmount(code) {
            const c = (code || "").toUpperCase();
            return AMOUNT_LABEL[c] || c || "--";
        }

        // --- 2. UPDATE UI FUNCTIONS ---

        function updateStatus() {
            // Food Bowl
            const foodVal = Math.min(999, Math.max(0, deviceData.foodBowl));
            document.getElementById('val-food-bowl').textContent = foodVal + "g";
            const lowChip = document.getElementById('food-low-chip');
            lowChip.style.display = (foodVal < FOOD_LOW_BADGE) ? 'inline-flex' : 'none';

            // Water Bowl
            const wbEl = document.getElementById('val-water-bowl');
            const waterVal = Math.min(999, Math.max(0, deviceData.waterBowl));
            wbEl.textContent = waterVal + "g";
            if(waterVal < WATER_LOW) {
                wbEl.classList.add('is-alert');
                wbEl.classList.remove('is-ok');
            } else {
                wbEl.classList.add('is-ok');
                wbEl.classList.remove('is-alert');
            }

            // Alerts summary
            const alerts = [];
            if (waterVal < WATER_LOW) alerts.push("Water bowl is LOW");
            if (deviceData.timeWarn) alerts.push("Clock not synced");
            const banner = document.getElementById('alert-banner');
            if (alerts.length) {
                banner.style.display = 'block';
                banner.textContent = alerts.join(" · ");
            } else {
                banner.style.display = 'none';
                banner.textContent = "";
            }
        }

        function renderSchedule() {
            const container = document.getElementById('schedule-container');
            const btn = document.getElementById('addBtn');
            const countLabel = document.getElementById('schedule-count');
            
            container.innerHTML = '';
            
            // Sort by time
            deviceData.schedule.sort((a, b) => a.time.localeCompare(b.time));

            deviceData.schedule.forEach((item, index) => {
                const div = document.createElement('div');
                div.className = 'schedule-item';
                div.innerHTML = `
                    <div style="display:flex; align-items:center">
                        <span class="sch-time">${item.time}</span>
                        <span class="sch-amount">${labelForAmount(item.amount)}</span>
                    </div>
                    <button class="del-btn" onclick="deleteSchedule(${index})">✕</button>
                `;
                container.appendChild(div);
            });

            // Update Counter
            countLabel.textContent = `${deviceData.schedule.length}/${MAX_SCHEDULES}`;

            // Disable button if full
            if(deviceData.schedule.length >= MAX_SCHEDULES) {
                btn.disabled = true;
                btn.textContent = "Max 8 Reached";
            } else {
                btn.disabled = false;
                btn.textContent = "+ Add Schedule";
            }
        }

        // --- 3. ACTIONS ---

        function openSettings() {
            window.location.href = "/wifi";
        }

        const feedModal = document.getElementById('feedOverlay');

        function openFeedModal() {
            feedModal.style.display = 'flex';
        }

        function closeFeedModal() {
            feedModal.style.display = 'none';
        }

        async function sendFeed(level) {
            closeFeedModal();
            const btn = document.getElementById('feedBtn');
            const prevText = btn.textContent;
            btn.textContent = `Feeding ${level}...`;
            btn.disabled = true;
            try {
                const body = new URLSearchParams();
                body.append('level', level);
                const res = await fetch('/api/feed_now', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: body.toString()
                });
                if (!res.ok) {
                    throw new Error(await res.text());
                }
                btn.textContent = `Fed (${level})`;
                setTimeout(()=>{
                    btn.textContent = 'Feed Now';
                    btn.disabled = false;
                }, 1500);
            } catch (e) {
                alert('Feed failed: ' + e.message);
                btn.textContent = prevText;
                btn.disabled = false;
            }
        }

        feedModal.addEventListener('click', (e)=>{
            if (e.target === feedModal) closeFeedModal();
        });

        const cloneSchedule = () => deviceData.schedule.map(item => ({ time: item.time, amount: item.amount }));
        const delay = (ms) => new Promise(resolve => setTimeout(resolve, ms));

        function showLoading(text = 'Saving...') {
            document.getElementById('loadingText').textContent = text;
            document.getElementById('loadingOverlay').style.display = 'flex';
        }

        function hideLoading() {
            document.getElementById('loadingOverlay').style.display = 'none';
        }

        async function deleteSchedule(index) {
            if(confirm("Remove this feeding time?")) {
                showLoading('Deleting...');
                const prev = cloneSchedule();
                deviceData.schedule.splice(index, 1);
                renderSchedule();
                const ok = await sendDataToESP(); // Sync
                hideLoading();
                if (!ok) {
                    deviceData.schedule = prev;
                    renderSchedule();
                }
            }
        }

        // Modal Logic
        const modal = document.getElementById('modalOverlay');
        
        function openModal() {
            if(deviceData.schedule.length >= MAX_SCHEDULES) return;
            // Default time to now
            const now = new Date();
            const timeStr = now.toTimeString().substring(0,5);
            document.getElementById('inputTime').value = timeStr;
            modal.style.display = 'flex';
        }

        function closeModal() {
            modal.style.display = 'none';
        }

        async function saveSchedule() {
            const time = document.getElementById('inputTime').value;
            const amount = document.getElementById('inputAmount').value.toUpperCase();

            if(!time) return;

            // --- Check for Duplicates ---
            // Frontend check to prevent same time
            const exists = deviceData.schedule.some(item => item.time === time);
            if (exists) {
                alert("This time is already scheduled! \nPlease choose a different time. (=^･ω･^=)");
                return;
            }

            closeModal();
            showLoading('Saving schedule...');
            const prev = cloneSchedule();
            deviceData.schedule.push({ time, amount });
            renderSchedule();
            const ok = await sendDataToESP(); // Sync
            hideLoading();
            if (!ok) {
                deviceData.schedule = prev;
                renderSchedule();
            }
        }

        // --- 4. DATA SYNC (Simulated) ---
        async function loadStatus() {
            const res = await fetch('/api/status');
            const json = await res.json();
            deviceData.foodBowl = Math.min(999, Math.max(0, json.foodBowl ?? deviceData.foodBowl));
            deviceData.waterBowl = Math.min(999, Math.max(0, json.waterBowl ?? deviceData.waterBowl));
            deviceData.timeWarn = !!json.timeWarn;
        }

        async function loadSchedule() {
            const res = await fetch('/api/schedule');
            const list = await res.json();
            deviceData.schedule = Array.isArray(list) ? list.map(item => ({
                time: item.time || "",
                amount: (item.amount || '').toUpperCase()
            })) : [];
        }

        async function sendDataToESP() {
            const payload = JSON.stringify(deviceData.schedule);
            try {
                // Send schedule update request (returns immediately)
                const res = await fetch('/api/schedule', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: payload
                });
                if (!res.ok) throw new Error('Request failed');

                console.log('Schedule update queued, polling for status...');

                // Poll for completion
                const maxPollTime = 35000; // 35 seconds
                const pollInterval = 500;  // 500ms
                const startTime = Date.now();

                while (Date.now() - startTime < maxPollTime) {
                    await delay(pollInterval);

                    const statusRes = await fetch('/api/schedule_status');
                    if (!statusRes.ok) throw new Error('Status check failed');

                    const status = await statusRes.json();
                    console.log('Schedule status:', status);

                    if (status.status === 'success') {
                        console.log('Schedule update SUCCESS');
                        // Clear the task status
                        await fetch('/api/schedule_status?clear=1');
                        // Refresh schedule display
                        await loadSchedule();
                        renderSchedule();
                        return true;
                    } else if (status.status === 'failed') {
                        const errMsg = status.error || 'Unknown error';
                        // Clear the task status
                        await fetch('/api/schedule_status?clear=1');
                        throw new Error(errMsg);
                    } else if (status.status === 'processing') {
                        const retry = status.retry || 0;
                        console.log(`Processing... retry ${retry}`);
                        // Update loading text to show progress
                        const elapsed = Math.floor((Date.now() - startTime) / 1000);
                        showLoading(`Syncing... (retry ${retry}, ${elapsed}s)`);
                    }
                    // Continue polling if pending or processing
                }

                throw new Error('Timeout - device may be offline');

            } catch (e) {
                console.error('Schedule sync failed:', e);
                alert(`计划保存失败: ${e.message}\n请重试 (=^･ω･^=)`);
                await loadSchedule();
                renderSchedule();
                return false;
            }
        }

        let statusTimer = null;
        function startStatusTimer() {
            if (statusTimer) return;
            statusTimer = setInterval(async ()=>{
                try { await loadStatus(); updateStatus(); } catch(e){ console.warn('Status poll failed', e); }
            }, 5000);  // Reduced polling from 3s to 5s
        }
        function stopStatusTimer() {
            if (statusTimer) {
                clearInterval(statusTimer);
                statusTimer = null;
            }
        }

        async function onVisibleRefresh() {
            try {
                await Promise.all([loadStatus(), loadSchedule()]);
                updateStatus();
                renderSchedule();
            } catch (e) {
                console.warn('Visible refresh failed', e);
            }
        }

        // Initialize
        window.onload = async function() {
            await onVisibleRefresh();
            startStatusTimer();
        };

        document.addEventListener('visibilitychange', async ()=>{
            if (document.visibilityState === 'visible') {
                await onVisibleRefresh();
                startStatusTimer();
            } else {
                stopStatusTimer();
            }
        });

    </script>
</body>
</html>
)HTML";

void registerWebHandlers() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("[HTTP] GET / -> HOME_PAGE");
            request->send(200, "text/html", HOME_PAGE);
        } else {
            Serial.println("[HTTP] GET / -> CONFIG_PAGE");
            request->send(200, "text/html", CONFIG_PAGE);
        }
    });

    server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
        Serial.println("[HTTP] GET /wifi");
        request->send(200, "text/html", CONFIG_PAGE);
    });

    // ---- API: backed by TM4C UART JSON line ----
    server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        String err;
        if (tm4c.getStatus(statusData, err)) {
            String payload = statusToJson();
            request->send(200, "application/json", payload);
        } else {
            Serial.printf("[UART] get_status fail: %s\n", err.c_str());
            request->send(500, "text/plain", err);
        }
    });

    server.on("/api/tz", HTTP_GET, [](AsyncWebServerRequest *request) {
        String payload = "{\"tzOffset\":" + String(timezoneOffsetSeconds) + "}";
        request->send(200, "application/json", payload);
    });

    server.on("/api/schedule", HTTP_GET, [](AsyncWebServerRequest *request) {
        String err;
        if (tm4c.getSchedule(scheduleData, err)) {
            String payload = scheduleToJson();
            request->send(200, "application/json", payload);
        } else {
            Serial.printf("[UART] get_schedule fail: %s\n", err.c_str());
            request->send(500, "text/plain", err);
        }
    });

    server.on("/api/schedule_status", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Check if client wants to acknowledge/clear the result
        bool shouldClear = request->hasParam("clear");

        String status;
        switch (scheduleTask.state) {
            case ScheduleTaskState::IDLE:
                status = "idle";
                break;
            case ScheduleTaskState::PENDING:
                status = "pending";
                break;
            case ScheduleTaskState::PROCESSING:
                status = "processing";
                break;
            case ScheduleTaskState::SUCCESS:
                status = "success";
                break;
            case ScheduleTaskState::FAILED:
                status = "failed";
                break;
        }
        String payload = "{\"status\":\"" + status + "\"";
        if (scheduleTask.state == ScheduleTaskState::FAILED && scheduleTask.errorMessage.length()) {
            payload += ",\"error\":\"" + scheduleTask.errorMessage + "\"";
        }
        if (scheduleTask.state == ScheduleTaskState::PROCESSING) {
            payload += ",\"retry\":" + String(scheduleTask.retryCount);
        }
        payload += "}";
        request->send(200, "application/json", payload);

        // Clear state after client acknowledges
        if (shouldClear && (scheduleTask.state == ScheduleTaskState::SUCCESS ||
                           scheduleTask.state == ScheduleTaskState::FAILED)) {
            scheduleTask.state = ScheduleTaskState::IDLE;
        }
    });

    static String scheduleBody;
    server.on("/api/schedule", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  // Immediate response - processing will happen in background
                  request->send(202, "application/json", "{\"status\":\"accepted\"}");
              },
              nullptr,
              [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
                  if (index == 0) {
                      scheduleBody = "";
                      scheduleBody.reserve(total + 1);
                  }
                  scheduleBody.concat((const char *) data, len);
                  if (index + len != total) return;

                  JsonDocument doc;
                  DeserializationError err = deserializeJson(doc, scheduleBody);
                  if (err || !doc.is<JsonArray>()) {
                      Serial.printf("[Async] set_schedule parse error: %s\n", err.c_str());
                      scheduleTask.state = ScheduleTaskState::FAILED;
                      scheduleTask.errorMessage = "Invalid JSON";
                      return;
                  }

                  // Parse schedule and queue it for background processing
                  scheduleTask.pendingSchedule.clear();
                  for (JsonObject obj: doc.as<JsonArray>()) {
                      const char *time = obj["time"] | "";
                      const char *amount = obj["amount"] | "";
                      String amountCode = Tm4cLink::amountToCode(String(amount));
                      if (strlen(time) == 0 || amountCode.length() == 0) continue;
                      scheduleTask.pendingSchedule.push_back({String(time), amountCode});
                  }

                  // Mark task as pending - will be processed in loop()
                  scheduleTask.state = ScheduleTaskState::PENDING;
                  scheduleTask.errorMessage = "";
                  scheduleTask.retryCount = 0;
                  scheduleTask.startMs = millis();
                  Serial.printf("[Async] Schedule update queued: %d items\n", scheduleTask.pendingSchedule.size());
              });

    server.on("/api/set_time", HTTP_POST,
              [](AsyncWebServerRequest *request) {
                  request->send(200, "application/json", "{\"ok\":true}");
              },
              nullptr,
              [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
                  static String timeBody;
                  if (index == 0) {
                      timeBody = "";
                      timeBody.reserve(total + 1);
                  }
                  timeBody.concat((const char *) data, len);
                  if (index + len == total) {
                      JsonDocument doc;
                      DeserializationError errJson = deserializeJson(doc, timeBody);
                      if (errJson) {
                          Serial.printf("[UART] set_time parse error: %s\n", errJson.c_str());
                          return;
                      }
                      uint32_t unixTs = doc["unix"] | 0;
                      String err;
                      if (!tm4c.timeSync(unixTs, timezoneOffsetSeconds, err)) {
                          Serial.printf("[UART] set_time fail: %s\n", err.c_str());
                      } else {
                          Serial.printf("[UART] set_time ok -> ts=%lu tz=%d\n", (unsigned long) unixTs, timezoneOffsetSeconds);
                      }
                  }
              });

    server.on("/api/feed_now", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("level", true)) {
            request->send(400, "text/plain", "level required");
            return;
        }
        String level = request->getParam("level", true)->value();
        level.trim();
        if (level.length() > 1) level = level.substring(0, 1);
        level.toUpperCase();
        String err;
        if (tm4c.feedNow(level, err)) {
            request->send(200, "application/json", "{\"ok\":true}");
        } else {
            int code = (err == "invalid level") ? 400 : 500;
            request->send(code, "text/plain", err);
        }
    });

    server.on("/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Mark request; actual scan starts in loop to avoid blocking HTTP task.
        scanRequested = true;
        if (!cachedSsids.empty()) {
            String json = "[";
            for (size_t i = 0; i < cachedSsids.size(); ++i) {
                if (i) json += ",";
                json += "\"" + cachedSsids[i] + "\"";
            }
            json += "]";
            request->send(200, "application/json", json);
        } else {
            request->send(200, "application/json", "[]");
        }
    });

    server.on("/configure", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("ssid", true) || !request->hasParam("password", true) || !request->hasParam("tzOffset", true)) {
            request->send(400, "text/plain", "ssid/password/tzOffset required");
            return;
        }
        String ssid = request->getParam("ssid", true)->value();
        String pwd = request->getParam("password", true)->value();
        int32_t tzOffset = request->getParam("tzOffset", true)->value().toInt();
        request->send(200, "text/plain", "Connecting to home Wi-Fi...");
        Serial.printf("[CFG] Received SSID=%s TZ=%d\n", ssid.c_str(), tzOffset);
        saveCreds(ssid, pwd, tzOffset);
        connectHome(ssid, pwd);
    });

    server.on("/send", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!request->hasParam("msg", true)) {
            request->send(400, "text/plain", "msg required");
            return;
        }
        String msg = request->getParam("msg", true)->value();
        Serial.printf("[USER] %s\n", msg.c_str());
        request->send(200, "text/plain", "ok");
    });

    server.on("/reset", HTTP_ANY, [](AsyncWebServerRequest *request) {
        clearCreds();
        request->send(200, "text/plain", "reset, restarting");
        delay(200);
        ESP.restart();
    });
}

// ---------- Wi-Fi events ----------
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
            // 手机连上 AP，切到配置页二维码
            showStage(Stage::CONFIG_URL);
            Serial.println("[WiFi] Phone joined AP");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP: {
            homeIp = WiFi.localIP();
            makeQr(qrMainPage, String("http://") + homeIp.toString() + "/");
            showStage(Stage::ONLINE);
            // 关闭 AP，切到纯 STA
            WiFi.softAPdisconnect(true);
            WiFi.mode(WIFI_MODE_STA);
            Serial.printf("[WiFi] Connected to home, IP=%s\n", homeIp.toString().c_str());
            // Sync time via NTP using selected timezone
            syncTimeAndTm4c();
            break;
        }
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            // 回退到 AP 让用户重新配置，但节流避免重复刷日志
            static unsigned long lastFallbackMs = 0;
            if (millis() - lastFallbackMs < 5000) {
                Serial.println("[WiFi] STA lost (throttled)");
                break;
            }
            lastFallbackMs = millis();
            Serial.println("[WiFi] STA lost, fallback to AP");
            startAP();
            break;
        default:
            break;
    }
}

// ---------- Setup & Loop ----------
void setup() {
    Serial.begin(115200);
    tm4c.begin();
    tft.init();
    tft.setRotation(0);
    tft.fillScreen(COLOR_BG);
    tft.setTextDatum(TC_DATUM);
    tft.setTextColor(COLOR_TEXT, COLOR_BG);
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.drawString("WiFi QR setup...", tft.width() / 2, 16);
    pinMode(DISPLAY_BTN_PIN, INPUT_PULLUP);
    lastBtnState = digitalRead(DISPLAY_BTN_PIN);

    WiFi.onEvent(onWiFiEvent);
    tryConnectStored();
    // Initial fetch to populate display/cache
    fetchStatusOnce();
    fetchScheduleOnce();
    registerWebHandlers();
    server.begin();

    Serial.printf("AP SSID: %s  PASS: %s\n", AP_SSID, AP_PASSWORD);
}

void loop() {
    const uint32_t nowMs = millis();
    tm4c.poll();  // handle asynchronous AT+GETTIME

    // Process async schedule update task
    if (scheduleTask.state == ScheduleTaskState::PENDING) {
        scheduleTask.state = ScheduleTaskState::PROCESSING;
        Serial.println("[Async] Starting schedule update task");
    }

    if (scheduleTask.state == ScheduleTaskState::PROCESSING) {
        const int MAX_RETRIES = 5;  // Reduced from 10
        const uint32_t TASK_TIMEOUT_MS = 30000;  // 30 seconds total timeout

        // Check timeout
        if (nowMs - scheduleTask.startMs > TASK_TIMEOUT_MS) {
            scheduleTask.state = ScheduleTaskState::FAILED;
            scheduleTask.errorMessage = "Timeout after " + String(scheduleTask.retryCount) + " retries";
            Serial.printf("[Async] Schedule update timeout after %d retries\n", scheduleTask.retryCount);
        } else {
            // Try to send schedule
            String err;
            bool slowMode = scheduleTask.retryCount > 0;  // Use slow mode after first failure
            bool ok = tm4c.setSchedule(scheduleTask.pendingSchedule, err, slowMode);

            if (ok) {
                // Success!
                scheduleData = scheduleTask.pendingSchedule;
                scheduleTask.state = ScheduleTaskState::SUCCESS;
                scheduleTask.errorMessage = "";
                displayDirty = true;
                Serial.printf("[Async] Schedule update SUCCESS after %d attempts\n", scheduleTask.retryCount + 1);
            } else {
                // Failed, check if we should retry
                scheduleTask.retryCount++;
                Serial.printf("[Async] Schedule update retry %d/%d failed: %s\n",
                             scheduleTask.retryCount, MAX_RETRIES, err.c_str());

                if (scheduleTask.retryCount >= MAX_RETRIES) {
                    scheduleTask.state = ScheduleTaskState::FAILED;
                    scheduleTask.errorMessage = err.length() ? err : "Failed after retries";
                    Serial.printf("[Async] Schedule update FAILED: %s\n", scheduleTask.errorMessage.c_str());
                } else {
                    // Wait before next retry (non-blocking in next loop iteration)
                    delay(200);  // Small delay, but won't block for long
                }
            }
        }
    }

    // Start scan synchronously if requested (triggered by /scan), in loop to avoid blocking HTTP task.
    if (scanRequested) {
        scanRequested = false;
        WiFi.mode(WIFI_MODE_APSTA);  // ensure STA up
        WiFi.scanDelete();
        int res = WiFi.scanNetworks(false, true, false, 150);  // blocking, show hidden, passive off
        if (res >= 0) {
            cachedSsids.clear();
            for (int i = 0; i < res; ++i) {
                String ssid = WiFi.SSID(i);
                ssid.trim();
                if (ssid.length() == 0) continue;
                if (std::find(cachedSsids.begin(), cachedSsids.end(), ssid) == cachedSsids.end()) {
                    cachedSsids.push_back(ssid);
                }
            }
            Serial.printf("[Scan] done, found %d\n", res);
        } else {
            Serial.printf("[Scan] failed rc=%d\n", res);
        }
    }

    // Display button (active LOW) to cycle pages
    bool btnState = digitalRead(DISPLAY_BTN_PIN);
    if (!btnState && lastBtnState && millis() - lastBtnMs > 200) {
        lastBtnMs = millis();
        if (displayMode == DisplayMode::QR && currentStage != Stage::ONLINE) {
            Serial.println("[UI] Button ignored: not online, QR mode locked");
        } else {
            cycleDisplayMode();
        }
    }
    lastBtnState = btnState;

    // Track display mode changes to control targeted polling
    if (displayMode != lastDisplayMode) {
        if (displayMode == DisplayMode::DASH_STATUS) {
            fetchStatusOnce();
            statusPollActive = true;
            lastStatusPollMs = nowMs;
        } else {
            statusPollActive = false;
        }
        if (displayMode == DisplayMode::DASH_SCHEDULE) {
            scheduleNeedsFetch = true;
        }
        lastDisplayMode = displayMode;
    }

    // Status polling only while on status page (reduced from 3s to 5s)
    if (statusPollActive && nowMs - lastStatusPollMs > 5000) {
        lastStatusPollMs = nowMs;
        fetchStatusOnce();
    }

    // Schedule fetch when entering schedule page
    if (displayMode == DisplayMode::DASH_SCHEDULE && scheduleNeedsFetch) {
        scheduleNeedsFetch = false;
        fetchScheduleOnce();
    }

    // Daily NTP refresh and sync to TM4C when Wi-Fi is available.
    if (WiFi.status() == WL_CONNECTED) {
        const uint32_t DAY_MS = 24UL * 60UL * 60UL * 1000UL;
        if (lastNtpSyncMs == 0 || nowMs - lastNtpSyncMs > DAY_MS) {
            syncTimeAndTm4c();
        }
    }

    // Refresh dashboards when marked dirty
    if (displayDirty && displayMode != DisplayMode::QR) {
        renderCurrentDisplay();
    }

    // Refresh QR screen time display while online
    if (displayMode == DisplayMode::QR && currentStage == Stage::ONLINE) {
        String nowStr = currentTimeString();
        if (nowStr != lastQrTimeShown || lastQrTimeWarn != timeDesyncWarning) {
            renderCurrentDisplay();
        }
    }

    delay(20);
}
