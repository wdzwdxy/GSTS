// ============================================================
// Cardputer Mesh Kit (K152) 户外应用
//   功能1: 轨迹记录  每 RECORD_INTERVAL_S 秒记录 GNSS 定位 -> SD 卡 GPX
//   功能2: 紧急 SOS  在救援频道(Meshtastic 主信道)循环广播求救报文
// 按键: [R] 开始/停止记录   [S] 触发 SOS   [ESC/`] 取消 SOS
// ============================================================
#include <M5Cardputer.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <SD.h>
#include "config.h"
#include "mesh_packet.h"
#include "tracklog.h"

// ---------------- 全局对象 ----------------
SX1262 radio = new Module(PIN_LORA_NSS, PIN_LORA_DIO1, PIN_LORA_RST, PIN_LORA_BUSY);
TinyGPSPlus gps;
HardwareSerial GNSS(1);
TrackLog tracker;
M5Canvas canvas(&M5Cardputer.Display);

bool sdOk = false, loraOk = false;
bool sosActive = false;
bool trackView = false;                // 轨迹全览页开关
uint32_t sosSentCount = 0;
// --- SOS 两段式防误触状态 ---
bool sosArmed = false;                 // Fn+L 解锁后进入"待触发"状态
uint32_t sosArmMs = 0;                 // 解锁/最近一次操作时刻(用于超时上锁)
uint8_t sosSPress = 0;                 // 解锁后 S 连按计数
uint32_t lastSPressMs = 0;             // 上次按 S 时刻(用于连按窗口)
uint32_t lastRecordMs = 0, lastSosMs = 0, recStartMs = 0;
bool sosFlash = false;
uint32_t lastFlashMs = 0;
// --- 自动息屏省电 ---
bool screenOn = true;                  // 当前背光状态(true=亮)
uint32_t lastActivityMs = 0;           // 最近一次按键时刻(用于自动息屏计时)

// 颜色 (RGB565)
#define C_BG      0x08A5   // 深蓝黑
#define C_BAR     0x1926   // 顶栏
#define C_TEXT    0xE73C   // 主文本(近白)
#define C_DIM     0x8410   // 灰
#define C_ACCENT  0x5E8B   // 青绿
#define C_BLUE    0x861F   // 亮蓝
#define C_AMBER   0xFDC9   // 琥珀
#define C_RED     0xE9A6   // 红
#define C_GREEN   0x2FEC   // 起点绿
#define C_SOSBG   0x78E2   // SOS 深红底

// ---------------- 天线开关: PI4IOE5V6408 P0 拉高 ----------------
static bool enableAntennaSwitch() {
    auto &i2c = m5::In_I2C;   // 内部 I2C (G8/G9), M5Cardputer.begin 已初始化
    uint8_t dir = 0, out = 0, hiz = 0xFF;
    if (!i2c.readRegister(IOE_I2C_ADDR, 0x03, &dir, 1, 400000)) return false;
    i2c.readRegister(IOE_I2C_ADDR, 0x05, &out, 1, 400000);
    i2c.readRegister(IOE_I2C_ADDR, 0x07, &hiz, 1, 400000);
    dir |= 0x01;               // P0 -> 输出
    out |= 0x01;               // P0 -> 高电平
    hiz &= ~0x01;              // P0 -> 关闭高阻
    bool ok = true;
    ok &= i2c.writeRegister8(IOE_I2C_ADDR, 0x03, dir, 400000);
    ok &= i2c.writeRegister8(IOE_I2C_ADDR, 0x05, out, 400000);
    ok &= i2c.writeRegister8(IOE_I2C_ADDR, 0x07, hiz, 400000);
    return ok;
}

// ---------------- SOS 发送 ----------------
// GNSS 日期时间 -> unix 秒 (供 POSITION 报文 time 字段)
static uint32_t gpsUnixTime() {
    if (!gps.date.isValid() || !gps.time.isValid() || gps.date.year() < 2020) return 0;
    long y = gps.date.year(), m = gps.date.month(), d = gps.date.day();
    if (m <= 2) { y--; m += 12; }
    long days = 365L * y + y / 4 - y / 100 + y / 400
                + (153L * (m - 3) + 2) / 5 + d - 719469L;
    return (uint32_t)days * 86400UL + gps.time.hour() * 3600UL
           + gps.time.minute() * 60UL + gps.time.second();
}

static void sendSos() {
    if (!loraOk) return;
    uint8_t buf[256];
    size_t n;
    // 有定位时: TEXT 与 POSITION(地图图钉) 交替广播; 无定位时只发 TEXT
    bool sendPos = gps.location.isValid() && (sosSentCount % 2 == 1);
    if (sendPos) {
        n = meshpkt::buildPosition(buf, meshpkt::BROADCAST, MY_NODE_ID,
                                   gps.location.lat(), gps.location.lng(),
                                   (int32_t)gps.altitude.meters(), gpsUnixTime(),
                                   (uint8_t)gps.satellites.value(),
                                   SOS_HOP_LIMIT, CHANNEL_INDEX);
    } else {
        char msg[160];
        if (gps.location.isValid()) {
            snprintf(msg, sizeof(msg), "%s @ %.5f,%.5f alt=%.0fm sats=%d #%lu",
                     SOS_TEXT, gps.location.lat(), gps.location.lng(),
                     gps.altitude.meters(), (int)gps.satellites.value(),
                     (unsigned long)(sosSentCount + 1));
        } else {
            snprintf(msg, sizeof(msg), "%s (no GPS fix) #%lu", SOS_TEXT,
                     (unsigned long)(sosSentCount + 1));
        }
        n = meshpkt::buildText(buf, meshpkt::BROADCAST, MY_NODE_ID, msg,
                               SOS_HOP_LIMIT, CHANNEL_INDEX);
    }
    int st = radio.transmit(buf, n);
    if (st == RADIOLIB_ERR_NONE) sosSentCount++;
}

// ---------------- 轨迹记录 ----------------
static void recordPoint() {
    if (!tracker.active || !gps.location.isValid()) return;
    char iso[24];
    snprintf(iso, sizeof(iso), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
    tracker.addPoint(gps.location.lat(), gps.location.lng(),
                     gps.altitude.meters(), iso);
}

static void startRecording() {
    if (!sdOk) return;
    // 文件名用 UTC+8 本地时间; 无定位时用开机毫秒占位
    int y = 1970, mo = 1, d = 1, h = 0, mi = 0;
    if (gps.date.isValid()) {
        y = gps.date.year(); mo = gps.date.month(); d = gps.date.day();
        h = (gps.time.hour() + 8) % 24; mi = gps.time.minute();
    } else {
        mi = (millis() / 60000) % 60; h = (millis() / 3600000) % 24;
    }
    if (tracker.start(y, mo, d, h, mi)) {
        recStartMs = millis();
        lastRecordMs = 0;
    }
}

// ---------------- UI ----------------
static void drawTopBar(bool rec) {
    canvas.fillRect(0, 0, 240, 18, C_BAR);
    canvas.setFont(&fonts::Font0);
    canvas.setTextDatum(middle_left);
    if (rec) {
        canvas.fillCircle(7, 9, 4, ((millis() / 500) & 1) ? C_RED : C_BAR);
        uint32_t el = (millis() - recStartMs) / 1000;
        canvas.setTextColor(C_RED, C_BAR);
        canvas.setCursor(15, 6);
        canvas.printf("REC %02lu:%02lu:%02lu", (unsigned long)(el / 3600),
                      (unsigned long)(el / 60 % 60), (unsigned long)(el % 60));
    } else {
        canvas.setTextColor(C_ACCENT, C_BAR);
        canvas.setCursor(5, 6);
        canvas.print("OUTDOOR");
    }
    canvas.setTextColor(C_DIM, C_BAR);
    canvas.setCursor(120, 6);
    canvas.printf("SAT%2d %s %s %3d%%", (int)gps.satellites.value(),
                  sdOk ? "SD" : "--", loraOk ? "LR" : "--",
                  M5Cardputer.Power.getBatteryLevel());
}

static void drawSoftKeys(const char *k1, const char *k2, const char *k3) {
    canvas.fillRect(0, 119, 240, 16, C_BAR);
    canvas.setFont(&fonts::efontCN_12);
    canvas.setCursor(4, 121);
    canvas.setTextColor(C_ACCENT, C_BAR);  canvas.print(k1);
    canvas.setTextColor(C_RED, C_BAR);     canvas.print("  "); canvas.print(k2);
    if (k3) { canvas.setTextColor(C_DIM, C_BAR); canvas.print("  "); canvas.print(k3); }
}

// SOS 已解锁提示横幅(覆盖在待机/记录/轨迹页顶部, 提示连按两下 S)
static void drawArmedBanner(uint32_t now) {
    bool blink = (now / 300) & 1;
    canvas.fillRect(0, 20, 240, 22, blink ? C_RED : C_SOSBG);
    canvas.setFont(&fonts::efontCN_12);
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextDatum(middle_center);
    uint32_t left = (SOS_ARM_TIMEOUT_MS - min((uint32_t)SOS_ARM_TIMEOUT_MS,
                                              now - sosArmMs)) / 1000 + 1;
    char b[48];
    snprintf(b, sizeof(b), "SOS已解锁: 连按2下[S]触发 (%lus)", (unsigned long)left);
    canvas.drawString(b, 120, 31);
    canvas.setTextDatum(top_left);
}

static void drawIdle() {
    canvas.fillSprite(C_BG);
    drawTopBar(false);

    bool fix = gps.location.isValid();
    char latStr[16], lonStr[16];
    if (fix) {
        snprintf(latStr, sizeof(latStr), "%.5f%c", fabs(gps.location.lat()),
                 gps.location.lat() >= 0 ? 'N' : 'S');
        snprintf(lonStr, sizeof(lonStr), "%.5f%c", fabs(gps.location.lng()),
                 gps.location.lng() >= 0 ? 'E' : 'W');
    } else {
        strcpy(latStr, "--.-----");
        strcpy(lonStr, "--.-----");
    }

    // 纬度 / 经度: 标签在左、数值在右, 横向排列(垂直居中对齐, 避免遮挡)
    canvas.setTextDatum(middle_left);
    canvas.setFont(&fonts::efontCN_12);
    canvas.setTextColor(C_DIM, C_BG);
    canvas.drawString("纬度", 8, 34);
    canvas.drawString("经度", 8, 62);

    canvas.setFont(&fonts::FreeSansBold12pt7b);
    canvas.setTextColor(fix ? C_TEXT : C_DIM, C_BG);
    canvas.drawString(latStr, 54, 34);
    canvas.drawString(lonStr, 54, 62);

    if (!fix) {
        canvas.setFont(&fonts::efontCN_12);
        canvas.setTextColor(C_AMBER, C_BG);
        canvas.drawString("正在搜星…", 176, 34);
    }

    // 高度 / 速度 / HDOP: 三项横向排成一行
    // 注: 用"高度"而非"海拔" —— 12px 点阵字体下"拔"的右半"犮"易被误认成"找"
    canvas.setFont(&fonts::efontCN_12);
    canvas.setTextColor(C_BLUE, C_BG);
    char ext[56];
    snprintf(ext, sizeof(ext), "高度%.0fm   速度%.1fkm/h   HDOP%.1f",
             gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
             gps.speed.isValid() ? gps.speed.kmph() : 0.0,
             gps.hdop.isValid() ? gps.hdop.hdop() : 99.9);
    canvas.drawString(ext, 8, 96);
    canvas.setTextDatum(top_left);

    drawSoftKeys("[R]开始记录", "[Fn+L]SOS", sdOk ? "[T]轨迹图" : "[T]轨迹(无SD)");
}

static void drawRecording() {
    canvas.fillSprite(C_BG);
    drawTopBar(true);
    struct { const char *label; char val[20]; } items[4];
    snprintf(items[0].val, 20, "%lu", (unsigned long)tracker.points);
    snprintf(items[1].val, 20, "%.2fkm", tracker.distanceM / 1000.0);
    snprintf(items[2].val, 20, "%.1fkm/h", gps.speed.isValid() ? gps.speed.kmph() : 0.0);
    snprintf(items[3].val, 20, "%.0fm", gps.altitude.isValid() ? gps.altitude.meters() : 0.0);
    items[0].label = "轨迹点"; items[1].label = "里程";
    items[2].label = "速度";   items[3].label = "高度";
    for (int i = 0; i < 4; i++) {
        int x = (i % 2) ? 124 : 8, y = (i < 2) ? 24 : 62;
        canvas.setFont(&fonts::efontCN_12);
        canvas.setTextColor(C_DIM, C_BG);
        canvas.setCursor(x, y); canvas.print(items[i].label);
        canvas.setFont(&fonts::FreeSansBold12pt7b);
        canvas.setTextColor(i < 2 ? C_TEXT : C_ACCENT, C_BG);
        canvas.setCursor(x, y + 13); canvas.print(items[i].val);
    }
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(C_DIM, C_BG);
    canvas.setCursor(8, 105);
    canvas.printf("interval %ds  %s", RECORD_INTERVAL_S, tracker.path);
    drawSoftKeys("[R]停止记录", "[Fn+L]SOS", "[T]轨迹图");
}

// ---------------- 轨迹全览页 (不加载地图, 纯矢量绘制) ----------------
static void drawTrackView() {
    canvas.fillSprite(C_BG);
    drawTopBar(tracker.active);

    uint16_t n = tracker.np;
    if (n < 2) {
        canvas.setFont(&fonts::efontCN_12);
        canvas.setTextColor(C_AMBER, C_BG);
        canvas.setTextDatum(middle_center);
        canvas.drawString("轨迹点不足", 120, 60);
        canvas.setTextColor(C_DIM, C_BG);
        canvas.drawString("开始记录并采集≥2个点后查看", 120, 78);
        canvas.setTextDatum(top_left);
        drawSoftKeys("[T]返回", tracker.active ? "[R]停止" : "[R]记录", "[S]SOS");
        return;
    }

    // 经纬度包围盒
    float minLa = 1e9f, maxLa = -1e9f, minLo = 1e9f, maxLo = -1e9f;
    for (uint16_t i = 0; i < n; i++) {
        float la = tracker.plat[i], lo = tracker.plon[i];
        if (la < minLa) minLa = la;  if (la > maxLa) maxLa = la;
        if (lo < minLo) minLo = lo;  if (lo > maxLo) maxLo = lo;
    }
    float lat0 = (minLa + maxLa) * 0.5f;
    float cosL = cosf(lat0 * 0.01745329f);          // 经度按纬度做等距圆柱投影修正
    float spanX = (maxLo - minLo) * cosL;
    float spanY = (maxLa - minLa);
    if (spanX < 1e-9f) spanX = 1e-9f;
    if (spanY < 1e-9f) spanY = 1e-9f;

    // 绘图区 (顶栏18 / 底部状态+软键)
    const int X0 = 4, Y0 = 20, W = 232, H = 84;
    float s = ((float)W / spanX < (float)H / spanY) ? (float)W / spanX : (float)H / spanY;
    float drawW = spanX * s, drawH = spanY * s;
    float offX = X0 + (W - drawW) * 0.5f;
    float offY = Y0 + (H - drawH) * 0.5f;
    auto SX = [&](float lo) { return (int)(offX + (lo - minLo) * cosL * s); };
    auto SY = [&](float la) { return (int)(offY + drawH - (la - minLa) * s); };  // 北在上

    // 轨迹线
    for (uint16_t i = 1; i < n; i++)
        canvas.drawLine(SX(tracker.plon[i - 1]), SY(tracker.plat[i - 1]),
                        SX(tracker.plon[i]),     SY(tracker.plat[i]), C_BLUE);
    // 起点(绿) / 终点(红)
    canvas.fillCircle(SX(tracker.plon[0]),     SY(tracker.plat[0]),     3, C_GREEN);
    canvas.fillCircle(SX(tracker.plon[n - 1]), SY(tracker.plat[n - 1]), 3, C_RED);

    // 指北标记
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(C_DIM, C_BG);
    canvas.drawLine(228, 22, 228, 30, C_DIM);
    canvas.drawLine(228, 22, 225, 26, C_DIM);
    canvas.drawLine(228, 22, 231, 26, C_DIM);
    canvas.setCursor(224, 32); canvas.print("N");

    // 底部信息: 缓冲点数/里程/范围(米)
    float wM = (maxLo - minLo) * 111320.0f * cosL;
    float hM = (maxLa - minLa) * 111320.0f;
    canvas.setFont(&fonts::efontCN_12);
    canvas.setTextColor(C_DIM, C_BG);
    canvas.setCursor(4, 106);
    canvas.printf("点%lu 里程%.2fkm 范围%.0fx%.0fm",
                  (unsigned long)tracker.points, tracker.distanceM / 1000.0, wM, hM);

    drawSoftKeys("[T]返回", tracker.active ? "[R]停止" : "[R]记录", "[S]SOS");
}

static void drawSos() {
    canvas.fillSprite(sosFlash ? C_SOSBG : C_RED);
    canvas.setTextDatum(top_center);
    canvas.setFont(&fonts::FreeSansBold24pt7b);
    canvas.setTextColor(TFT_WHITE);
    canvas.drawString("SOS", 120, 12);
    canvas.setFont(&fonts::efontCN_12);
    canvas.setTextColor(0xFDD7);
    canvas.drawString("正在救援频道循环广播…", 120, 58);
    canvas.setTextColor(TFT_WHITE);
    char line[64];
    uint32_t nextIn = SOS_INTERVAL_S - min((uint32_t)SOS_INTERVAL_S,
                                           (uint32_t)((millis() - lastSosMs) / 1000));
    snprintf(line, sizeof(line), "已发送 %lu 次   下次 %lus",
             (unsigned long)sosSentCount, (unsigned long)nextIn);
    canvas.drawString(line, 120, 76);
    if (gps.location.isValid())
        snprintf(line, sizeof(line), "%.5f%c %.5f%c  SAT %d",
                 fabs(gps.location.lat()), gps.location.lat() >= 0 ? 'N' : 'S',
                 fabs(gps.location.lng()), gps.location.lng() >= 0 ? 'E' : 'W',
                 (int)gps.satellites.value());
    else snprintf(line, sizeof(line), "无定位 (照常发送)");
    canvas.setTextColor(0xFDD7);
    canvas.drawString(line, 120, 94);
    canvas.fillRect(0, 119, 240, 16, 0x4863);
    canvas.drawString("[ESC] 取消SOS (记录不中断)", 120, 121);
    canvas.setTextDatum(top_left);
}

// ---------------- 开机载入画面: G S T S ----------------
// 时间轴: 0~500ms 四字依次淡入 -> 停留(期间跑硬件初始化) -> 末 500ms 自上而下卷帘切入主界面
// 排布: 一行四字, 字形不变形, 四周留白 SPLASH_MARGIN 像素
static uint32_t splashT0 = 0;

// RGB565 线性插值(用于淡入)
static uint16_t lerp565(uint16_t a, uint16_t b, float t) {
    if (t <= 0.0f) return a;
    if (t >= 1.0f) return b;
    int ar = (a >> 11) & 0x1F, ag = (a >> 5) & 0x3F, ab = a & 0x1F;
    int br = (b >> 11) & 0x1F, bg = (b >> 5) & 0x3F, bb = b & 0x1F;
    int r  = ar + (int)((br - ar) * t + 0.5f);
    int g  = ag + (int)((bg - ag) * t + 0.5f);
    int bl = ab + (int)((bb - ab) * t + 0.5f);
    return (uint16_t)((r << 11) | (g << 5) | bl);
}

// el = 距淡入开始的毫秒数; 第 i 个字母在 i*STAGGER 时刻起, 用 LETTER_MS 淡入
static void drawSplashFrame(int32_t el) {
    canvas.fillSprite(C_BG);
    canvas.setFont(&fonts::FreeSansBold18pt7b);
    canvas.setTextSize(2);                       // 18pt 字高约 25px, x2 后约 50px
    canvas.setTextDatum(middle_center);
    static const char *L[4] = {"G", "S", "T", "S"};
    const int cellW = (240 - SPLASH_MARGIN * 2) / 4;      // 内容区 230 四等分
    for (int i = 0; i < 4; i++) {
        float t = (float)(el - i * SPLASH_STAGGER_MS) / (float)SPLASH_LETTER_MS;
        canvas.setTextColor(lerp565(C_BG, C_TEXT, t), C_BG);
        canvas.drawString(L[i], SPLASH_MARGIN + cellW * i + cellW / 2, 135 / 2);
    }
    canvas.setTextSize(1);
    canvas.setTextDatum(top_left);
}

// 四个全亮 GSTS 字母(不含背景清屏) —— 供卷帘阶段叠加在最下方未揭开区域
static void drawSplashLetters() {
    canvas.setFont(&fonts::FreeSansBold18pt7b);
    canvas.setTextSize(2);
    canvas.setTextDatum(middle_center);
    static const char *L[4] = {"G", "S", "T", "S"};
    const int cellW = (240 - SPLASH_MARGIN * 2) / 4;
    for (int i = 0; i < 4; i++) {
        canvas.setTextColor(C_TEXT, C_BG);
        canvas.drawString(L[i], SPLASH_MARGIN + cellW * i + cellW / 2, 135 / 2);
    }
    canvas.setTextSize(1);
    canvas.setTextDatum(top_left);
}

// 阶段1: 逐字淡入(阻塞约 500ms)
static void splashBegin() {
    splashT0 = millis();
    Serial.printf("[splash] begin t=%lu\n", splashT0);
    const int32_t fadeEnd = SPLASH_STAGGER_MS * 3 + SPLASH_LETTER_MS;
    int32_t el;
    while ((el = (int32_t)(millis() - splashT0)) < fadeEnd) {
        drawSplashFrame(el);
        canvas.pushSprite(0, 0);
        delay(16);
    }
    drawSplashFrame(fadeEnd);                    // 四字全亮
    canvas.pushSprite(0, 0);
    Serial.printf("[splash] fade done t=%lu\n", millis());
}

// 阶段2+3: 停留补齐总时长 -> 自上而下卷帘揭开主界面
// 卷帘在 canvas 内部用 setClipRect 实现(不依赖显示 GRAM 残留, 也不碰 display 级裁剪区)
static void splashEnd() {
    while (millis() - splashT0 < (uint32_t)(SPLASH_TOTAL_MS - SPLASH_WIPE_MS)) delay(5);
    Serial.printf("[splash] wipe start t=%lu\n", millis());
    uint32_t tw = millis(), el;
    while ((el = millis() - tw) < (uint32_t)SPLASH_WIPE_MS) {
        drawIdle();                                  // 主界面画到底层
        int h = (int)(135UL * el / SPLASH_WIPE_MS); // 已揭开行数(从上)
        if (h < 135) {                               // 未揭开的下半部仍叠加载入字母
            canvas.setClipRect(0, h, 240, 135 - h);
            drawSplashLetters();
            canvas.clearClipRect();
        }
        canvas.pushSprite(0, 0);                     // 整帧推送, 始终有合法内容
        delay(12);
    }
    drawIdle();
    canvas.pushSprite(0, 0);
    Serial.printf("[splash] end t=%lu\n", millis());
}

// ---------------- 初始化 ----------------
void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);          // true = 启用键盘(ADV TCA8418)
    M5Cardputer.Display.setRotation(1);
    M5Cardputer.Display.setBrightness(SCREEN_BRIGHTNESS);
    canvas.createSprite(240, 135);
    canvas.setFont(&fonts::efontCN_12);

    splashBegin();                         // 载入画面: GSTS 逐字淡入

    // GNSS
    GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);

    // SPI 总线 (LoRa + SD 共享)
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, -1);
    sdOk = SD.begin(PIN_SD_CS, SPI, 20000000);

    // 天线开关 + SX1262 (与 Meshtastic CN LONG_FAST 对齐)
    bool ant = enableAntennaSwitch();
    int st = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                         LORA_SYNCWORD, LORA_POWER, LORA_PREAMBLE, 3.0f, true);
    if (st == RADIOLIB_ERR_NONE) {
        radio.setCRC(2);                   // Meshtastic: LoRa CRC 开启
        radio.setCurrentLimit(140);
        loraOk = ant;
    }
    Serial.printf("[init] SD=%d ANT=%d LoRa=%d(st=%d)\n", sdOk, ant, loraOk, st);

    splashEnd();                           // 停留补齐 2s 后, 卷帘切入主界面
    lastActivityMs = millis();             // 自动息屏计时从进入主界面开始
}

// ---------------- 主循环 ----------------
void loop() {
    M5Cardputer.update();
    while (GNSS.available()) gps.encode(GNSS.read());

    uint32_t now = millis();

    // 解锁超时: 解锁后 SOS_ARM_TIMEOUT_MS 内无有效操作则自动重新上锁
    if (sosArmed && now - sosArmMs > SOS_ARM_TIMEOUT_MS) {
        sosArmed = false;
        sosSPress = 0;
    }

    // 按键 (含息屏唤醒)
    bool keyEvent = M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed();
    if (keyEvent) {
        lastActivityMs = now;               // 有按键 -> 刷新活动时刻
        if (!screenOn) {                    // 息屏状态: 任意键仅唤醒屏幕, 不执行该键动作
            M5Cardputer.Display.setBrightness(SCREEN_BRIGHTNESS);
            screenOn = true;
            keyEvent = false;               // 消费本次按键(防止唤醒同时误触发)
        }
    }
    if (keyEvent) {
        bool fnHeld = M5Cardputer.Keyboard.keysState().fn;

        if (M5Cardputer.Keyboard.isKeyPressed('r')) {
            if (tracker.active) tracker.stop();
            else startRecording();
        }
        // --- SOS 两段式防误触 ---
        // 第一段: Fn + L 解锁
        if (fnHeld && M5Cardputer.Keyboard.isKeyPressed('l') && !sosActive) {
            sosArmed = true;
            sosArmMs = now;
            sosSPress = 0;
            lastSPressMs = 0;
        }
        // 第二段: 解锁后连按两下 S 才触发(单独按 S 无效, 防误触)
        else if (M5Cardputer.Keyboard.isKeyPressed('s') && !sosActive && sosArmed) {
            if (now - lastSPressMs > SOS_DOUBLE_MS) sosSPress = 0;  // 间隔过久重新计数
            sosSPress++;
            lastSPressMs = now;
            sosArmMs = now;                 // 刷新解锁超时
            if (sosSPress >= 2) {           // 连按两下 -> 正式触发
                sosActive = true;
                lastSosMs = 0;              // 立即发第一包
                sosSentCount = 0;
                sosArmed = false;
                sosSPress = 0;
            }
        }
        if (M5Cardputer.Keyboard.isKeyPressed('`')) {
            sosActive = false;              // ESC 取消 SOS
            sosArmed = false;               // 同时取消解锁待触发状态
            sosSPress = 0;
        }
        if (M5Cardputer.Keyboard.isKeyPressed('t')) {
            trackView = !trackView;         // 切入/切出轨迹全览页
        }
    }

    // 轨迹记录 (SOS 期间照常进行)
    if (tracker.active && now - lastRecordMs >= RECORD_INTERVAL_S * 1000UL) {
        lastRecordMs = now;
        recordPoint();
    }

    // SOS 循环广播
    if (sosActive && (lastSosMs == 0 || now - lastSosMs >= SOS_INTERVAL_S * 1000UL)) {
        lastSosMs = now;
        sendSos();
    }

    // SOS 红屏闪烁
    if (now - lastFlashMs >= 500) { lastFlashMs = now; sosFlash = !sosFlash; }

    // 自动息屏省电: SOS 模式强制常亮; 其他模式 30s 无按键自动息屏(按任意键唤醒)
    if (sosActive) {
        lastActivityMs = now;               // SOS 期间视为持续活动 -> 永不息屏
        if (!screenOn) { M5Cardputer.Display.setBrightness(SCREEN_BRIGHTNESS); screenOn = true; }
    } else if (screenOn && now - lastActivityMs > SCREEN_OFF_MS) {
        M5Cardputer.Display.setBrightness(0);   // 关背光息屏(GNSS/记录/LoRa 照常运行)
        screenOn = false;
    }

    // 绘制 (~5Hz); 息屏时跳过绘制以省电
    static uint32_t lastDraw = 0;
    if (screenOn && now - lastDraw >= 200) {
        lastDraw = now;
        if (sosActive) {
            drawSos();                      // SOS 优先级最高(安全)
        } else {
            if (trackView) drawTrackView();
            else if (tracker.active) drawRecording();
            else drawIdle();
            if (sosArmed) drawArmedBanner(now);  // 解锁待触发提示横幅
        }
        canvas.pushSprite(0, 0);
    }
    delay(5);
}
