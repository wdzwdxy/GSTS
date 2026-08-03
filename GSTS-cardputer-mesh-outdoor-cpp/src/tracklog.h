#pragma once
// GPX 轨迹落盘 (microSD)。每个点写入后文件始终保持合法 GPX:
// 追加点时 seek 回退覆盖固定长度的尾部, 写完点再补尾部。
#include <Arduino.h>
#include <SD.h>

class TrackLog {
public:
    bool active = false;
    uint32_t points = 0;
    double distanceM = 0;
    char path[48] = {0};

    // ---- 轨迹全览用的内存坐标缓冲(带自动抽稀, 无需 SD 即可绘制) ----
    static constexpr int PLOT_MAX = 500;   // 缓冲上限, 满了自动隔点合并
    float    plat[PLOT_MAX];
    float    plon[PLOT_MAX];
    uint16_t np        = 0;   // 缓冲内点数
    uint16_t plotDecim = 1;   // 当前抽稀倍数(每 plotDecim 个点入缓冲 1 个)

    static constexpr const char *FOOTER = "</trkseg></trk></gpx>\n";

    // y/mo/d/h/mi 用于文件名 (本地时间)
    bool start(int y, int mo, int d, int h, int mi) {
        snprintf(path, sizeof(path), "/track_%04d%02d%02d_%02d%02d.gpx", y, mo, d, h, mi);
        File f = SD.open(path, FILE_WRITE);
        if (!f) return false;
        f.print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                "<gpx version=\"1.1\" creator=\"CardputerMesh\" "
                "xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
                "<trk><name>Cardputer Mesh Track</name><trkseg>");
        f.print(FOOTER);
        f.close();
        points = 0; distanceM = 0;
        lastLat = lastLon = 0; hasLast = false;
        np = 0; plotDecim = 1; sinceKeep = 0;   // 清空全览缓冲
        active = true;
        return true;
    }

    // isoTime 形如 2026-07-23T09:54:00Z (UTC)
    bool addPoint(double lat, double lon, double alt, const char *isoTime) {
        if (!active) return false;
        File f = SD.open(path, "r+");
        if (!f) return false;
        size_t footLen = strlen(FOOTER);
        size_t sz = f.size();
        if (sz >= footLen) f.seek(sz - footLen);
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "\n<trkpt lat=\"%.6f\" lon=\"%.6f\"><ele>%.1f</ele>"
                 "<time>%s</time></trkpt>", lat, lon, alt, isoTime);
        f.print(buf);
        f.print(FOOTER);
        f.close();
        if (hasLast) distanceM += haversine(lastLat, lastLon, lat, lon);
        lastLat = lat; lastLon = lon; hasLast = true;
        points++;
        pushPlot(lat, lon);            // 同步进内存缓冲供全览页绘制
        return true;
    }

    void stop() { active = false; }

private:
    double lastLat = 0, lastLon = 0;
    bool hasLast = false;
    uint16_t sinceKeep = 0;            // 距上次入缓冲已跳过的点数

    // 把点写入全览缓冲; 缓冲满时隔点合并(整条轨迹形状保留, 内存有界)
    void pushPlot(double lat, double lon) {
        if (++sinceKeep < plotDecim) return;
        sinceKeep = 0;
        if (np >= PLOT_MAX) {                       // 满 -> 保留偶数下标, 点数减半
            uint16_t j = 0;
            for (uint16_t i = 0; i < np; i += 2) { plat[j] = plat[i]; plon[j] = plon[i]; j++; }
            np = j; plotDecim *= 2;
        }
        plat[np] = (float)lat; plon[np] = (float)lon; np++;
    }

    static double haversine(double la1, double lo1, double la2, double lo2) {
        const double R = 6371000.0, D = PI / 180.0;
        double dla = (la2 - la1) * D, dlo = (lo2 - lo1) * D;
        double a = sin(dla / 2) * sin(dla / 2) +
                   cos(la1 * D) * cos(la2 * D) * sin(dlo / 2) * sin(dlo / 2);
        return 2 * R * atan2(sqrt(a), sqrt(1 - a));
    }
};
