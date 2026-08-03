#pragma once
// Meshtastic 兼容空口报文构造
// 结构 = 16B PacketHeader(小端) + protobuf Data
//   0x00 4B to | 0x04 4B from | 0x08 4B id
//   0x0C 1B flags(bit0-2=hop_limit, bit5-7=hop_start)
//   0x0D 1B channel = (chIndex<<3)|(from&0x07)
//   0x0E 1B next_hop | 0x0F 1B relay_node(=from&0xFF)
// 主信道(空PSK)不加密, stock 节点可直接解出 TEXT_MESSAGE。
#include <Arduino.h>
#include <esp_random.h>

namespace meshpkt {

constexpr uint32_t BROADCAST  = 0xFFFFFFFFUL;
constexpr uint8_t  PORT_TEXT  = 1;   // TEXT_MESSAGE_APP
constexpr uint8_t  PORT_POS   = 3;   // POSITION_APP (地图图钉)

inline size_t encodeVarint(uint32_t v, uint8_t *out) {
    size_t n = 0;
    do {
        uint8_t b = v & 0x7F;
        v >>= 7;
        out[n++] = v ? (b | 0x80) : b;
    } while (v);
    return n;
}

inline void putU32LE(uint8_t *p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

// 组装文本消息空口包, 返回总长度 (buf 需 >= 16+8+textLen)
inline size_t buildText(uint8_t *buf, uint32_t to, uint32_t from,
                        const char *text, uint8_t hopLimit, uint8_t chIndex) {
    size_t textLen = strlen(text);
    uint32_t pid = esp_random();
    uint8_t hl = hopLimit & 7;

    putU32LE(buf + 0x00, to);
    putU32LE(buf + 0x04, from);
    putU32LE(buf + 0x08, pid);
    buf[0x0C] = hl | (hl << 5);                       // hop_limit == hop_start
    buf[0x0D] = ((chIndex & 0x1F) << 3) | (from & 0x07);
    buf[0x0E] = 0;                                    // next_hop
    buf[0x0F] = from & 0xFF;                          // relay_node

    size_t n = 16;
    // protobuf Data: field1 varint portnum, field2 bytes payload
    buf[n++] = 0x08;
    n += encodeVarint(PORT_TEXT, buf + n);
    buf[n++] = 0x12;
    n += encodeVarint((uint32_t)textLen, buf + n);
    memcpy(buf + n, text, textLen);
    n += textLen;
    return n;
}

inline void putU32Fixed(uint8_t *p, uint32_t v) {   // protobuf fixed32/sfixed32 小端
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
}

// zigzag 编码 (protobuf sint32, 用于海拔可为负)
inline uint32_t zigzag32(int32_t v) { return ((uint32_t)v << 1) ^ (uint32_t)(v >> 31); }

// 组装 POSITION_APP 空口包 -> 收到方在 Meshtastic 地图上显示位置图钉
// Position protobuf 字段(meshtastic/mesh.proto):
//   1 latitude_i  sfixed32 = lat*1e7   tag 0x0D
//   2 longitude_i sfixed32 = lon*1e7   tag 0x15
//   3 altitude    int32(varint)        tag 0x18
//   4 time        fixed32(unix 秒)     tag 0x25
//  19 sats_in_view uint32(varint)      tag 0x98 0x01
// Data 外层: field1 portnum=3, field2 payload bytes
inline size_t buildPosition(uint8_t *buf, uint32_t to, uint32_t from,
                            double lat, double lon, int32_t altM,
                            uint32_t unixTime, uint8_t sats,
                            uint8_t hopLimit, uint8_t chIndex) {
    uint32_t pid = esp_random();
    uint8_t hl = hopLimit & 7;

    putU32LE(buf + 0x00, to);
    putU32LE(buf + 0x04, from);
    putU32LE(buf + 0x08, pid);
    buf[0x0C] = hl | (hl << 5);
    buf[0x0D] = ((chIndex & 0x1F) << 3) | (from & 0x07);
    buf[0x0E] = 0;
    buf[0x0F] = from & 0xFF;

    // 先编码 Position 子消息
    uint8_t pos[40];
    size_t pn = 0;
    int32_t latI = (int32_t)(lat * 1e7);
    int32_t lonI = (int32_t)(lon * 1e7);
    pos[pn++] = 0x0D; putU32Fixed(pos + pn, (uint32_t)latI); pn += 4;  // latitude_i
    pos[pn++] = 0x15; putU32Fixed(pos + pn, (uint32_t)lonI); pn += 4;  // longitude_i
    if (altM != 0) {                                                    // altitude (int32 varint)
        pos[pn++] = 0x18;
        if (altM >= 0) pn += encodeVarint((uint32_t)altM, pos + pn);
        else {  // 负数 int32 varint = 10 字节, 这里按 64 位补符号简化处理
            uint64_t v = (uint64_t)(int64_t)altM;
            do { uint8_t b = v & 0x7F; v >>= 7; pos[pn++] = v ? (b | 0x80) : b; } while (v);
        }
    }
    if (unixTime) { pos[pn++] = 0x25; putU32Fixed(pos + pn, unixTime); pn += 4; }  // time
    if (sats) {                                                                    // sats_in_view (field 19)
        pos[pn++] = 0x98; pos[pn++] = 0x01;
        pn += encodeVarint(sats, pos + pn);
    }

    // Data 外层
    size_t n = 16;
    buf[n++] = 0x08;
    n += encodeVarint(PORT_POS, buf + n);
    buf[n++] = 0x12;
    n += encodeVarint((uint32_t)pn, buf + n);
    memcpy(buf + n, pos, pn);
    n += pn;
    return n;
}

}  // namespace meshpkt
