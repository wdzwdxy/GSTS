#pragma once
// ============ Cardputer Mesh 户外应用 - 全局配置 ============

// ---------------- 轨迹记录 ----------------
#define RECORD_INTERVAL_S  5          // 每隔 X 秒记录一个定位点

// ---------------- GNSS (ATGM336H) ----------------
#define GNSS_BAUD          115200     // 若无数据可尝试 9600
#define GNSS_RX_PIN        15         // ESP RX  <- GNSS TX
#define GNSS_TX_PIN        13         // ESP TX  -> GNSS RX

// ---------------- SPI 总线 (LoRa 与 SD 共享) ----------------
#define PIN_SPI_SCK        40
#define PIN_SPI_MISO       39
#define PIN_SPI_MOSI       14
#define PIN_SD_CS          12

// ---------------- SX1262 (Cap LoRa-1262) ----------------
#define PIN_LORA_NSS       5
#define PIN_LORA_DIO1      4
#define PIN_LORA_RST       3
#define PIN_LORA_BUSY      6
#define IOE_I2C_ADDR       0x43       // PI4IOE5V6408, P0 拉高接通天线开关

// ---------------- LoRa / Meshtastic (CN 区域) ----------------
// !! LORA_FREQ 必须与你的 stock Meshtastic 节点一致 (meshtastic --info 的 Freq)
// CN 公式: 470.0 + slot*0.125 MHz
#define LORA_FREQ          470.125f   // MHz (默认 slot=1, 按实际修改!)
#define LORA_BW            250.0f     // kHz  (CN LONG_FAST)
#define LORA_SF            11
#define LORA_CR            5          // 4/5
#define LORA_POWER         17         // dBm (CN 上限 19)
#define LORA_PREAMBLE      16         // Meshtastic 固定 16
#define LORA_SYNCWORD      0x2B       // Meshtastic (SX126x 寄存器展开为 0x24B4)

// ---------------- 本机身份 / SOS ----------------
#define MY_NODE_ID         0x11223344UL  // 自定义 32 位节点号(非零)
#define CHANNEL_INDEX      0             // 主信道(空PSK, 不加密) = 救援频道
#define SOS_INTERVAL_S     15            // SOS 循环发送间隔(秒)
#define SOS_HOP_LIMIT      5             // 最大转发跳数
#define SOS_TEXT           "SOS! Need rescue 需要救援"

// ---------------- SOS 防误触 (两段式触发) ----------------
// 触发流程: 先按 Fn+L 解锁 -> 解锁后连按两下 S 才发 SOS
#define SOS_ARM_TIMEOUT_MS 5000          // 解锁后无操作自动重新上锁(毫秒)
#define SOS_DOUBLE_MS      800           // 两次 S 之间的最大间隔(毫秒)

// ---------------- 省电 (自动息屏) ----------------
#define SCREEN_BRIGHTNESS  80            // 正常背光亮度 (0-255)
#define SCREEN_OFF_MS      30000         // 无按键自动息屏(毫秒); SOS 模式强制常亮

// ---------------- 开机载入画面 (GSTS) ----------------
// 时间轴: 0~500ms 四字依次淡入 -> 停留(期间完成硬件初始化) -> 末 500ms 自上而下卷帘切入主界面
#define SPLASH_MARGIN      5             // 四周留白像素
#define SPLASH_TOTAL_MS    2000          // 载入画面总时长(毫秒)
#define SPLASH_STAGGER_MS  100           // 相邻字母淡入的起始间隔
#define SPLASH_LETTER_MS   200           // 单字淡入时长 (100*3+200=500ms 全部亮起)
#define SPLASH_WIPE_MS     500           // 末尾卷帘过渡时长
