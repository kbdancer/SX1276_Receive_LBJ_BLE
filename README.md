# SX1276_Receive_LBJ_BLE

**BLE** 固件变体：解码 + OLED + SD + BLE JSON。不含 WiFi。

| 项 | 值 |
|----|----|
| 变体名 | **BLE**（与 App 对齐） |
| 硬件 | LilyGO T3 V1.6（ESP32 + SX1276） |
| 通道 | BLE `LBJReceiver` / FFE0 / FFE1 |
| 配套 App | [LBJ_Console_BLE](https://github.com/kbdancer/LBJ_Console_BLE) |
| 姊妹固件 | [SX1276_Receive_LBJ_WiFi](https://github.com/kbdancer/SX1276_Receive_LBJ_WiFi) |

> 本仓库相对上游的裁剪与文档，主要由 AI（Cursor Agent）整理。

## 命名对照

| 变体 | 固件仓库 | App 仓库 |
|------|----------|----------|
| BLE | `SX1276_Receive_LBJ_BLE` | `LBJ_Console_BLE` |
| WiFi | `SX1276_Receive_LBJ_WiFi` | `LBJ_Console_WiFi` |

## 通道

| 通道 | 地址 | 说明 |
|------|------|------|
| BLE | `LBJReceiver` / FFE0 / FFE1 | notify 推送 + write 命令 |

## 命令（BLE write 一行 JSON）

| cmd | 作用 |
|-----|------|
| `sd_status` | SD 状态与容量 |
| `sd_clear` | 清除 SD 同步/日志历史 |
| `sync` | 回放未同步记录 |
| `sync_ack` | `{"cmd":"sync_ack","seq":N}` |

## 编译 / 烧录

```bash
cd SX1276_Receive_LBJ_BLE
pio run -t upload
pio device monitor
```

下载时若失败，可暂时取出 SD 卡（MISO=GPIO2）。
