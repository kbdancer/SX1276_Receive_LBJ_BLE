# SX1276_Receive_LBJ_BLE

**纯 BLE** 固件：解码 + OLED + SD + BLE JSON。不含 WiFi / NetJSON / Telnet / 蓝牙以外的无线上行。

| 项 | 值 |
|----|----|
| 硬件 | LilyGO T3 V1.6（ESP32 + SX1276） |
| 通道 | BLE `LBJReceiver` / FFE0 / FFE1 |
| 配套 App | [LBJ_Console_BLE](https://github.com/kbdancer/LBJ_Console_BLE) |
| 对照变体 | 同级目录 `SX1276_Receive_LBJ_NetJSON`（纯 WiFi NetJSON） |

> 本仓库/目录相对上游的裁剪与文档，主要由 AI（Cursor Agent）整理。

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

## RF 默认

- `TARGET_FREQ` / `INITIAL_PPM` / AFC：见 `src/utilities.h`
