# SX1276_Receive_LBJ_fusion

融合版精简：**上游解码 / OLED / SD + BLE JSON**。  
WiFi、NetJSON、Telnet 已从固件去除，优先保证解码可用内存。

含 WiFi/NetJSON 的完整备份：同级目录 `SX1276_Receive_LBJ_fusion_backup_wifi_*`。  
**最简固件源码快照**：同级目录 `SX1276_Receive_LBJ_minimal_*`（见其中 `MINIMAL.md`）。

## 通道

| 通道 | 地址 | 说明 |
|------|------|------|
| BLE | `LBJReceiver` / FFE0 / FFE1 | notify 推送 + write 命令 |

对接 App：[LBJ_Console](https://github.com/undef-i/LBJ_Console)（仅 BLE 连接收机）。

## 命令（BLE write 一行 JSON）

| cmd | 作用 |
|-----|------|
| `sd_status` | SD 状态与容量 |
| `sd_clear` | 清除 SD 同步/日志历史 |
| `sync` | 回放未同步记录 |
| `sync_ack` | `{"cmd":"sync_ack","seq":N}` |

## 事件

- 列车：原字段 + `seq`（同步回放带 `"sync":true`）
- 响应：`{"type":"resp","cmd":"...","ok":true/false,...}`

## 离线同步

- 有 SD：写入 `/SYNC/records.jsonl`，BLE 连上可自动 sync  
- 无 SD：仅实时推送给当前 BLE 客户端
