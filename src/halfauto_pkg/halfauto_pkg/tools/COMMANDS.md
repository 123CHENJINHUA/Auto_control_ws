# 常用 TCP 命令（摘自 `Core/Src/tcpclient.c`）

下面为工程中已实现并可用于测试的命令表。通常以纯文本发送（建议追加换行 `\n`），设备收到并执行后多数会回复 `finish\r\n` 并在 UART 输出调试信息。

| 命令 | 作用 | 预期回复 | 备注 |
|---|---:|---|---|
| `hello?` | 测试命令，询问设备是否在线 | `hi, i am here\r\n`（及其它提示） | 用于连通性检查 |
| `startDrill` | 启动钻头 / 开启相关 GPIO（示例为点亮紫外灯） | `finish\r\n` | MCU 也会在 UART 打印状态 |
| `stopDrill` | 停止钻头 / 关闭相关 GPIO | `finish\r\n` | |
| `slowShunScrew` | 电机1 顺向慢速（示例通过 PWM 控速） | `finish\r\n` | “Shun” 表示顺时针/正向 |
| `mediumShunScrew` | 电机1 顺向中速 | `finish\r\n` | |
| `fastShunScrew` | 电机1 顺向快速 | `finish\r\n` | |
| `slowNiScrew` | 电机1 逆向慢速 | `finish\r\n` | “Ni” 表示逆向 |
| `mediumNiScrew` | 电机1 逆向中速 | `finish\r\n` | |
| `fastNiScrew` | 电机1 逆向快速 | `finish\r\n` | |
| `STOPSCREW` | 停止电机1（立即停止） | `finish\r\n` | 注意大小写匹配 |

## 测试建议
- 建议发送时追加换行：`--newline` 或 在命令后加 `\n`，便于解析和调试。代码中使用 `strncmp` 比较前缀，换行不会影响识别，但有助于手工操作工具（nc/telnet）。
- 若设备作为客户端（见 `Core/Src/tcpclient.c`），它会主动连接到 `192.168.0.5:5001`（可在 `Core/Inc/main.h` / `Core/Inc/tcpclient.h` 中修改）。可在 PC 上启动 server（`nc -l 5001`）并输入上表命令。此时设备会执行并回复 `finish\r\n`。
- 若设备作为服务器（`Core/Src/tcpserver.c`），可用 `nc <device_ip> 5001` 直接连接并测试 echo 功能。

## 快速示例
- 在 PC 上作为 server（等待 MCU 的 `tcpclient` 连接并顺序发送命令）：

```bash
python3 tools/tcp_command_tool.py server --port 5001 --commands-file tools/commands.txt --interval 1.0 --newline
```

- 直接作为 client 向设备发送（从文件逐条发送一次）：

```bash
python3 tools/tcp_command_tool.py client --host 192.168.0.100 --port 5001 --commands-file tools/commands.txt --interval 0.5 --repeat 1 --newline
```

## 参考源码位置
- 命令处理：`Core/Src/tcpclient.c`（函数 `client_recv` 中查询命令）
- 服务器（echo）：`Core/Src/tcpserver.c`

