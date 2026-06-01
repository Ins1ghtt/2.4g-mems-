# 2.4g-mems-
在宋仓库的基础上对rx_link_tx修改，实现从IIM-42352传感器到nrf54l15的数据通路，采用fifo半满逻辑，传输速率2.4m左右
┌─────────────────────────────────────────────────────────────┐
│  LP核 (FLPR / cpuflpr)                                      │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ adc_sampler.c: 假数据 → IIM-42352 FIFO 半满中断模式  │  │
│  │ prj.conf: +SPI +GPIO +WORKQUEUE_STACK               │  │
│  │ overlay: +SPI00 pinctrl +IIM-42352 device node       │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  HP核 (cpuapp)                                              │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ main.c: +send_binary_frame() 二进制帧串口输出        │  │
│  │ debug_uart: +write_raw() 原始字节流支持              │  │
│  │ proto.h: 采样率 50kHz → 12kHz (真实 4kHz×3轴)        │  │
│  │ overlay: +spi00 disabled (让给 FLPR)                 │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  工具链                                                     │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ +save_mems_binary.py (串口→.bin采集)                 │  │
│  │ +view_mems.py (帧数据查看)                           │  │
│  │ +.vscode/settings.json                               │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘


//详细内容如下
# 工程差异说明

## 概述

本文档描述以下两个工程之间的差异：

| 标识 | 路径 | 说明 |
|------|------|------|
| **工程 A** | `d:\nrf54l15cc` | 当前开发版（FIFO 半满中断模式） |
| **工程 B** | `d:\nRF54L15-2.4G-codex-rf-link-2p4g-devlog\...` | 参考/原始版（虚假 ADC 测试模式） |

**核心变更**：工程 A 在工程 B 的基础上，将 LP 核（FLPR）的数据采集从"顺序递增假数据"改造为**真实 IIM-42352 MEMS 加速度计 FIFO 半满中断模式**，并增加了 HP 核二进制帧串口输出功能。

---

## 一、新增文件（工程 A 有，工程 B 无）

| 文件路径 | 说明 |
|----------|------|
| `.vscode/settings.json` | VS Code 工作区配置（指定 rf_link_tx 为活跃应用） |
| `save_mems_binary.py` | Python 脚本：通过串口接收二进制 MEMS 帧并保存为 .bin/.hex |
| `view_mems.py` | Python 脚本：查看 mems_frames.bin 中的帧数据 |
| `mems_frames.bin` | 已采集的 MEMS 帧二进制数据文件 |
| `mems_frames.hex` | 已采集的 MEMS 帧 hex 转储文件 |

---

## 二、删除的配置项（工程 B 有，工程 A 移除）

| 文件路径 | 差异 |
|----------|------|
| `applications/rf_link_rx/prj.conf` | 工程 B 包含 `CONFIG_NRFX_DPPI10=y`，工程 A 已移除 |

---

## 三、修改文件详细对比

### 3.1 `applications/rf_link_tx/flpr_app/src/adc_sampler.c` —— **核心改造**

| 项目 | 工程 B（原始） | 工程 A（当前） |
|------|---------------|---------------|
| 行数 | 31 行 | 544 行 |
| 功能 | 生成顺序递增假数据 | 完整 IIM-42352 SPI + FIFO 半满中断驱动 |

**工程 A 新增功能：**
- IIM-42352 寄存器定义（WHO_AM_I、FIFO_CONFIG、INT_SOURCE0 等）
- SPI 单寄存器读/写辅助函数
- FIFO 突发读取（burst read）最多 256 字节
- 环形缓冲区（384 个 int16_t，4 帧 × 96 采样）
- GPIO INT1 中断回调 → 系统工作队列异步处理
- `fifo_work_handler()`：读 INT_STATUS、FIFO_COUNT、突发读取 → 解析 8 字节包 → 提取 XYZ 三轴加速度 → 写入环形缓冲区
- `adc_sampler_init()`：完整的 10 步传感器初始化序列（含 FIFO flush、STREAM 模式、WM=128 packets、4 kHz ODR、INT1 路由、诊断验证）
- `adc_sampler_fill_frame()`：通过信号量等待 32 组数据，超时容错处理

### 3.2 `applications/rf_link_tx/flpr_app/prj.conf` —— LP 核配置

```diff
 CONFIG_MBOX=y
 CONFIG_IPC_SERVICE=y
 CONFIG_IPC_SERVICE_BACKEND_ICMSG=y
 CONFIG_ASSERT=y

+CONFIG_SPI=y
+CONFIG_GPIO=y
+
 CONFIG_MAIN_STACK_SIZE=4096
+CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048
 CONFIG_IPC_SERVICE_BACKEND_ICMSG_WQ_STACK_SIZE=2048
 CONFIG_PBUF_RX_READ_BUF_SIZE=256
```

**说明：** 新增 SPI/GPIO 驱动支持，增加系统工作队列栈大小以支持 FIFO 中断处理。

### 3.3 `applications/rf_link_tx/flpr_app/boards/nrf54l15_connectkit_nrf54l15_cpuflpr.overlay` —— LP 核设备树

| 项目 | 工程 B | 工程 A |
|------|--------|--------|
| 行数 | 45 行 | 80 行 |
| SPI | 无 | SPI00 + IIM-42352 设备节点 |
| GPIO | 无 | gpio2 启用 |
| pinctrl | 无 | SCK=P2.01, MOSI=P2.02, MISO=P2.04 |
| CS | 无 | P2.00 (GPIO 管理, 低有效) |
| uart30 | `/delete-property/ hw-flow-control` | `status = "disabled"` |
| SPI 频率 | — | 8 MHz |

### 3.4 `applications/rf_link_tx/boards/nrf54l15_connectkit_nrf54l15_cpuapp.overlay` —— HP 核设备树

工程 A 新增以下节点（HP 核禁用 SPI00，将其让给 FLPR）：

```dts
/* SPI00 is assigned to FLPR for IIM-42352 accelerometer */
&spi00 {
    status = "disabled";
};
```

### 3.5 `applications/rf_link_tx/src/proto.h` —— 协议参数

```diff
-#define RF_LINK_TARGET_SAMPLE_HZ   50000u
+/* Aggregate sample rate: 3 axes × 4 kHz ODR = 12000 samples/s
+ * Frame period = 96 samples / 12000 Hz = 8 ms (32 accel groups @ 4 kHz)
+ */
+#define RF_LINK_TARGET_SAMPLE_HZ   12000u
```

**说明：** 采样率从 50 kHz 虚假 ADC 改为 12 kHz（3 轴 × 4 kHz ODR），帧周期相应调整为 8 ms。

### 3.6 `applications/rf_link_tx/src/main.c` —— HP 核主程序

工程 A 新增：
- `#include <string.h>`
- `send_binary_frame()` 函数：通过 UART 输出二进制帧（`0xAA 0x55 + len + payload`）
- 在主循环中发射前先调用 `send_binary_frame(&frame)` 输出到串口

```diff
+static void send_binary_frame(const struct rf_frame *frame)
+{
+    static const uint8_t sync[2] = { 0xAA, 0x55 };
+    uint16_t len = (uint16_t)sizeof(struct rf_frame);
+    uint8_t len_bytes[2] = { (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
+    debug_uart_write_raw(sync, 2);
+    debug_uart_write_raw(len_bytes, 2);
+    debug_uart_write_raw((const uint8_t *)frame, sizeof(struct rf_frame));
+}
```

主循环变更：
```diff
+       send_binary_frame(&frame);
        ret = radio_link_send_frame(&frame, K_MSEC(5));
```

### 3.7 `applications/rf_link_tx/src/debug_uart.c` / `debug_uart.h` —— 调试串口

工程 A 新增 `debug_uart_write_raw()` 函数，用于输出原始字节流（支持二进制帧协议）：

```c
void debug_uart_write_raw(const uint8_t *data, uint32_t len);
```

---

## 四、无差异文件（关键模块确认一致）

以下文件在两个工程中完全相同：

- `applications/rf_link_tx/src/tx_queue.c` / `tx_queue.h`
- `applications/rf_link_tx/src/radio_link.c` / `radio_link.h`
- `applications/rf_link_tx/src/ipc_bridge.c` / `ipc_bridge.h`
- `applications/rf_link_tx/src/lp_trace.h`
- `applications/rf_link_tx/src/fatal_diag.c`
- `applications/rf_link_tx/flpr_app/src/main.c`
- `applications/rf_link_tx/flpr_app/src/ipc_tx.c` / `ipc_tx.h`
- `applications/rf_link_tx/flpr_app/src/sample_buffer.c` / `sample_buffer.h`
- `applications/rf_link_tx/flpr_app/CMakeLists.txt`
- `applications/rf_link_tx/prj.conf`
- `applications/rf_link_rx/` 目录下所有源码文件
- 根目录脚本：`dump_rx_frames.py`、`export_fake_adc_csv.py`、`save_serial_csv.py`

---
