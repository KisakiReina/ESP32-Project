# Proximity Unlock — 基于 BLE 扫描的智能门锁

![esp32c6](https://img.shields.io/badge/target-ESP32--C6-blue)

一个 BLE 外设设备，当**已配对的手机靠近门**时驱动舵机开锁，仅使用 **BLE 扫描**——配对后不再重复连接。

## 工作原理

### 配对

1. 按下 **GPIO4** 上的按钮进入配对模式（60 秒超时）。
2. 在手机蓝牙设置中连接到 "Proximity Unlock"。
3. 在两侧确认配对码。
4. 配对成功后，设备**断开连接**并进入**扫描模式**。

> 配对后不再建立任何连接，系统仅监听扫描结果。

### 开锁逻辑

`
BLE 扫描（持续，不启用重复过滤）
   │
   ├── 设备已配对？
   │    ├── 否 → 忽略
   │    └── 是 → 检查扫描缓存
   │              ├── 在缓存中（30 秒内见过）→ 在室内，忽略
   │              └── 不在缓存中 → 检查 RSSI
   │                                    ├── RSSI > -50 dBm → 开锁！
   │                                    └── RSSI ≤ -50 dBm → 等待（不缓存）
`

- **观察窗口**：进入扫描模式后 10 秒内。在此期间看到的任何已配对设备会被静默加入缓存（它们已经在室内）。
- **室内超时（TTL）**：30 秒。如果某个设备在 30 秒内未被 BLE 扫描到，则不再视为"在室内"，再次靠近时可触发新的开锁。
- **RSSI 阈值**：-50 dBm。手机必须非常靠近门才能开锁。

### 关锁

开锁后，舵机保持打开状态 **5 秒钟**，然后自动旋转回锁定位置。

### 硬件接线

| 组件 | 引脚 |
|------|------|
| 舵机（信号线） | 可通过 menuconfig 配置（默认取决于配置） |
| 按钮（配对） | GPIO4（低电平有效，内部上拉） |

### 舵机动作

| 动作 | 旋转角度 |
|------|----------|
| 锁定（初始位置） | 180° |
| 开锁 | 180° → 30° |
| 持续时间 | 800ms |

### 多用户支持

可配对多部手机。第一个靠近的已配对手机将触发开锁。开锁后，5 秒保持计时器开始运行——在此期间其他手机到达不会重复触发。


# 设置环境（根据你的配置调整路径）
set IDF_PATH=E:\esp\v6.0\esp-idf
set IDF_TOOLS_PATH=C:\Users\<user>\.espressif
set PATH=%IDF_TOOLS_PATH%\python_env\idf6.0_py3.11_env\Scripts;%PATH%

cd e:\ESP-Project\esp_hid_device
idf.py build
idf.py -p COM<x> flash
`

查看监控输出：
`ash
idf.py -p COM<x> monitor
`

## 配置

运行 idf.py menuconfig → **Proximity Unlock Configuration**：
- **Servo GPIO pin** — 更改舵机信号线连接的 GPIO

## 项目结构

`
main/
├── CMakeLists.txt              — 构建配置
├── Kconfig.projbuild           — 菜单配置选项
├── esp_hid_device_main.c       — 核心逻辑：扫描、缓存、开锁
├── esp_hid_gap.c/.h            — BLE GAP 层（扫描、广播）
├── servo_control.c/.h          — SG90 舵机 PWM 驱动
`

## 关键参数（在 main.c 中修改）

| 宏定义 | 默认值 | 说明 |
|-------|-------|------|
| RSSI_THRESHOLD_NEAR | -50 | 触发开锁的 RSSI 阈值（dBm） |
| UNLOCK_HOLD_TIME_US | 5000000 | 开锁保持时间（5 秒） |
| SCAN_CACHE_TTL_US | 30000000 | 室内设备 TTL（30 秒） |
| SCAN_GRACE_PERIOD_US | 10000000 | 扫描开始后的观察窗口（10 秒） |
| SERVO_ANGLE_UNLOCK | 30 | 开锁时舵机角度（度） |
| SERVO_ANGLE_LOCK | 180 | 锁定时舵机角度（度） |
| SERVO_MOVE_TIME_MS | 800 | 舵机旋转时间（毫秒） |
