# ESP-AMS-S3

用 **ESP-IDF（C 语言）** 重写的拓竹打印机自动换料系统，目标芯片 **ESP32-S3（N16R8 模组，16MB Flash）**。

和仓库里那份 MicroPython 版是**两套独立实现**，可以并存：MicroPython 版还在 `python_code/`，这份在 `esp-ams-s3/`，互不影响各自的构建与发布流水线。

## 仓库结构

```
esp-ams-s3/
├── CMakeLists.txt          顶层工程文件（只指定工程名 + 引入 IDF 构建系统）
├── partitions.csv          分区表：双 OTA 分区 + NVS + storage
├── sdkconfig.defaults      默认配置（每一项都写了"为什么"，改前先读）
├── main/
│   ├── main.c              启动流程（只负责按顺序拉起各模块）
│   ├── board_pins.h        ★ 引脚表 —— 改接线只改这一个文件
│   ├── config.c/.h         NVS 持久化配置（对应 config.json + wifi.dat）
│   ├── wifi_manager.c/.h   WiFi 连接与配网热点（对应 network_model.py）
│   ├── web_server.c/.h     配置网页与 REST 接口（对应 AMS_WEB.py）
│   ├── bambu_mqtt.c/.h     打印机 MQTT over TLS（对应 bambu/bambu_mqtt.py）
│   ├── bambu_proto.c/.h    拓竹报文解析（对应 get_event_info.py + bambu_const.py）
│   ├── motor.c/.h          共享直流电机（LEDC PWM 双路输出）
│   ├── clutch.c/.h         4 路电磁离合 + 互斥仲裁（对应 motor_clutch.py）
│   ├── filament_sensor.c/.h 12 个微动 + 挤出机到位信号
│   ├── ams_controller.c/.h ★ 业务状态机（对应 AMS_MODEL.py）
│   ├── log_buffer.c/.h     日志环形缓冲（对应 logout.py）
│   └── CMakeLists.txt      编译单元 + 把 web/ 三个前端文件编进固件
├── web/
│   ├── index.html          配置界面（8 个面板）
│   ├── app.js              前端逻辑（只轮询 /status 一个接口）
│   └── style.css           样式（自适应明暗主题、手机优先）
└── tools/
    └── lint_c.py           没有工具链时用的 C 源码静态检查（见文末）
```

---

## 一、为什么要用 ESP-IDF 重写

不是因为"MicroPython 不好"，而是有几件事在 MicroPython 那套架构里**做不到**或在真机上**反复出问题**：

| 问题           | MicroPython 版                                                                                                                                                                  | ESP-IDF 版                                                                    |
| ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------- |
| **堆碎片**      | GC 不压缩。应用加载完后空闲还有 64KB，但**最大连续块只剩 3,584 字节**。于是"要一大块连续内存且只在启动时做一次"的操作（开射频约 24KB 连续堆）必须抢在应用 import 之前做完，`main.py` 的阶段划分全是为绕开这个限制，往前面插一个 import 就可能报 `Wifi Unknown Error 0x0101` | WiFi 驱动用**静态分配**缓冲，编译期定死，不存在这个问题。启动顺序可以按逻辑依赖来排，不用按内存来排                       |
| **网页卡死**     | 每个 handler 都得拆成 `await` 片段，否则 uasyncio 事件循环被按住 → 网页打不开。为此写了非阻塞收包、缓存探测、带超时的 ping 一堆防御代码                                                                                         | `esp_http_server` 跑在自己的任务里，handler **可以随便写阻塞代码**（扫 WiFi 2 秒、OTA 几十秒都没事）      |
| **MQTT 卡网页** | 同上，要写 `poll_msg()`、`mqtt_alive_cached()`、`tcp_reachable()`                                                                                                                     | `esp-mqtt` 自带任务，收包是回调，`bambu_mqtt_is_connected()` 只读一个标志、零网络 I/O             |
| **整机固件 OTA** | 分区表只有一个 factory 应用分区，没有备用分区 → 只能更新"应用与界面"（`.ams` 包），整机固件必须插 USB                                                                                                                | `ota_0` / `ota_1` 双分区 + `otadata` → 网页上直接升级**整机固件**，写入中途断电旧固件完好，新固件起不来还能自动回滚 |

另外 `sdkconfig.defaults` 里**在编译期**关掉了 WiFi 省电（`esp_wifi_set_ps(WIFI_PS_NONE)` 在 `wifi_mgr_init()` 里，另有一份 sdkconfig 说明），这一类"传大文件时掉关联"的问题从配置层消失。

---

## 二、硬件接线

### 执行机构

```
                 ┌── 离合1 ──> 料盘位1 送料轮
共享电机 ────────┼── 离合2 ──> 料盘位2 送料轮
(H桥 IN1/IN2)    ├── 离合3 ──> 料盘位3 送料轮
                 └── 离合4 ──> 料盘位4 送料轮
```

> ★ **硬性约束：任何时刻最多 1 路电磁离合吸合。**>   
> 两路同时咬合会让两卷料被同一个电机反向拉扯 → 料线绷断 / 打滑；两路线圈浪涌叠加还会把 5V 拉塌导致复位。>   
> 这条由 `clutch.c` **四重强制**（与 MicroPython 版 `FilamentMotorBus` 同样做法）：>   
> ① 吸合前无条件先全部断开 ② 断开后回读复核，发现残留就取消本次动作>   
> ③ 写引脚只有一个内部函数 `clutch_apply()`，不对外暴露，没有绕过仲裁的路径>   
> ④ `clutch_assert_single()` 由主循环每周期调一次，发现多路吸合立即全断

> ★ 电机那两路还有一个类似的硬件红线：**IN1 / IN2 绝不同时为高**（H 桥上下管直通会当场烧驱动芯片）。>   
> `motor_apply()` 的写引脚顺序因此固定为"先双路清零、再给目标通道赋值"，并额外拦截"两路都非 0"的调用。>   
> **不要把这个顺序"优化"成先写目标通道。**

### 引脚表（`main/board_pins_s3.h`，ESP32-S3 42 针，8 通道）

| 功能                  | GPIO                        | 说明                          |
| ------------------- | --------------------------- | --------------------------- |
| 电机 H 桥 IN1 / IN2   | 4 / 5                       | 进料（正转）/ 退料（反转）             |
| 电磁离合 1 ~ 8         | 6, 7, 15, 16, 17, 18, 21, 47 | 高电平吸合；经 ULN2803 反推，见下表     |
| ISEN 电流采样          | 3                           | 200mΩ 采样电阻，200mV/A           |
| 状态 LED              | 42                          | 不需要就改成 `-1`                 |
| 微动 1 ~ 8（每路 1 个 ADC 脚） | 1, 2, 9, 8, 10, 11, 12, 13  | **每脚读两只微动**（电阻编码），见下文    |
| RS-485               | 14(TX) / 48(RX) / 39(DE)    | BambuBus 直连打印机              |
| 补充机满足信号 IN_EXT    | 0                           | 输入，仅上报不参与控制                 |
| 挤出机到位信号            | 不接 GPIO                    | 走打印机 MQTT 上报事件              |

**微动电气接法（每路 1 个 ADC 脚读两只开关）：**

```
       开始位微动(常开) ──4.7K──┐
                              ├── GPIO ── 内部上拉 45K ── 3V3
       停止位微动(常闭) ──10K ──┘
```

靠电压分三档判定：**≈220mV** = 开始位压下（缓冲器空，要补料）· **≈600mV** = 中间（保持上一拍）· **≈3300mV** = 停止位顶开（满了，停）。判定阈值 `< 433mV` / `> 1050mV`，按"内部上拉 31K~58K 全范围都不误判"选定。

> ⚠️ **ADC2 与 WiFi 互斥**：8 路微动里有 3 路（GPIO11/12/13）固定落在 ADC2，而 ADC2 由 WiFi 射频仲裁。**MQTT 模式（WiFi 开）这 3 路读不到**，固件会自动降级并在网页 / 日志里标明原因；**485 直连（WiFi 关）8 路全部正常** —— 这是最终形态。
>
> ⚠️ **GPIO3 是 strapping 脚**（JTAG 源选择）。ISEN 经 200Ω 到地，上电无电流时 ≈0V，ROM 会选"外部 JTAG"；固件随后把 39/42 重新配成普通 GPIO，功能不受影响。若 485 方向脚或状态灯异常，优先怀疑这里。

上电时 `main.c` 会打印完整接线表，并跑一次自检，查三件事：同一只脚被两个角色占用 / 踩到保留脚（Flash、USB、UART）/ strapping 脚被当输出用。**只报警不中断** —— 让用户能连上网页看到这条错误再决定怎么改。

---

## 三、每路两只微动 + 缓冲送料

### 两只微动的语义（`ams_controller.c`）

| 微动       | 触发后                                  |
| -------- | ------------------------------------ |
| **停止送料** | 立刻停电机（料到位 / 料尽 / 机构顶到头都很常见）          |
| **开始送料** | 按常规速度送料，直到停止送料微动触发或超时                |

「缓冲送料」这一个开关管这两只：开始位触发 → 送料，停止位触发 → 停。

> **「自吸上料」功能已于 2026-10-23 整套删除**（现场要求）。原第三只「自吸微动」
> 和它触发的「送料 → 等挤出机到位 → 蠕动 3 次」全自动流程都已去掉；
> `ams_config_t` 里原来那个使能掩码字节改名保留为占位，**不删字段**，
> 以免结构体长度变化导致老设备上的 WiFi / MQTT / 通道映射整份配置被清掉。

### 换料的送料闭环（`feed_until_extruder_ms`）

```
① 吸合目标路离合 + 电机进料（2 秒一段，边推边盯打印机的到位信号）
        │
        ▼
② 打印机报「耗材已到挤出机」→ 立刻停送料
        │  （没等到则推满 AMS_LOAD_TOTAL_MS 先停机，等下一阶段再补推）
        ▼
③ 等 1 秒让打印机齿轮把料头咬稳
        │
        ▼
④ 蠕动送料 × 5 次收尾（CREEP_EXCHANGE_TIMES × CREEP_EXCHANGE_MS = 1 秒）
        │  慢速短脉冲把料一点点推进齿轮咬合区，
        │  否则料只是虚顶在挤出机入口，打印机一拉就可能脱开
        ▼
⑤ 停电机 → 断开离合（并等机械彻底脱开）
```

第 ④ 步是这套机构的关键：送料到位后耗材往往只是**虚顶在挤出机入口**，还没有被挤出机齿轮咬住。慢速短脉冲才能保证后续打印时挤出机真的能拉动料。

蠕动参数（次数 / 单次脉冲时长 / 速度）在网页「微动开关」面板上可改，存在 NVS 里。

> 「挤出机到位信号」如果机器上没有空闲 IO 能引出来，可以在网页上把来源切成 **MQTT 事件**，改由打印机上报驱动，就不用接那根线了。

---

## 四、编译与烧录

### 前置：装 ESP-IDF

**必须是 v5.x。** 本项目用了 v5 的 API（`esp_chip_info()`、`OTA_SIZE_UNKNOWN`、`esp_mqtt_client_config_t` 的嵌套字段写法），**降到 v4.x 会直接编译失败** —— v4 里 MQTT 配置还是扁平字段（`uri` / `username` / `password`）。

```bash
# Linux / macOS
git clone -b v5.3.2 --recursive https://github.com/espressif/esp-idf.git ~/esp-idf
~/esp-idf/install.sh esp32s3
. ~/esp-idf/export.sh
```

Windows 用 `install.bat` / `export.bat`，或直接用 VS Code 的 ESP-IDF 插件。

### 编译与 CI

**GitHub Actions 自动编译**：`.github/workflows/build.yml` 在 push 到 `main` 或打 tag 时自动触发：

```
push main → 编译 → 上传 artifacts（供下载）
push tag  → 编译 → 创建 GitHub Release + 上传固件文件
```

产物包含：

| 文件                    | 用途                          |
| --------------------- | --------------------------- |
| `firmware.bin`        | 仅应用（OTA 升级用）                |
| `bootloader.bin`      | 引导加载器                       |
| `partition-table.bin` | 分区表                         |
| `esp-ams-s3-full.bin` | 合并文件（bootloader + 分区表 + 应用） |
| `version.txt`         | 编译元信息                       |

**刷写合并文件（推荐）**：

```bash
esptool --port COM3 --chip esp32s3 write_flash 0x0 esp-ams-s3-full.bin
```

**或分开刷**：

```bash
esptool --port COM3 --chip esp32s3 write_flash \
  0x0 bootloader.bin \
  0x8000 partition-table.bin \
  0x20000 firmware.bin
```

**发布 Release**：打 tag 后自动创建：

```bash
git tag v0.1.0
git push origin v0.1.0
```

### 第一次上电

1. 串口会打印版本、芯片、接线表、引脚自检结论。
2. 如果 NVS 里没有能连上的 WiFi 记录 → 自动开热点 **`AMS_WIFI`**（密码 `A12345678`）。
3. 手机连上热点，浏览器打开 **<http://192.168.4.1>** ，在「网络」页填 WiFi。
4. 配网成功后同一网络下访问板子 IP（也支持 \*\*<http://ams.local\*\*）。>

> ⚠️ **ESP32 默认不回 ICMP，`ping` 不通是正常的** —— 别用 ping 判断板子在不在线，看网页或串口日志。

---

## 五、Web 配置界面

`web/` 下的三个文件通过 `EMBED_FILES` **编译进固件**（`.rodata` 段），不挂文件系统 —— 没有"SPIFFS 挂载失败导致网页打不开"这一类故障，也没有额外工具依赖。

| 面板    | 作用                                      |
| ----- | --------------------------------------- |
| 状态    | 网络 / 打印机 / 业务 / 内存 / 固件版本 / 复位原因 / 启动次数 |
| 诊断    | 硬件自检结论、离合违规计数、换料成功失败统计、设备日志            |
| 网络    | 扫描附近 WiFi、连接、开关配置热点                     |
| 打印机   | MQTT 参数（IP / 序列号 / 访问码），**读回时不回传访问码**   |
| 通道    | 打印机通道号 ↔ 物理料盘位映射、通道颜色                   |
| 硬件调试  | 单通道点动（进退响应时间可调）、紧急停止                    |
| 微动开关  | 逐路开关「缓冲送料」（决定走微动还是降级模式）、蠕动参数           |
| 固件升级  | 上传整机 `.bin` 升级（真双分区 OTA）                |

前端每 2 秒只请求 **`/status` 一个接口**（约 2KB）——不是每块面板一个请求。这样既省连接又不会出现"某块面板的数据比另一块旧一个周期"的错位。

### 复位原因诊断

`/status` 会回一个把 `esp_reset_reason()` 翻成"下一步该查什么"的说明，而不是复述代号。例如 `BROWNOUT` → "★ 欠压复位。供电不足是最常见的原因：换 USB 口、换短线、外接 5V（离合吸合瞬间电流很大，USB 口带不动）"。

---

## 六、分区表（16MB Flash）

`partitions.csv` 按 **16MB Flash（N16R8 模组）** 排（ota 分区 3MB，留足余量）：

| 分区         | 偏移       | 大小   | 用途                            |
| ---------- | -------- | ---- | ----------------------------- |
| `nvs`      | 0x9000   | 24KB | WiFi / 打印机参数 / 通道映射 / 当前料盘    |
| `otadata`  | 0xF000   | 8KB  | "下次从哪块 app 启动"，`esp_ota` 自动维护 |
| `phy_init` | 0x11000  | 4KB  | 射频校准数据                        |
| `ota_0`    | 0x20000  | 3MB  | 应用（当前版本）                      |
| `ota_1`    | 0x300000 | 3MB  | 应用（备用版本）                      |
| `storage`  | 0x600000 | 1MB  | 预留（放可热更资源 / 导出日志；网页资源现在编在固件里） |
| `coredump` | 0x700000 | 4KB  | 核心转储（调试用）                     |

### 双分区 OTA 的回滚保险

`main.c` 的最后一步是：

```c
if (web_ok) {
    esp_ota_mark_app_valid_cancel_rollback();
}
```

**这一句不能省。** 开了 `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` 之后，新固件启动后如果不主动"确认有效"，下一次重启引导程序就认为它有问题、自动切回上一版 —— 现象是"升级成功了，但一重启又变回旧版本"，非常难查。放在最后（WiFi 和网页都起来了）是刻意的：**只有体检通过才确认**。新固件要是把网页写坏了，就不会确认，下次重启自动回滚。

---

## 七、MQTT 换料通信与 G-code 宏

### 换料触发原理

打印机要换料时，在 G-code 里调用 `M73 P101 R[next_extruder]`，打印机会把自身状态置成：

```
gcode_state  = "PAUSE"
mc_percent   = 101    ← 拓竹约定的"等 AMS 换料"标记
```

固件端判定条件：

```c
change_needed = (gcode_state == "PAUSE" && mc_percent == 101)
```

**这个判定与 G-code 宏里的 `M73 P101` 是一一对应的。**

### 目标通道号解析

`M73 P101 R[next_extruder]` 里的 `R` 参数告诉打印机"切到哪个通道"。不同固件版本把 `R` 值回传到 MQTT 上报里用的字段名不一样，固件按以下顺序探测：

| 优先级 | 字段名             | 说明     |
| --- | --------------- | ------ |
| 1   | `filament_next` | 最常见    |
| 2   | `mc_next_tray`  | 部分固件版本 |
| 3   | `mc_tray_idx`   | 早期版本   |
| 4   | `next_tray`     | 兜底     |

都没找到时退回 `mc_remaining_time`（⚠️ 该字段名义上是"剩余打印时间（分钟）"，不是通道号，仅作兜底，匹配率很低）。

`filament_next` 是 **0 起索引**（0 = 通道1，1 = 通道2…），255 表示不换料。固件收到后 +1 转成 1 起。

### 自动颜色匹配（4 通道）

G-code 宏里 `M1002 set_filament_type:{filament_type[next_extruder]}` 会把目标颜色回传到上报里（`ams.filament_color` 或 `ams.color` 字段，格式 `#RRGGBB`）。

固件端的匹配流程：

```
打印机上报 target_color (0xRRGGBB)
        │
        ▼
config_color_match()  ←  4 个通道的 color_list[] 里找最近的
        │
        ├── 匹配成功 → 用匹配到的通道号覆盖默认的 filament_next
        └── 匹配失败（4 个通道都没配颜色）→ 退回按 filament_next 换料
```

颜色距离用**感知加权欧氏距离**（整数运算）：

```
d² = 900×ΔR² + 3481×ΔG² + 121×ΔB²
```

权重来自 BT.601 亮度系数（0.30² / 0.59² / 0.11²），人眼对绿色最敏感、蓝色最不敏感。

### G-code 冲洗阶段（P102-P105）

G-code 宏里 `M73 P102/P103/P104/P105` 是冲洗阶段（flush），由打印机自己完成，AMS 不需要介入。固件端的状态机**只识别 P101**，冲洗阶段运行期间 `mc_percent` 会变成 102-105，但 `change_needed` 始终为 false，AMS 不会误动作。

### 冲洗阶段时序（对应 G-code 宏）

```
M73 P101 R[next]     ←  触发换料，AMS 开始动作
M400 U1               ←  打印机等待 AMS 完成
  （AMS: 退料 → 进料 → 辅助推料 → 通知打印机）
M400                  ←  同步
M1002 set_filament_type:UNKNOWN
M73 P102 R[flush_1]   ←  冲洗阶段 1（打印机自己处理）
  G1 E23.7 / 脉冲挤出 …
M73 P103 R[flush_2]   ←  冲洗阶段 2（如果有）
  G1 E… × 5 脉冲
M73 P104 R[flush_3]   ←  冲洗阶段 3（如果有）
M73 P105 R[flush_4]   ←  冲洗阶段 4（如果有）
M1002 set_filament_type:{filament_type[next]}
T[next_extruder]       ←  切换到新通道
M400 / M106 / M109    ←  收尾
```

### 换料完整时序（AMS 侧）

```
打印机请求换料（P101）
        │
        ▼
handle_report() 判定 change_needed
        │
        ▼
auto_match_channel()（可选，有颜色时覆盖通道号）
        │
        ▼
ams_post_cmd(AMS_CMD_EXCHANGE, channel)
        │
        ▼
do_exchange():
  ① 退料（M400; G1 E-50 F200 → 打印机退料 → AMS 收料回盘）
  ② 进料（do_load：吸合离合 → 电机正转 → 等停止微动 / 超时）
  ③ 辅助推料（M400; G1 E50 F200 → 打印机拉料 → AMS 辅助推一小段）
  ④ 记录状态（config_set_filament_current）
        │
        ▼
bambu_mqtt_send_resume()  ←  通知打印机继续打印
```


````

### 挤出机到位信号（hw_switch_state）

打印机在 `device/{序列号}/report` 的 `print` 子对象里上报 **`hw_switch_state`** 字段，指示耗材是否已到位：

| 值 | 含义 |
| --- | --- |
| `0` | 挤出机内无耗材（OFF） |
| 非零（1/2/3） | 挤出机内有耗材（ON） |

不同机型/固件值域可能不同（双喷头可能用 2/3 编码），但**非零一律视为"有耗材"**。

这个字段对应 [ha-bambulab](https://github.com/greghesp/ha-bambulab) 的 `extruder_filament_state` 二进制传感器，HA 里的 ON/OFF 就是它的 `bool(hw_switch_state)` 结果。

固件端在 `bambu_proto.c` 的 `probe_extruder_inplace()` 里探测，`extruder_inplace_hint` 字段返回值：
- `1` = 有耗材（`hw_switch_state` 非零）
- `0` = 无耗材（`hw_switch_state` 为 0）
- `-1` = 报文里没有该字段（某些固件版本不上报）

当来源配置为「MQTT 事件」时，AMS 的换料流程用这个信号代替 GPIO 硬件线来判定耗材是否到位。

### MQTT 连接参数

| 项目 | 值 |
| --- | --- |
| 端口 | 8883（TLS） |
| 用户名 | `bblp` |
| 密码 | 打印机「设置 → 网络 → 访问码」 |
| 订阅 | `device/{序列号}/report` |
| 发布 | `device/{序列号}/request` |
| 证书 | 自签，跳过校验（不影响链路加密） |

连接在 `main.c` 的 `ams_connect_printer()` 中建立，位于 `start_network()` 之后，避免 MQTT 阻塞 WiFi 初始化。

---

## 八、验证状态（重要）

截至最后一次改动，这份代码的验证情况：

| 项目 | 状态 |
| --- | --- |
| `tools/lint_c.py` 静态检查（22 个 .c/.h） | ✅ 0 处可疑点 |
| `tools/lint_c.py --selftest`（判据自身可靠性） | ✅ 8/8 通过 |
| 跨模块函数声明 ↔ 定义 ↔ 调用一致性 | ✅ 逐项比对无缺失 |
| ESP-IDF v5 API 用法（`esp_chip_info` / `esp_ota_*` / `httpd_*` / `ledc_*` / MQTT 嵌套配置） | ✅ 逐项核对 |
| GitHub Actions CI 真编译 | ✅ 通过（`.github/workflows/build.yml`） |
| 实机烧录验证 | ✅ WiFi 连接 / 网页访问 / 心跳日志正常 |

> 已挂 GitHub Actions CI（`.github/workflows/build.yml`），push 后自动编译并上传产物，打 tag 自动创建 Release。

### `tools/lint_c.py`

没有交叉编译器时用的替代品，抓四类问题：

1. **代码位置出现中文字符**（最可靠的一条）。C 的标识符只能是 ASCII，中文合法出现的位置只有字符串和注释 —— 所以剔掉字符串和注释之后还能扫到中文，就一定是某个字符串被提前截断了，后面的中文掉出来变成了"代码"。
   这条比"数引号奇偶"可靠得多：`"可能检测到"意面"缺陷（打印乱丝）。"` 这行引号是**成对**的（6 个，偶数），奇偶判据完全看不见，但掉出来的 `意面` 一定暴露。**这条判据正是被一个真实 bug 逼出来的。**
2. 字符串引号数量为奇数（引号落单，作为补充判据）
3. 括号 / 花括号 / 方括号不配对（按整个文件累计，不能按行判 —— `}` 本来就经常单独占一行）
4. 代码位置出现全角标点

```bash
python tools/lint_c.py .          # 扫当前工程
python tools/lint_c.py --selftest # 先验证判据本身没坏（8 条用例）
````

两个命令都返回非 0 表示有问题，可以直接当 CI 的门禁用。
