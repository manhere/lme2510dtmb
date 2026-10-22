# lme2510dtmb — LME2510/LME2511 DTMB USB 棒合并版 out-of-tree 内核驱动

三项目优点合并版（out-of-tree，dvb-usb-v2 框架）。目标硬件：**LME2511 + LGS8GL5 + MAX2165**（VID 0x3344，热态 PID 0x1120 / 冷态 0x1111）。

## 来源与合并点

| 来源 | 取用的优点 |
|---|---|
| [ahtsiu/dybd_dtmb_driver](https://github.com/ahtsiu/dybd_dtmb_driver)（基线骨架） | dvb-usb-v2 模块化布局、streaming_ctrl 无自拆、**流期间禁 EP1 I2C** 纪律、固件单文件 + ACK 0x88/0x77 校验、MAX2165 ROM 校准 + 每次 tune reg04 `\|0xF0` latch、LGS8GL5 反相每轮失败翻转锁定、set_frontend 后补发流重启 |
| [HiSilicon-Development/linux-6.12.y](https://github.com/HiSilicon-Development/linux-6.12.y)（lme2510-dtmb.c） | 冷态 PID 0x1111（完全冷插可绑定）、EP8A 8 字节帧解析与强度/质量换算公式、demod_read_hint（0x85 读命令第 5 字节回显末次写值）、OVERFLOW 容忍（记录后继续轮询，**不做** EP1 降级——遵守流期间禁 EP1） |
| [xeonliu/CH1_DTMB_RE_cpp](https://github.com/xeonliu/CH1_DTMB_RE_cpp)（C++ 用户态） | 任意 1–16 PID 硬件过滤命令格式（CMD03×2 + CMD06 提交）、EP8A 默认启用决策（C++ 实测 EP8A 与 TS 流并存无冲突） |

相对 dybd 基线的差异：

1. **id_table 增加冷态 0x1111** —— 完全冷插可自动绑定并灌固件（修 dybd 冷插间隙）；
2. **EP8A 状态轮询默认开启**（`ep8a_status=0` 可关）—— 锁定状态、信号强度、信噪比全部走 EP8A 公式换算（锁定后 50–80% 真实显示），不再像 dybd 流期间报 0；
3. **硬件 PID 过滤**（默认关闭，`modprobe lme2510dtmb 时 dvb_usb_force_pid_filter_usage=1` 开启）—— 钩子只维护内存表，CMD03 提交固定在流启动前的 EP1 窗口内执行，遵守 MCU I2C/TS 总线冲突约束；
4. **demod_read_hint** 协议细节补全；
5. 清理：ts_carry 死代码删除、固件上传日志降为 dev_dbg、流路径 dev_info 降为 dev_dbg、函数中段 static 上移、PID 0x0000 合法值修正。

## 构建

```bash
# 需要内核头文件；dvb_usb.h 为私有头，Makefile 自带 fetch-headers
make fetch-headers   # 仅首次
make
sudo make install    # 同时安装 /lib/firmware/dvb-usb-lme2510dtmb.fw 与 lmedm04 blacklist
sudo modprobe lme2510dtmb
```

固件 `dvb-usb-lme2510dtmb.fw`（5348B）由 `rom.dat`(512B) + `fw_new.bin`(4836B) cat 生成，md5 与实测可用的 C++ 版内嵌 blob 一致（9d0c405f… / 1f38019e…）。

**必须 blacklist `dvb_usb_lmedm04`**（同 VID/PID 冲突，make install 已自动写入 /etc/modprobe.d）。

## 模块参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `ep8a_status` | 1 | EP8A 中断状态轮询；若实机出现 MCU 冲突（dybd 作者声称的现象）可 `ep8a_status=0` 回退 I2C 读取（流期间返回缓存值） |
| `debug_lgs8gl5` | 0 | 解调前端调试日志 |
| `debug_max2165` | 0 | 调谐器调试日志 |
| `dvb_usb_force_pid_filter_usage` | 0 | dvb-usb-v2 框架自带参数；置 1 启用硬件 PID 过滤（见上方风险说明） |

deb / `make install` 会把默认值写入 `/etc/modprobe.d/lme2510dtmb.conf`，按需直接编辑该文件（`options lme2510dtmb ep8a_status=1 debug_lgs8gl5=0 debug_max2165=0`），重新插拔或 reload 后生效。

## 与各来源的已知取舍

- **TS 重同步未移植**：内核 `dvb_dmx_swfilter` 自带 0x47 同步容错，收益低；
- **G75 路径未实现**：实机确认为 LGS8GL5（chip_id==0x0E），demod attach 会校验并在非 GL5 时拒绝；
- **HiSilicon 的 22D0 固件变体未实现**：该 blob 不可得，且本机不需要；
- **流中加 PID（hw filter 开启时）会即时经 EP1 提交**：lmedm04 主线驱动同硬件家族如此工作，但本固件未验证——默认全通路径完全无此风险；
- **流停止不发 {0x0a}**：沿用 dybd 行为（URB 停止后 MCU 输出被丢弃）。

## 验证清单

1. 冷插（未上电/换 USB 口）：`lsusb` 应先见 0x3344:0x1111 → 灌固件后重枚举为 0x3344:0x1120，`dmesg | grep lme2510dtmb` 无 error；
2. `dmesg` 确认 `EP8A status polling started` 与 `LGS-8GL5 DTMB demod attached`、chip_id=0x0E；
3. `tzap`/`dvbv5-zap` 或 tvheadend 锁频，观察锁定位与信号质量（EP8A 公式值，锁定后典型 50–80%）；
4. 收看 10 分钟确认无 TS 卡顿（EP8A 与流并存的最终实测）；
5. 若有卡顿：`rmmod lme2510dtmb && modprobe lme2510dtmb ep8a_status=0` 对比。

## 文件结构

```
lme2510dtmb_core.c       dvb_usb_device_properties / id_table / streaming_ctrl / pid_filter 钩子
lme2510dtmb_fw...c       冷热判定 + 50B 分包上传 + ACK 校验
lme2510dtmb_mcu.c        EP1 MCU 命令（PID 过滤 CMD03×2+CMD06、流启停、CMD16）
lme2510dtmb_i2c.c        EP1 I2C 桥（demod_read_hint 记账在此）
lme2510dtmb_lgs8gl5.c    LGS8GL5 解调（识别、训练、反相翻转锁定，dybd 已验证逻辑）
lme2510dtmb_max2165.c    MAX2165 调谐器（ROM 校准 + 0xF0 latch）
lme2510dtmb_status.c     EP8A URB 轮询 + 强度/质量公式
dvb_usb.h                框架私有头（vendored，make fetch-headers 可更新）
rom.dat / fw_new.bin     固件原料（与 C++ 版逐字节一致）
```
