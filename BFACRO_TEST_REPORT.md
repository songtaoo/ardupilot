# BFACRO 飞行模式测试报告

## 1. 测试对象

- 仓库分支：`Droneer4.5.7_BFACRO`
- 基础分支：`Droneer4.5.7`
- 飞行模式名称：`BFACRO`
- 四字符名称：`BFAC`
- 模式编号：`29`
- 目标板：`DroneerX6`
- 测试日期：2026-07-28

## 2. 功能范围

BFACRO 的设计目标是提供类似 Betaflight Acro/Manual 的手动飞行模式：

- 复用 `ACRO_RP_*`、`ACRO_Y_*`、`ACRO_OPTIONS`、`ACRO_THR_MID`
- 使用 Betaflight 风格三次方 Expo
- 目标速率通过 ArduPilot 接口以 `cd/s` 传递
- 强制纯 Rate Loop
- 手动油门不使用角度补偿和输入滤波
- 支持 AirMode
- 接入 failsafe、crash/parachute check 和 GCS 模式屏蔽

## 3. 源码审查结果

相对 `Droneer4.5.7` 的差异：

- 修改文件：11 个
- 新增代码：197 行
- 删除代码：19 行
- 工作区源码状态：干净

已确认的集成点：

- `Mode::Number::BFACRO = 29`
- `ModeBFAcro` 类、对象和 `mode_from_mode_num()` 注册入口完整
- `BFACRO/BFAC` 名称完整
- `MODE_BFACRO_ENABLED` 条件编译开关已加入
- `FLTMODE1～6`、`INITIAL_MODE` 元数据包含模式 29
- `FLTMODE_GCSBLOCK` 第 24 位说明已加入
- AirMode 辅助开关处理已接入
- 手动油门 failsafe 已接入
- crash check 和 parachute check 排除了 BFACRO
- ACRO 参数条件编译已扩展到 BFACRO

另外对所有 `ACRO` 特殊处理点进行了搜索，未发现 BFACRO 必须同步而遗漏的核心注册点。

## 4. SITL 编译验证

执行：

```bash
./waf configure --board sitl
./waf copter
```

结果：通过。

- 1292 个编译/链接任务完成
- `ArduCopter/mode_bfacro.cpp` 实际参与编译
- `arducopter` 链接成功
- 编译启用 `-Werror`
- 生成：
  - `build/sitl/bin/arducopter`
- 二进制中确认包含 `BFACRO` 标识

## 5. SITL 功能测试结果

### 5.1 参数注册与模式识别

已读取并确认：

```text
ACRO_OPTIONS     0
ACRO_THR_MID     0
ACRO_RP_RATE     360
ACRO_RP_EXPO     0.3
ACRO_Y_RATE      202.5
ACRO_Y_EXPO      0
```

执行：

```text
mode 29
```

结果：

```text
COMMAND_ACK command:176 result:0
HEARTBEAT custom_mode:29
```

结论：模式编号、模式对象和 MAVLink 模式切换正常。

### 5.2 解锁、Ground Idle 和手动油门

解锁成功：

```text
COMMAND_ACK command:400 result:0
HEARTBEAT base_mode:209 custom_mode:29
```

典型电机输出：

| RC3 油门 | 四路电机输出 |
|---:|---|
| 1000 | `1100, 1100, 1100, 1100` |
| 1100 | `1259, 1258, 1259, 1260` |
| 1300 | `1466, 1465, 1466, 1467` |

结论：

- BFACRO 可解锁
- 零油门进入 Ground Idle
- 手动油门能够直接、单调地控制电机输出
- 四路输出在无姿态输入时保持基本一致

### 5.3 Rate Loop 和 Expo

启用：

```text
param set GCS_PID_MASK 1
param set SR0_EXTRA1 10
```

通过 `PID_TUNING` 检查横滚轴目标速率。`PID_TUNING.desired` 的单位为 `rad/s`。

实测结果：

| RC1 | `PID_TUNING.desired` |
|---:|---:|
| 1750 | `+2.3148618 rad/s` |
| 2000 | `+6.2831850 rad/s` |
| 1250 | `-2.3148618 rad/s` |
| 1000 | `-6.2831850 rad/s` |

满杆值 `6.283185 rad/s` 等于 `360°/s`。1750 PWM 的目标值低于理想 50% 点，是因为 `norm_input_dz()` 扣除了 RC 死区，属于预期行为。

结论：

- `cd/s` 到内部 `rad/s` 的单位转换正确
- 满杆目标速率正确
- 三次方 Expo 生效
- 正负方向对称
- 纯 Rate Loop 工作正常

### 5.4 AirMode

设置：

```text
param set ACRO_OPTIONS 1
```

重新从 STABILIZE 进入 BFACRO 后，在零油门下施加横滚：

```text
SERVO_OUTPUT_RAW:
1150, 1300, 1234, 1235
```

结论：

- AirMode 进入时正确启用
- 零油门下 Rate Loop 未被重置
- 电机仍能产生姿态控制差动
- 退出 BFACRO 后 `ACRO_OPTIONS` 恢复为 0，清理路径正常

### 5.5 油门 failsafe

设置：

```text
FS_THR_ENABLE = 1
FS_THR_VALUE = 975
```

#### 飞行状态模拟

油门从 1150 降至 900 后：

```text
HEARTBEAT base_mode:217 custom_mode:6 system_status:5
```

结论：进入 RTL（模式 6），保持解锁，符合飞行状态 failsafe 逻辑。

#### 地面零油门模拟

在未提高油门、保持地面状态时降至 900：

```text
HEARTBEAT base_mode:81 custom_mode:29 system_status:5
STATUSTEXT: PreArm: Radio failsafe on
```

结论：清除 armed 位，直接上锁，符合地面手动油门 failsafe 逻辑。

### 5.6 GCS 模式屏蔽

设置：

```text
FLTMODE_GCSBLOCK = 16777216
```

其中 `16777216 = 1 << 24`。

结果：

- MAVProxy 可用模式列表中不再出现 BFACRO
- GCS 的 `mode 29` 被拒绝
- HEARTBEAT 保持 `custom_mode:0`

随后将 `FLTMODE6` 设置为 29，通过 RC5 进入 BFACRO，结果：

```text
HEARTBEAT custom_mode:29
```

结论：GCS 屏蔽只限制 GCS 入口，不影响 RC 模式切换。

### 5.7 INITIAL_MODE

设置：

```text
INITIAL_MODE = 29
```

重启后：

```text
HEARTBEAT base_mode:81 custom_mode:29
```

结论：BFACRO 可作为启动模式进入，并保持上锁状态。

测试完成后已恢复：

```text
INITIAL_MODE = 0
FLTMODE1 = 0
FLTMODE6 = 0
FLTMODE_GCSBLOCK = 0
ACRO_OPTIONS = 0
```

最终 HEARTBEAT：

```text
base_mode:81
custom_mode:0
```

### 5.8 原 ACRO 回归测试

通过 `mode 1` 进入原 ACRO：

```text
HEARTBEAT base_mode:209 custom_mode:1
```

原 ACRO 能够：

- 正常进入
- 正常解锁
- 输出 Rate 控制
- 产生电机差动
- 正常上锁并退出

原 ACRO 的目标速率与 BFACRO 不完全相同，这是因为原 ACRO 保留 Trainer、姿态回正和角度限制逻辑，属于预期差异。

## 6. DroneerX6 目标板编译

执行：

```bash
./waf configure --board DroneerX6
./waf copter
```

配置成功，目标信息：

- MCU：STM32H743xx
- Flash：2048 KB
- 保留区域：128 KB
- 可用固件空间：1920 KB

结果：

- 1176 个编译任务完成
- `mode_bfacro.cpp` 实际参与编译
- `arducopter` ELF 链接成功
- 目标固件二进制大小：1,680,732 B
- Text：1,676,999 B
- Data：3,712 B
- 估算剩余 Flash：约 239 KB，约 12.5%

已生成：

- `build/DroneerX6/bin/arducopter.bin`
- `build/DroneerX6/bin/arducopter.apj`
- `build/DroneerX6/bin/arducopter.abin`
- `build/DroneerX6/bin/arducopter_with_bl.hex`

Waf 最后自动打包阶段报告了两个执行权限错误：

```text
PermissionError: make_abin.sh
PermissionError: make_intel_hex.py
```

这是仓库脚本权限/环境问题，不是 C++ 编译、链接或 BFACRO 功能问题。已通过显式调用：

```bash
bash Tools/scripts/make_abin.sh ...
python3 Tools/scripts/make_intel_hex.py ...
```

完成等价打包，所有目标固件产物均已生成。

## 7. 总体结论

BFACRO 已通过当前阶段的源码、SITL 功能和 DroneerX6 目标板编译验证：

- 模式注册和编号正确
- 控制接口单位正确
- Betaflight Expo 正常
- 纯 Rate Loop 正常
- 手动油门、AirMode 和 failsafe 正常
- GCS、RC、INITIAL_MODE 三种入口正常
- 原 ACRO 未受破坏
- DroneerX6 固件可成功编译并完成打包

当前尚未完成的项目：

- DroneerX6 实机刷写
- 实机上锁状态启动检查
- 实机 BFACRO 模式切换
- 实机低风险悬停/姿态响应测试
- 实机 failsafe 和 AirMode 测试

实机测试前应先使用上锁状态、拆桨或安全测试架，并确认遥控器 failsafe、解锁开关和紧急停机策略。
