# Codex交接：ArduPilot BFACRO + MATLAB/Simulink高速机仿真

更新时间：2026-08-03。把本文件复制到新电脑，并在新Codex任务开始时要求它完整读取。

## 1. 总目标和当前接续点

目标：

1. 在ArduPilot Copter 4.5.7保留原ACRO，新增类似Betaflight Acro/Manual的BFACRO。
2. 使用ArduPilot SITL + MATLAB/Simulink JSON联合仿真。
3. 将教学型四旋翼模型升级为最大速度约110m/s、前飞倾角接近80度的高速穿越机模型。

当前状态：

- BFACRO代码、SITL功能、DroneerX6编译已验证。
- MATLAB脚本版JSON模型已学习并制作中文注释版。
- Betaflight Blackbox已完成第一轮筛选和统计。
- Simulink与SITL静止状态通信已跑通，约400fps、100%实时。
- 下一步是在新电脑建立独立MotorBench_Model，离线验证单电机PWM到RPM动态。
- 当前静止占位模型不会响应PWM，禁止在其中解锁。

## 2. 仓库和提交

旧电脑仓库：F:\ardupilot

- 基础版本：ArduPilot Copter 4.5.7
- 基础分支：Droneer4.5.7
- BFACRO分支：Droneer4.5.7_BFACRO
- 旧机当前分支：Droneer_BFACRO_SITL模型修改
- 远端：origin/Droneer4.5.7_BFACRO
- 19ce66b696：Droneer4.5.7版本硬件定义
- 2336d0e183：DroneerX6硬件引脚/罗盘方向修改
- 24e0d9452c：BFACRO飞行模式
- 3a9c111c04：创建BFACRO_TEST_REPORT.md

tracked工作树干净；只有未追踪的.codex_staging临时教学注释目录。

新电脑：

    git clone <实际远程URL> ~/ardupilot
    cd ~/ardupilot
    git fetch --all --tags
    git checkout Droneer4.5.7_BFACRO
    git submodule update --init --recursive
    git log --oneline -3

应看到24e0d9452c和3a9c111c04。

## 3. BFACRO实现

- 新模式，不覆盖原ACRO。
- 名称BFACRO，四字符BFAC，模式编号29。
- 类ModeBFAcro，主文件ArduCopter/mode_bfacro.cpp。
- 条件开关MODE_BFACRO_ENABLED。
- 第一版不新增BFACRO参数，复用ACRO_RP_RATE、ACRO_RP_EXPO、ACRO_Y_RATE、ACRO_Y_EXPO、ACRO_OPTIONS、ACRO_THR_MID。
- Expo公式：output=input*(1-expo)+input^3*expo。
- norm_input_dz仍使用ArduPilot RC deadzone。
- 目标角速度以centidegrees/s传给input_rate_bf_roll_pitch_yaw_2。
- 纯Rate Loop，不使用原ACRO Trainer/回正逻辑。
- 手动油门使用set_throttle_out(...,false,0.0f)，无倾角补偿和油门输入滤波。
- 已接入AirMode、failsafe、crash/parachute check、GCS模式屏蔽。
- 原ACRO保留并通过回归测试。

修改文件：

    ArduCopter/Copter.h
    ArduCopter/Parameters.cpp
    ArduCopter/Parameters.h
    ArduCopter/RC_Channel.cpp
    ArduCopter/config.h
    ArduCopter/crash_check.cpp
    ArduCopter/events.cpp
    ArduCopter/mode.cpp
    ArduCopter/mode.h
    ArduCopter/mode_bfacro.cpp
    libraries/AP_Vehicle/AP_Vehicle.cpp

完整测试报告在仓库根目录BFACRO_TEST_REPORT.md。

## 4. BFACRO验证结果

编译：

    ./waf configure --board sitl
    ./waf copter
    ./waf configure --board DroneerX6
    ./waf copter

SITL和DroneerX6 C++编译/链接均成功；DroneerX6生成bin/apj/abin/hex。末尾打包曾因Cygwin脚本执行权限报错，后用显式bash/python3完成，不是C++问题。

功能已通过：

- mode 29进入成功，HEARTBEAT custom_mode=29。
- 可解锁、Ground Idle和手动油门正常。
- ACRO_RP_RATE=360时满横滚杆得到6.283185rad/s=360deg/s。
- Expo、正负对称、纯Rate Loop正常。
- ACRO_OPTIONS=1时AirMode正常。
- 地面/飞行油门failsafe正常。
- FLTMODE_GCSBLOCK、RC入口、INITIAL_MODE=29正常。
- 原ACRO正常。

未完成：DroneerX6实机刷写、实机模式切换、低风险悬停、实机AirMode/failsafe。

GCC14/C++20会对旧4.5.7产生大量template-id兼容警告，不是BFACRO特有问题；最终链接成功即可。

## 5. MATLAB脚本版JSON模型

官方目录：

    F:\ardupilot\libraries\SITL\examples\JSON\MATLAB\Copter\

文件职责：

- Copter.m：机体、电机、桨、电池、质量、惯量、阻力参数生成器。
- Hexsoon.mat：参数包。
- SIM_multicopter.m：PWM到电机/桨、力/力矩和刚体状态。
- 上级SITL_connector.m：UDP9002+JSON锁步接口。

旧机中文教学版：

    C:\Users\zhimukeji\Desktop\SITL\Copter\

Copter.m、SIM_multicopter.m、readme.md已加入详细中文注释和高速机提示；Hexsoon.mat未改。换机需复制整个目录。

官方模型局限：

- 推力有人工2.2倍率。
- 固定CT/CQ，无前进比J=Va/(nD)。
- 固定三轴CdA，无迎角/侧滑气动。
- 旋转阻尼0.2为经验值。
- 刚体方程缺omega×(Iomega)。
- DCM一阶积分而非四元数。
- 电池仅简单内阻压降。
- 不能直接用于110m/s定量验证。

## 6. JSON与FRAME参数

JSON替代PWM后的动力学，不替代ArduPilot混控：

    ArduPilot FRAME_CLASS/TYPE：控制指令 -> 电机PWM
    MATLAB/Simulink：PWM -> 力/力矩 -> 姿态/速度/位置

不要只用-f JSON:IP，因为它不匹配vehicleinfo机架项，可能不加载copter.parm并留下FRAME_CLASS=0。

建议：

    sim_vehicle.py -v ArduCopter -f quad --model JSON:<MATLAB主机IP> --console --map

X四旋翼：

    FRAME_CLASS=1
    FRAME_TYPE=1

## 7. Blackbox分析

旧机原始日志：

    C:\Users\zhimukeji\Desktop\入职信息\BF\7.28汕尾R7D测试 黑匣子\

解码器：

    cd /cygdrive/f
    git clone --depth 1 https://github.com/betaflight/blackbox-tools.git
    cd /cygdrive/f/blackbox-tools
    make obj/blackbox_decode

生成F:\blackbox-tools\obj\blackbox_decode.exe。pkg-config警告可忽略。

结果：

- 23个BBL、47段会话。
- 过滤规则：时长>5s并有气压高度、油门、eRPM离地证据。
- 保留23段明确飞行；排除18段≤5s和6段未确认起飞。
- 有效飞行总时长1255.61s，平均54.59s。
- 文件03内4段全无效，已送入旧机Windows回收站。
- 其他包含短会话的文件也包含有效飞行，未删除。

最适合人工查看：BTFL_BLACKBOX_LOG_20260728_11_Droneer405.BBL；单段88.38s、177032帧，数据最多。

动态候选：文件18最强俯仰/横滚；文件21最大偏航；文件14内部第3段电机饱和最高。

日志配置和极值：

- Betaflight4.5.2，STM32F405/Droneer405。
- looptime125us，PID denom2，PID约4kHz，Blackbox约2kHz。
- 双向DShot，日志头显示14极，需实物复核。
- PID：Roll 45/80/40，Pitch 47/84/46，Yaw 45/80/0。
- 实际最大角速度：Roll494、Pitch548、Yaw259deg/s。
- 期望最大：Roll567、Pitch569、Yaw246deg/s。
- eRPM最大1604；按电气RPM/100、14极估算机械RPM约22914。
- 最大单段电机饱和4.55%，平均1.31%。
- 最低电压21.86V。
- 电流峰值反复停在66.02~66.46A，疑似量程/换算封顶。
- 无GPS速度和直接姿态角，不能单独确认110m/s或80度。
- 约50%missing iteration是4kHz控制/2kHz日志抽样，不是50%丢包。

曾生成分析CSV/脚本，但当前旧机仓库根下已看不到blackbox_analysis目录；新机如需CSV，应从BBL重新解码。

## 8. 高速机已知参数

- 桨：7×12英寸，三叶。待补品牌型号、质量、台架曲线。
- 电机：1300KV。待补型号、内阻、空载电流、极数确认。
- 电池：2200mAh、29.6V、150C，即8S；日志最高33.36V吻合。
- ESC：四合一，总电流最大100A。待确认持续/峰值口径、峰值时间、单通道限制。

估算：

- 8S满电理论空载RPM约1300×33.36=43368。
- 日志最大机械RPM约22914，约为空载值53%，需台架复核。
- 150C理论标称330A不可直接当持续能力；ESC总限制和电流采样会先制约。

第一版物理模型最低还需：

1. 含电池/桨/载荷的起飞质量。
2. 质心及四电机轴心相对质心XYZ。
3. Ixx/Iyy/Izz，最好完整惯量矩阵。
4. 电机/桨/ESC/电池完整型号。
5. 电机内阻、空载电流、磁极数。
6. 电池重量、实测内阻、放电曲线。
7. 指令-电压-电流-RPM-推力-转矩台架数据。
8. 正面/侧面/顶面投影面积。
9. 与Blackbox对时的速度-倾角-油门-电流-RPM数据。

## 9. Simulink当前状态

旧机目录：

    C:\Users\zhimukeji\Desktop\SITL\SimulinkCopter\

至少包含：

    AP_receve.m
    AP_send.m
    HighSpeedCopter_Model.slx
    tcp_udp_ip_2.0.6\pnet.m
    tcp_udp_ip_2.0.6\pnet.mexw64

AP_Conector.slx是锁定接口库，不是普通飞机模型。已新建HighSpeedCopter_Model.slx：

    AP receive -> MulticopterDynamics(MATLAB Function) -> AP send

求解器：

    Fixed-step
    ode1(Euler)
    0.0025s
    Stop time inf

pnet路径：

    workDir = 'C:\Users\zhimukeji\Desktop\SITL\SimulinkCopter';
    pnetDir = fullfile(workDir,'tcp_udp_ip_2.0.6');
    cd(workDir);
    addpath(workDir);
    addpath(pnetDir);
    rehash;
    which pnet -all
    pnet('closeall')

通信已经成功：

    Connected to 127.0.0.1:56962
    约400fps
    约100% realtime

静止验证成功。当前MulticopterDynamics输出：

    gyro=[0;0;0] rad/s，body
    attitude=[0;0;0] rad
    accel=[0;0;-9.80665] m/s²，比力
    velocity=[0;0;0] m/s，NED
    position=[0;0;0] m，NED

禁止在此占位模型解锁。

reset含义：AP receive发现SITL帧号回退时拉高一个时间步；模型必须清零RPM、电流、角速度、姿态、速度、位置、电池和所有积分/滤波状态。

## 10. 下一步：MotorBench单电机离线验证

新建独立MotorBench_Model.slx，验证：

    PWM -> throttle -> targetRPM -> dynamicRPM

临时经验MATLAB Function：

    function [rpm, throttle, rpm_target] = MotorModel(pwm)
    %#codegen
    dt=0.0025;
    loadedMaxRPM=23000;
    tau=0.04;
    persistent rpm_state
    if isempty(rpm_state)
        rpm_state=0;
    end
    throttle=(pwm-1100)/800;
    throttle=min(max(throttle,0),1);
    rpm_target=loadedMaxRPM*throttle;
    rpm_dot=(rpm_target-rpm_state)/tau;
    rpm_state=rpm_state+rpm_dot*dt;
    rpm_state=min(max(rpm_state,0),loadedMaxRPM);
    rpm=rpm_state;
    end

测试：

    0~1s PWM1100
    1~2s PWM1300
    2~3s PWM1500
    3~4s PWM1700
    4~5s PWM1900
    5~6s PWM1500
    6~7s PWM1100

临时目标RPM：0、5750、11500、17250、23000。

这只用于验证数值链路，不是最终物理模型。之后替换为电池负载电压、电机反电动势/电阻/电流/转矩、桨惯量/阻力转矩、静推台架数据和高速CT(J)/CQ(J)或二维查表。

后续顺序：

    四电机
    -> 电池压降和总电流限制
    -> 推力/反扭矩
    -> 合力/合力矩
    -> 六自由度
    -> 地面接触
    -> 对比官方.m模型
    -> 对比Blackbox RPM/角速度
    -> 高速阻力和前进比模型

## 11. 新电脑WSL2 + Windows MATLAB 2022b

Windows侧从旧机复制：

    C:\Users\zhimukeji\Desktop\SITL\SimulinkCopter\
    C:\Users\zhimukeji\Desktop\SITL\Copter\

MATLAB：

    workDir='<新电脑实际路径>\SimulinkCopter';
    addpath(workDir);
    addpath(fullfile(workDir,'tcp_udp_ip_2.0.6'));
    rehash;
    which pnet -all
    pnet('closeall')
    open_system(fullfile(workDir,'HighSpeedCopter_Model.slx'))

Windows防火墙允许MATLAB接收UDP9002。

WSL2编译：

    cd ~/ardupilot
    ./waf configure --board sitl
    ./waf copter -j$(nproc)

WSL2与Windows是否可用127.0.0.1取决于网络模式。镜像网络先试127.0.0.1；NAT模式先查看：

    grep nameserver /etc/resolv.conf

启动：

    cd ~/ardupilot/ArduCopter
    ../Tools/autotest/sim_vehicle.py \
        -v ArduCopter \
        -f quad \
        --model JSON:<WINDOWS_HOST_IP> \
        --console --map

确认FRAME_CLASS=1、FRAME_TYPE=1。若连接失败，检查WSL目标IP、Windows UDP9002防火墙、pnet路径和端口占用。

成功标志：Connected、约400fps/100% realtime，MAVProxy status/watch HEARTBEAT/ATTITUDE有数据。

## 12. 已知陷阱

1. AP_Conector.slx是库，要复制AP receive/AP send到普通模型。
2. MATLAB每次启动要添加tcp_udp_ip_2.0.6，最好配置模型InitFcn。
3. 持续400fps锁步说明接收PWM和返回JSON都工作。
4. MAVProxy默认不连续打印遥测；用status、watch HEARTBEAT、watch ATTITUDE。
5. 静止占位模型禁止解锁。
6. Simulink时间步由Simulink设置，AP_receve.m不采用ArduPilot帧率字段。
7. NED向下为正，空中position_D<0，水平静止加速度计比力为[0;0;-g]。
8. 高速模型不能只放大EDU450的Cd或推力倍率，必须台架/实飞辨识。

## 13. 新Codex首轮行动

1. 完整读取本文件。
2. 检查仓库提交24e0d9452c和BFACRO_TEST_REPORT.md。
3. 检查Windows SimulinkCopter目录和HighSpeedCopter_Model.slx。
4. 在WSL2重新做一次静止锁步验证。
5. 不解锁，新建MotorBench_Model，完成PWM阶跃和一阶RPM响应。
6. 保存Scope/导出数据，检查无负RPM、无发散、时间常数合理。
7. 优先从Blackbox辨识真实RPM动态，再迁移四电机和六自由度。

## 14. 2026-08-03补充：MATLAB脚本模型和SITL日志最新结果

### 14.1 已写入脚本模型的已知参数

旧电脑工作目录：

    C:\Users\zhimukeji\Desktop\SITL\Copter\

当前 `Copter.m` 已知/曾写入的参数：

- 整机质量：2 kg（仍需最终实称确认）。
- 电机：1300 KV。
- 螺旋桨：7×12英寸、三叶。
- 电池：8S，满电33.36 V，标称29.6 V，2.2 Ah，标称150C。
- 电池等效内阻暂写0.0034 ohm，只是待校准初值。
- 电机垂直安装，无内倾/外倾；标准对称X机架。

电机编号必须区分Betaflight与ArduPilot：

    Betaflight:
      右后1 顺时针
      右前2 逆时针
      左后3 逆时针
      左前4 顺时针

    ArduPilot:
      右前1 逆时针
      左后2 逆时针
      左前3 顺时针
      右后4 顺时针

不能按数字直接对应，必须按物理位置和旋向映射。

### 14.2 已确认的三向CAD投影面积

ArduPilot机体坐标：

    X：向前
    Y：向右
    Z：向下

结构同事给出并由用户确认：

    沿X观察（正面）：30833.0 mm^2 = 0.0308330 m^2
    沿Y观察（侧面）：30833.0 mm^2 = 0.0308330 m^2
    沿Z观察（顶部）：12868.9 mm^2 = 0.0128689 m^2

飞机顶视轮廓类似子弹头，顶部投影最小；正面与侧面投影面积相同。

第一版面积应写为：

    copter.cd_ref_area = [
        0.0308330;
        0.0308330;
        0.0128689
    ];

旧教学模型三轴面积约0.159 m^2，明显过大。

重要：风阻由 `Cd*A` 决定，面积最小不等于风阻必然最小。流线型效果主要还体现在阻力系数。正向/反向投影面积相同，但 `Cd_forward` 和 `Cd_backward` 可以不同。

### 14.3 迎风方向的正确判断

不能用飞机 `Pitch` 单独判断顶部是否迎风。必须用相对气流在机体系中的方向：

    v_air_ef = state.velocity - wind_velocity;
    v_air_bf = state.dcm' * v_air_ef;
    u = v_air_bf(1);
    v = v_air_bf(2);
    w = v_air_bf(3);
    V = norm(v_air_bf);
    alpha = atan2(w,u);
    beta  = asin(v/V);

近似关系：

    迎角 alpha ≈ 机体俯仰角 theta - 航迹爬升角 gamma

因此，即使姿态为50~80度，如果航迹也以类似角度爬升，气流仍可能主要沿X轴，而不是Z轴。需在日志中同时看 `Pitch`、速度向量和航迹角。

最终建议从固定三轴阻力升级为：

    Cx = f(alpha,beta,Re)
    Cy = f(alpha,beta,Re)
    Cz = f(alpha,beta,Re)
    Cl,Cm,Cn = f(alpha,beta,Re)

或初期直接建立 `CdA(alpha,beta,V)` 查表。阻力系数需由CFD、风洞或实飞辨识得到，不应为了达到110 m/s直接反向调数后当作实机结论。

### 14.4 DataFlash日志结果与纠正

旧电脑日志：

    C:\cygwin64\home\zhimukeji\sim\logs\00000001.BIN

关键统计：

- 最大水平速度：31.85 m/s。
- 最大三维速度：34.36 m/s。
- 最高速度时实际俯仰角：-60.41 deg。
- 当时输入油门约99.1%，输出油门100%。
- 电机最高输出1950 PWM，存在混控饱和。
- 当时垂直速度约-9.64 m/s，不是稳定保高平飞。
- BAT始终12.6 V、0 A，MATLAB电池动态未传入SITL日志。

重要纠正：该日志同时出现Mode 29和Mode 0，用户确认该次最高速度测试是自稳模式，不是BFACRO。不得将31.85 m/s结果描述为BFACRO性能。

日志说明当时是“油门/电机输出饱和+教学模型阻力过大”，不是ArduPilot的20 m/s速度参数限制。

### 14.5 MATLAB电池与ArduPilot电池遥测是两套独立状态

MATLAB脚本当前JSON只返回时间、IMU、位置、姿态和速度，未返回电压/电流。MATLAB中33.36 V只影响内部电机/推力模型；地面站显示由ArduPilot SITL参数决定。

`BFACRO_JSON.parm` 建议包含：

    SIM_BATT_VOLTAGE 33.36
    SIM_BATT_CAP_AH 2.2
    BATT_CAPACITY 2200

这只能修正初始电压/容量显示；要记录实时电压、压降和电流，需扩展MATLAB JSON和ArduPilot `SIM_JSON` 解析。

### 14.6 旧Cygwin启动参考（新机WSL2需换路径/IP）

    cd /cygdrive/f/ardupilot/ArduCopter
    ../Tools/autotest/sim_vehicle.py \
        -v ArduCopter \
        -f quad \
        --model JSON:127.0.0.1 \
        --add-param-file=/home/zhimukeji/sim/BFACRO_JSON.parm \
        -w \
        --console \
        --map

`-w` 必须在反斜杠续行结构内，不能在 `--map` 之后断开；否则不会执行。`-w` 会擦除已保存SITL参数并重新加载默认值/参数文件，只建议在需要可重复初始状态时使用。

### 14.7 新电脑交接后的最优先任务

1. 复制旧电脑的 `Copter`、`SimulinkCopter`、`BFACRO_JSON.parm`、`00000001.BIN` 和必要Blackbox日志。
2. 在WSL2确认ArduPilot分支/提交和子模块完整。
3. 先做MATLAB 2022b与WSL2 SITL静止JSON锁步通信，不解锁。
4. 验证四电机PWM与ArduPilot物理位置/旋向映射。
5. 将三向CAD面积写入新模型，但保留Cd为“待辨识”，不将调到目标速度的Cd冒充真实系数。
6. 在模型内新增 `alpha`、`beta`、`u/v/w`、动压和各轴气动力日志，以确认实际迎风方向。
7. 分开两种目标：“控制功能压力测试模型”与“实机物理预测模型”，后者必须有台架/CFD/实飞数据校准。
