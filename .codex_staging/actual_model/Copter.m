clc                 % 清空命令窗口显示，不影响工作区变量
clear               % 清空当前 MATLAB 工作区，避免旧的 copter/motor 参数污染本次生成
close all           % 关闭已有绘图窗口

%% 文件作用
% 本文件不是实时动力学求解器，而是“机体参数生成脚本”。
% 运行本文件后，会把结构体 copter 保存到 Hexsoon.mat。
% SIM_multicopter.m 启动时读取 Hexsoon.mat，然后使用其中的质量、电机、桨、电池和阻力参数。
%
% 重要：修改本文件后必须重新运行 Copter.m，才能更新 Hexsoon.mat。
% 若只修改代码但不重新运行，联合仿真仍会使用旧参数。
%
% 当前参数大致对应 Hexsoon EDU450：450 mm 轴距、X 型四旋翼。
% 【高速穿越机改造】建议先复制为 HighSpeedCopter.m，并把输出文件改为
% HighSpeedCopter.mat，保留原始示例作为对照基线。

%% 1. 电机几何位置
% motor(i).location = [x, y, z]，单位 m，位置相对于整机质心。
% 机体系按 ArduPilot 常用约定理解：X 向前、Y 向右、Z 向下。
% 这里用角度和 450 mm 轴距计算四个电机的位置，0.5 表示质心到电机约为半轴距。
% 注意：真实机架的“对角轴距/相邻电机距离/机臂长度”不要混淆，应直接测量质心到各电机轴心坐标。
% 当前确认：标准对称X型，四个电机均垂直安装，不存在外倾或内倾。
% 真实轴距尚未提供，因此暂时保留示例的450 mm，只调整电机顺序为APM编号1~4。
motor(1).location = [[sind(45),cosd(45)]*450*0.5,0] * 0.001;  % APM1：右前；对应BF2
motor(2).location = [[sind(225),cosd(225)]*450*0.5,0] * 0.001; % APM2：左后；对应BF3
motor(3).location = [[sind(315),cosd(315)]*450*0.5,0] * 0.001; % APM3：左前；对应BF4
motor(4).location = [[sind(135),cosd(135)]*450*0.5,0] * 0.001; % APM4：右后；对应BF1
% 【高速穿越机改造】替换为实测坐标。若质心不在几何中心，四个坐标不能只按同一个半径计算。
% 如电机存在外倾/内倾，当前模型还不能表达推力方向，需在 SIM_multicopter.m 中把标量推力改成推力向量。

%% 2. ArduPilot 输出通道到物理电机的映射
% motor(i).channel 指定该物理电机读取 pwm_in 的第几路。
% 当前映射匹配 ArduPilot Quad X：
% APM1/PWM1→右前（BF2），APM2/PWM2→左后（BF3），
% APM3/PWM3→左前（BF4），APM4/PWM4→右后（BF1）。
motor(1).channel = 1; % 右前电机读取 ArduPilot 第1路电机输出
motor(2).channel = 2; % 左后电机读取 ArduPilot 第2路电机输出
motor(3).channel = 3; % 左前电机读取 ArduPilot 第3路电机输出
motor(4).channel = 4; % 右后电机读取 ArduPilot 第4路电机输出
% 【高速穿越机改造】必须与实机电机编号、FRAME_TYPE 和旋向一致；映射错误会导致姿态正反馈甚至翻机。

%% 3. 电机/螺旋桨旋转方向
% direction：1 表示顺时针 CW，-1 表示逆时针 CCW。
% 该符号用于 SIM_multicopter.m 计算螺旋桨反扭矩产生的偏航力矩。
% “顺/逆时针”必须约定观察方向；本示例应整体保持一致，不要只凭文字修改单个符号。
motor(1).direction = -1; % APM1右前/BF2：逆时针CCW
motor(2).direction = -1; % APM2左后/BF3：逆时针CCW
motor(3).direction = 1;  % APM3左前/BF4：顺时针CW
motor(4).direction = 1;  % APM4右后/BF1：顺时针CW
% 【高速穿越机改造】根据实机 props-in/props-out 布局修改，并用小偏航指令检查偏航方向。

%% 4. 电机电气参数
electrical.kv = 1300;                % 已知实机参数：1300 KV
electrical.no_load_current = [0.7,10]; % 空载电流测试点：[电流 A, 测试电压 V]
electrical.resistance = 0.115;       % 电机等效绕组电阻，单位 ohm
electrical.poles = 14;               % Blackbox配置：14磁极（7对极）
electrical.logged_max_rpm = 22914;   % 日志最大eRPM=1604换算的机械转速估计值；不参与当前动力学计算
% 【高速穿越机改造】替换为实际电机参数；最好取得 KV、相间电阻、空载电流和电机/桨台架数据。
% 当前实时模型只实际使用了 kv 和 resistance；no_load_current 仅用于下方性能绘图。
% 高功率电机还应考虑电阻随温度变化、磁饱和、ESC限流和最大转速，但当前模型未实现。

%% 5. ESC 参数
esc.resistance = 0.01; % ESC 导通与线路的等效串联电阻，单位 ohm
esc.total_current_limit = 100; % 已知四合一电调总电流最大100 A；当前模型暂未执行限流
% 【高速穿越机改造】可由满油门电压、电流和压降估算；后续可增加电流限制、响应延迟和油门非线性。

%% 6. 螺旋桨参数
prop.diameter = 7 * 0.0254;   % 已知实机参数：7英寸直径，换算为0.1778 m
prop.pitch = 12 * 0.0254;     % 已知实机参数：12英寸螺距，换算为0.3048 m
prop.num_blades = 3;          % 已知实机参数：三叶桨；当前实时动力学尚未直接使用该字段
prop.PConst = 1.13;          % 功率/阻力矩经验系数，用于计算桨的阻力矩
prop.TConst = 1;             % 推力经验系数，用于计算静态推力
prop.mass = 12.5 * 0.001;    % 单片整桨质量，12.5 g 转成 kg；这里只用于估算转动惯量
prop.inertia = (1/12) * prop.mass * prop.diameter^2; % 把桨近似为细杆，计算绕中心的转动惯量 kg·m²
% 【高速穿越机改造】直径、螺距、叶片数必须换成实物；PConst/TConst 最好用台架数据拟合。
% 110 m/s 前飞时不能继续把 TConst 当常数，应在动力学中使用前进比 J=Va/(nD)，
% 建立 CT(J)、CQ(J) 或直接使用“转速+来流速度→推力/转矩”的二维查表。

%% 7. 把公共电机、ESC、螺旋桨参数复制给4个电机
for i = 1:4
    motor(i).electrical = electrical; % 每个电机使用相同电气参数
    motor(i).esc = esc;               % 每个电机使用相同 ESC 参数
    motor(i).prop = prop;             % 每个电机使用相同螺旋桨参数
end
% 若四个动力单元不一致，可在循环后单独覆盖某个 motor(i) 的参数。

%% 8. 电池参数
battery.voltage = 33.36;        % Blackbox日志最高电压约33.36 V；对应8S满电约33.6 V
battery.nominal_voltage = 29.6; % 已知实机参数：8S标称电压29.6 V
battery.resistance = 0.0034;   % 电池包及供电线路等效内阻，单位 ohm
battery.capacity = 2.2;        % 已知实机参数：2200 mAh，即2.2 Ah
battery.c_rating = 150;        % 已知铭牌参数：150 C；不直接等同于可持续真实电流
% 【高速穿越机改造】替换为实际串数、起始电压、容量和内阻。
% 高速大电流下建议后续加入 SOC-开路电压曲线、动态内阻、温度和欠压/限流模型。

%% 9. 整机参数
copter.motors = motor;         % 将四个动力单元写入整机结构体
copter.battery = battery;      % 将电池参数写入整机结构体
copter.frame_class = 1;        % 已确认：Quad
copter.frame_type = 1;         % 已确认：标准X型
copter.motor_tilt_deg = 0;     % 已确认：电机垂直安装，无外倾/内倾
copter.mass = 2;               % 整机起飞质量，单位 kg（包括电池、桨和载荷）

% 当前用“均匀实心球”公式粗略估算惯量，而且三个轴使用相同值。
% 0.45*0.2=0.09 m 被当作等效球半径；这只是示例估算，不代表真实机架。
inertia = (2/5) * copter.mass * (0.45*0.2)^2;
copter.inertia = diag(ones(3,1) * inertia); % 3×3 转动惯量矩阵，单位 kg·m²
% 【高速穿越机改造】优先从CAD质量属性获取 Ixx、Iyy、Izz 和必要时的 Ixy/Ixz/Iyz；
% 也可以通过摆锤实验或实飞角响应辨识。高速细长机通常 Ixx、Iyy、Izz 差异明显。

% 三个机体轴向的阻力系数 Cd：依次对应机体系 X、Y、Z。
copter.cd = [0.5; 0.5; 0.5];

% 三个机体轴向的参考迎风面积 A，单位 m²；示例粗略地给三个方向相同面积。
copter.cd_ref_area = [1; 1; 1] * pi * (0.45*0.5)^2;
% 【高速穿越机改造】至少分别填写 [正面面积; 侧面面积; 顶视面积] 和 [Cd_x;Cd_y;Cd_z]。
% 最大平飞速度主要受 Cd_x*A_x 影响；可用实飞稳态速度/倾角/推力反算等效 CdA。
% 高攻角时固定三轴 Cd 仍不够，应进一步建立随迎角、侧滑角变化的气动力模型。

%% 10. 保存机体参数
save('Hexsoon','copter') % 生成/覆盖 Hexsoon.mat，仅保存变量 copter
% 【高速穿越机改造】建议改为 save('HighSpeedCopter','copter')，
% 并同步修改 SIM_multicopter.m 中 load() 的文件名。

%% 11. 以下代码只用于绘制简化的电机性能曲线，不参与实时联合仿真
% 参考资料：
% http://www.bavaria-direct.co.za/constants/
% http://www.stefanv.com/rcstuff/qf200204.html
% 某些计算器会考虑温升导致电阻增加，但这还需要估计散热和功率损耗；本示例忽略温度。
max_power = 260;                       % 仅用于确定绘图电流上限，单位 W
battery.voltage = battery.voltage * 0.50; % 绘图临时使用半电压；不会改写上面已保存到 mat 的 copter

% 由 KV 估算转矩常数 Kt，单位近似为 N·m/A。
Kt = 1 / (electrical.kv * ((2*pi)/60));

% 从0到“最大功率/当前绘图电压”生成电流序列，步长0.1 A。
amps = 0:0.1:max_power/battery.voltage;
power_in = amps * battery.voltage;     % 输入电功率 P=UI，单位 W

copper_drop = amps * electrical.resistance; % 电机铜阻压降，单位 V
esc_drop = amps * esc.resistance;             % ESC等效电阻压降，单位 V

ideal_voltage = battery.voltage - copper_drop - esc_drop; % 扣除电阻压降后的等效电机电压
power_out = ideal_voltage .* (amps - electrical.no_load_current(1)); % 简化机械输出功率
efficiency = power_out ./ power_in;    % 简化效率；零电流点会产生 NaN，属于绘图现象

torque = Kt * amps;                    % 简化电磁转矩，单位 N·m
rpm = ideal_voltage * electrical.kv;   % 简化转速，单位 rpm

%% 12. 绘制电机特性
figure('name',sprintf('电机特性（电压 %.2f V）',battery.voltage))
subplot(2,2,1)
hold all
title('转速')
plot(amps,rpm)
xlabel('电流 (A)')
ylabel('转速 (rpm)')
xlim([0,amps(end)])

subplot(2,2,2)
hold all
title('转矩')
plot(amps,torque)
xlabel('电流 (A)')
ylabel('转矩 (N·m)')
xlim([0,amps(end)])

subplot(2,2,3)
hold all
title('功率')
plot(amps,power_in)
plot(amps,power_out)
xlabel('电流 (A)')
ylabel('功率 (W)')
ylim([0,inf])
xlim([0,amps(end)])
legend('输入功率','简化输出功率','location','northwest')

subplot(2,2,4)
hold all
title('效率')
plot(amps,efficiency)
xlabel('电流 (A)')
ylabel('效率（比例）')
ylim([0,inf])
xlim([0,amps(end)])
