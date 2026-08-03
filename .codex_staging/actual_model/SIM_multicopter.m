clc                         % 清空命令窗口
clearvars                   % 清空工作区变量，避免旧状态影响仿真
close all                   % 关闭已有图窗

% 桌面模型副本使用ArduPilot仓库中的公共JSON连接器和pnet。
addpath(genpath('F:\ardupilot\libraries\SITL\examples\JSON\MATLAB'))

%% 文件作用与数据流
% 本文件是多旋翼实时动力学模型：
% ArduPilot控制器 → PWM输出 → 本文件physics_step → 姿态/IMU/位置/速度 → JSON → ArduPilot。
% Copter.m 负责生成参数，Hexsoon.mat 保存参数，本文件负责每个时间步的动力学计算。
%
% 坐标约定：
% 1) 地球坐标采用 NED：North北、East东、Down地，向下为正；
% 2) 机体系按 X前、Y右、Z下理解；
% 3) 螺旋桨向上的总推力在机体Z轴为负值。
%
% 【高速穿越机改造总提示】
% 当前是教学示例，包含经验推力倍率、固定Cd、简化电机和一阶积分。
% 它适合验证接口和低速控制逻辑，不能直接作为110 m/s模型的最终物理依据。

%% 1. 载入 Copter.m 生成的机体参数
try
    state = load('Hexsoon','copter'); % 只从 Hexsoon.mat 中读取变量 copter，并存入 state.copter
catch
    run('Copter.m')                   % 找不到参数文件时，运行参数生成脚本
    fprintf('找不到 Hexsoon.mat，已运行 Copter.m 生成参数；请重新启动 SIM_multicopter。\n')
    return                            % 本次停止，避免用未完整初始化的状态进入接口
end
% 【高速穿越机改造】若参数文件改名为 HighSpeedCopter.mat，这里的 load 名称也要同步修改。

%% 2. 环境参数
state.environment.density = 1.225; % 空气密度 kg/m³，约为海平面15°C标准值
state.gravity_mss = 9.80665;       % 重力加速度 m/s²
% 【高速穿越机改造】可根据海拔、气温计算空气密度；110 m/s下密度变化会明显影响阻力和桨推力。
% 如需风场，应在计算阻力和螺旋桨来流时使用“机体速度-风速”，而不是直接使用地速。

%% 3. 物理模型最大时间步
max_timestep = 1/200; % 高速/高角速度联合仿真使用约200 Hz物理更新
% SITL_connector 使用锁步调度并给 state.delta_t 赋实际时间步。
% 【高速穿越机改造】高角速度和高速状态建议逐步测试1/200或1/400，
% 同时检查MATLAB算力、实时因子、数值稳定性和UDP帧率，不能只提高频率而不验证。

%% 4. 把初始化函数和单步物理函数交给公共连接器
init_function = @init;              % 仿真开始时调用本文件末尾的 init()
physics_function = @physics_step;   % 每收到一帧PWM后调用 physics_step()
SITL_connector(state,init_function,physics_function,max_timestep); % 启动UDP/JSON锁步循环

% 连接器要求模型至少维护并返回：
% gyro    = [滚转角速度;俯仰角速度;偏航角速度]，rad/s，机体系
% attitude= [滚转角;俯仰角;偏航角]，rad
% accel   = [ax;ay;az]，m/s²，机体系加速度计读数
% velocity= [北向;东向;地向]，m/s，地球NED系
% position= [北向;东向;地向]，m，地球NED系
% state 中也可以保存电机转速、电流、DCM等模型内部状态。

%% 5. 初始化全部动态状态
function state = init(state)
for i = 1:numel(state.copter.motors)
    state.copter.motors(i).rpm = 0;      % 初始电机转速 rpm
    state.copter.motors(i).current = 0;  % 初始电机电流 A
end
state.gyro = [0;0;0];       % 机体系角速度 [p;q;r]，rad/s
state.dcm = diag([1,1,1]);  % 初始方向余弦矩阵为单位阵：机体系与NED系重合
state.attitude = [0;0;0];   % 初始欧拉角 [roll;pitch;yaw]，rad
state.accel = [0;0;0];      % 初始加速度计读数，m/s²，机体系
state.velocity = [0;0;0];   % 初始地速 [VN;VE;VD]，m/s
state.position = [0;0;0];   % 初始位置 [PN;PE;PD]，m；地面PD=0
state.bf_velo = [0;0;0];    % 初始机体系速度，m/s，用于阻力计算
end

%% 6. 单个物理时间步：输入16路PWM，输出更新后的state
function state = physics_step(pwm_in,state)

% 用上一个时间步的各电机电流估计本时间步总电流。
state.copter.battery.current = sum([state.copter.motors.current]);

% 简化电池模型：负载电压 = 开路电压 - 总电流×电池等效内阻。
state.copter.battery.dropped_voltage = state.copter.battery.voltage ...
    - state.copter.battery.resistance * state.copter.battery.current;
% 【高速穿越机改造】需要时加入SOC、动态压降、温度、最大放电电流和最低电压限制。

%% 7. 分别计算每个动力单元的电流、转速、推力和力矩
for i = 1:numel(state.copter.motors)
    motor = state.copter.motors(i); % 取出本电机上一时刻状态及静态参数

    % 把该电机通道PWM线性映射为0~1油门：
    % 1100 μs→0，1900 μs→1，超出范围后限幅。
    throttle = (pwm_in(motor.channel) - 1100) / 800;
    throttle = max(throttle,0);
    throttle = min(throttle,1);
    % 【高速穿越机改造】将1100/1900参数化，并与MOT_PWM_MIN/MAX、DShot输出模型核对；
    % 如果有实测“指令→转速/推力”曲线，应替换当前线性油门模型。

    % 假定ESC等效为降压器：加在电机上的有效电压=油门×电池负载电压。
    voltage = throttle * state.copter.battery.dropped_voltage;

    % 由速度常数Kv换算转矩常数Kt；当前公式假定SI单位关系成立。
    Kt = 1/(motor.electrical.kv * ((2*pi)/60));

    % 由简化直流电机稳态方程反算电流：
    % 电流约等于(理想空载转速-当前转速)/(总电阻×Kv)。
    current = ((motor.electrical.kv * voltage) - motor.rpm) ...
        / ((motor.electrical.resistance + motor.esc.resistance) ...
        * motor.electrical.kv);

    torque = current * Kt; % 电磁转矩 N·m
    % 【高速穿越机改造】必要时限制current>=0及最大电流，并加入再生/制动、温升和ESC动态。

    % 螺旋桨气动阻力矩，采用静态经验公式 Q=Cq*rho*n²*D⁵。
    prop_drag = motor.prop.PConst * state.environment.density ...
        * (motor.rpm/60)^2 * motor.prop.diameter^5;

    w = motor.rpm * ((2*pi)/60); % rpm转换为角速度rad/s

    % 转动方程：角加速度=(电机转矩-桨阻力矩)/桨等效转动惯量，再用显式Euler积分。
    w1 = w + ((torque-prop_drag) / motor.prop.inertia) * state.delta_t;
    rps = w1 * (1/(2*pi)); % rad/s转换成转/秒 rps
    rps = max(rps,0);      % 本简化模型不允许负转速

    % 静态推力公式 T=2.2*CT*rho*n²*D⁴。
    % 2.2 是官方示例为了得到合理悬停油门而加入的经验倍率，并非通用物理常数。
    thrust = 2.2 * motor.prop.TConst * state.environment.density ...
        * rps^2 * motor.prop.diameter^4;
    % 【高速穿越机改造：重点】用台架数据消除2.2经验倍率。
    % 高速前飞需要计算桨轴向来流Va和前进比J=Va/(nD)，再使用CT(J)、CQ(J)查表；
    % 否则模型会在110 m/s时仍按静态空气计算推力，结果不可信。

    % 用推力和电机相对质心的位置计算滚转/俯仰力矩。
    moment_roll = thrust * motor.location(1);
    moment_pitch = thrust * motor.location(2);

    % 电机/桨反扭矩产生偏航力矩，符号由旋转方向决定。
    moment_yaw = -torque * motor.direction;

    % 将本时间步结果写回主状态，供下一时间步和总力/总力矩计算使用。
    state.copter.motors(i).torque = torque;
    state.copter.motors(i).current = current;
    state.copter.motors(i).rpm = rps * 60;
    state.copter.motors(i).thrust = thrust;
    state.copter.motors(i).moment_roll = moment_roll;
    state.copter.motors(i).moment_pitch = moment_pitch;
    state.copter.motors(i).moment_yaw = moment_yaw;
end

%% 8. 机体平动气动阻力
% 三轴分别使用 D=0.5*rho*Cd*A*V²；sign(V)让阻力方向与速度相反。
% bf_velo是机体系速度，因此Cd和参考面积也应对应机体X/Y/Z轴。
drag = sign(state.bf_velo) .* state.copter.cd ...
    .* state.copter.cd_ref_area * 0.5 ...
    .* state.environment.density .* state.bf_velo.^2;
% 【高速穿越机改造：重点】先用实测正面/侧面/顶面CdA；加入风后应使用相对气流速度。
% 若需要覆盖接近80°姿态和大迎角，应进一步按迎角alpha、侧滑角beta计算气动力，
% 而不能只使用三个互不耦合的固定Cd。

%% 9. 合成机体系总力
% 所有桨推力沿机体-Z轴（向上），再减去三轴气动阻力。
force = [0;0;-sum([state.copter.motors.thrust])] - drag;
% 当前不包含机身升力、电机倾角、侧力、地效、桨间干扰和陀螺效应。

%% 10. 旋转气动阻尼
% 使用经验二次阻尼，0.2没有严格物理来源，只用于得到看起来合理的最大角速度。
rotational_drag = 0.2 * sign(state.gyro) .* state.gyro.^2;
% 【高速穿越机改造】至少改成三轴独立系数 [Kp;Kq;Kr]，通过Blackbox角响应辨识；
% 更进一步应让气动力矩随空速、迎角、侧滑角和角速度共同变化。

%% 11. 合成机体系总力矩
% 符号与本示例的电机位置定义、DCM和ArduPilot轴向约定配套。
moments = [-sum([state.copter.motors.moment_roll]); ...
            sum([state.copter.motors.moment_pitch]); ...
            sum([state.copter.motors.moment_yaw])] - rotational_drag;

% 调用六自由度状态更新函数。
state = update_dynamics(state,force,moments);
end

%% 12. 根据合力和合力矩积分得到姿态、速度和位置
function state = update_dynamics(state,force,moments)

% 简化旋转方程：角加速度=I⁻¹M。
rot_accel = (moments' / state.copter.inertia)';
% 【高速穿越机改造：重点】严格刚体方程应包含角速度耦合项：
% omega=state.gyro; I=state.copter.inertia;
% rot_accel = I \ (moments - cross(omega,I*omega));
% 实施前必须用单轴力矩测试确认坐标和符号。

% 显式Euler积分角加速度，得到机体系角速度。
state.gyro = state.gyro + rot_accel * state.delta_t;

% 将角速度限制在±2000 deg/s，近似典型陀螺仪量程。
state.gyro = max(state.gyro,deg2rad(-2000));
state.gyro = min(state.gyro,deg2rad(2000));
% 【高速穿越机改造】按实机IMU量程设置；应区分真实刚体角速度和“传感器饱和后的测量值”。

% 使用当前角速度×时间步更新方向余弦矩阵和欧拉角。
[state.dcm, state.attitude] = rotate_dcm(state.dcm,state.gyro * state.delta_t);
% 【高速穿越机改造】高角速度长时间积分推荐改用归一化四元数，并做时间步收敛测试。

% 牛顿第二定律：机体系运动加速度=机体系合力/质量。
state.accel = force / state.copter.mass;

% 用DCM把机体系运动加速度转换到地球NED系，并在Down轴加入重力。
accel_ef = state.dcm * state.accel;
accel_ef(3) = accel_ef(3) + state.gravity_mss;

% 简化地面接触：位于地面且仍要向下加速时，把Down方向加速度限制为0。
if state.position(3) >= 0 && accel_ef(3) > 0
    accel_ef(3) = 0;
end

% 构造加速度计比力读数：把地球系运动加速度与重力项组合后转回机体系。
state.accel = state.dcm' * (accel_ef + [0; 0; -state.gravity_mss]);

% 显式Euler积分：加速度→速度→位置。
state.velocity = state.velocity + accel_ef * state.delta_t;
state.position = state.position + state.velocity * state.delta_t;
% 【高速穿越机改造】提高频率后做时间步收敛比较；必要时换半隐式Euler或Runge-Kutta积分。

% 防止飞机穿入地面。NED中Down为正，所以position(3)>0代表地下。
if state.position(3) >= 0
    state.position(3) = 0;     % 把位置钳制在地面
    state.velocity = [0;0;0];  % 接地后直接清零全部速度
    state.gyro = [0;0;0];      % 接地后直接清零全部角速度
end
% 这是非常简化的地面模型，不包含起落架弹性、摩擦、反弹和翻滚。

% 把NED地速转换回机体系，供下一时间步计算机体阻力。
state.bf_velo = state.dcm' * state.velocity;
end

%% 13. 用小角度增量更新方向余弦矩阵，并输出欧拉角
function [dcm, euler] = rotate_dcm(dcm, ang)

% 根据本时间步角增量ang=[dRoll;dPitch;dYaw]对DCM做一阶近似更新。
delta = [dcm(1,2) * ang(3) - dcm(1,3) * ang(2), dcm(1,3) * ang(1) - dcm(1,1) * ang(3), dcm(1,1) * ang(2) - dcm(1,2) * ang(1);
         dcm(2,2) * ang(3) - dcm(2,3) * ang(2), dcm(2,3) * ang(1) - dcm(2,1) * ang(3), dcm(2,1) * ang(2) - dcm(2,2) * ang(1);
         dcm(3,2) * ang(3) - dcm(3,3) * ang(2), dcm(3,3) * ang(1) - dcm(3,1) * ang(3), dcm(3,1) * ang(2) - dcm(3,2) * ang(1)];
dcm = dcm + delta;

% 数值积分会破坏DCM各行的正交性，下面用近似Gram-Schmidt方法重新正交化并归一化。
a = dcm(1,:);
b = dcm(2,:);
error = a * b';                  % 第一、第二行不正交的误差
t0 = a - (b *(0.5 * error));     % 对第一行修正一半误差
t1 = b - (a *(0.5 * error));     % 对第二行修正另一半误差
t2 = cross(t0,t1);               % 第三行由前两行叉乘得到
dcm(1,:) = t0 * (1/norm(t0));    % 第一行单位化
dcm(2,:) = t1 * (1/norm(t1));    % 第二行单位化
dcm(3,:) = t2 * (1/norm(t2));    % 第三行单位化

% 从DCM提取欧拉角：[roll;pitch;yaw]，单位rad。
euler = [atan2(dcm(3,2),dcm(3,3)); ...
        -asin(dcm(3,1)); ...
         atan2(dcm(2,1),dcm(1,1))];
% 欧拉角在pitch接近±90°时存在奇异性；DCM本身仍可表达姿态，
% 但高速大机动模型建议内部使用四元数积分，仅在输出/显示时转换欧拉角。
end
