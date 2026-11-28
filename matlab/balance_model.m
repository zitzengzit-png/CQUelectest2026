function api = balance_model(mode)
%BALANCE_MODEL  GA25-370 两轮平衡车：参数化建模 / 控制器设计 / 数值验证
%
%   运行：  matlab -batch "balance_model"      （R2025b 批处理验证）
%           或在 MATLAB 里直接 F5
%
%   自动调参：api = balance_model('api');  % 返回内部函数句柄
%             api.params / api.derive / api.sim / api.ternary / api.delay_margin
%             （autotune_pid.m 用它复用同一套参数与仿真，避免两套代码）
%
%   参数来源（params() 里逐条标注）：
%     DATASHEET : 迈辰微 GA25-370BMQ V1.0（空载 170rpm、最大扭矩 3.3kg.cm、
%                 减速比 1:35、12V、堵转 1.8A、90g）
%     MEASURED  : 用户实测（轮径 76mm、轮距 170mm、重心轴线上方 40mm、
%                 前后光电排各距轴线 50mm、光电间距 11.5mm、385 线含减速比）
%     CODE      : 工程代码（TIM2 ARR 4199、250/50/10 Hz 三环）
%     ESTIMATE  : 本脚本估计（质量/转动惯量/死区/偏航阻尼），单独标出

% API 模式：只返回内部函数句柄，不做任何计算（供 autotune_pid.m 复用）
if (nargin >= 1) && ischar(mode) && strcmp(mode, 'api')
    api = struct('params', @params, 'derive', @derive, 'sim', @sim_balance, ...
                 'ternary', @ternary, 'delay_margin', @delay_margin);
    return;
end

clc;
fprintf('==============================================================\n');
fprintf(' GA25-370 two-wheel balancer: model / design / verification\n');
fprintf('==============================================================\n');

% 批处理模式不画图。注意：不要用 usejava('desktop') —— R2025b 在 -batch 下
% 调 usejava 会偶发内部断言崩溃（Failed to bind constant term '[M-function usejava]'）。
if exist('batchStartupOptionUsed', 'var')
    make_figs = ~batchStartupOptionUsed;
else
    make_figs = true;
end

%% 1. 12 V 主设计
P  = params(12.0);
D  = derive(P);
fprintf('\n[1] Plant at Vbat = %.1f V\n', P.Vbat);
fprintf('    v_nl = %.3f m/s   tau_stall = %.3f N.m/motor   a_max(no slip) = %.1f m/s^2\n', ...
        D.v_nl, D.tau_stall, D.a_max);
fprintf('    M = %.3f kg   Ieff = %.5f kg.m^2   l_eff = %.3f m   w0 = %.2f rad/s (%.2f Hz)\n', ...
        D.M, D.Ieff, D.l_eff, D.w0, D.w0/2/pi);

[Kp, Kd] = design_pd(D, 2.0, 0.7);
fprintf('\n[2] PD design (k=2.0, zeta=0.70)\n');
fprintf('    Kp = %.1f PWM/rad = %.1f PWM/deg\n', Kp, Kp*pi/180);
fprintf('    Kd = %.1f PWM/(rad/s) = %.2f PWM/(deg/s)\n', Kd, Kd*pi/180);

cl = roots([1, D.A*Kd, D.A*Kp - D.w0^2]);
wn = abs(cl(1)); zeta = -real(cl(1))/wn;
fprintf('    closed-loop poles = %.2f %+.2fj -> wn = %.2f rad/s (%.2f Hz), zeta = %.3f\n', ...
        real(cl(1)), imag(cl(1)), wn, wn/2/pi, zeta);

[w_c, pm_deg, T_margin] = delay_margin(D, Kp, Kd);
fprintf('    gain crossover = %.2f rad/s (%.2f Hz), phase margin = %.1f deg\n', ...
        w_c, w_c/2/pi, pm_deg);
fprintf('    exact delay margin = %.1f ms  (design target: total loop delay = 10 ms)\n', T_margin*1e3);
fprintf('    PM at 10 ms / 20 ms delay = %.1f / %.1f deg\n', ...
        pm_deg - w_c*0.010*180/pi, pm_deg - w_c*0.020*180/pi);

%% 1b. 实测倒下时间的一致性检查
fprintf('\n[1b] Measured free-fall check: t = %.0f ms (5 -> 20 deg, 12 V)\n', P.t_fall_meas*1e3);
w0m   = 2.063/P.t_fall_meas;
Ieffm = P.m_body*9.81*P.l/w0m^2;
Jpm   = Ieffm + (P.m_body*P.l)^2/D.M;
Am    = w0m^2*D.a_max/(P.u_max*9.81);
Kdm   = 2*0.7*2*w0m/Am;
fprintf('    implies: w0 = %.2f rad/s (%.2f Hz), l_eff = %.3f m, Jp = %.4f kg.m^2\n', ...
        w0m, w0m/2/pi, 9.81/w0m^2, Jpm);
fprintf('    geometric: w0 = %.2f (%.2f Hz), l_eff = %.3f m, Jp = %.4f, fall = %.0f ms\n', ...
        D.w0, D.w0/2/pi, D.l_eff, P.Jp, D.fall_t*1e3);
fprintf('    Kd from measured fall = %.1f  vs geometric %.1f PWM/(deg/s)\n', ...
        Kdm*pi/180, Kd*pi/180);
if P.t_fall_meas > 1.5*D.fall_t
    fprintf('    WARNING: measured fall is %.1fx the geometric estimate.\n', P.t_fall_meas/D.fall_t);
    fprintf('    In the default HOLD state the firmware calls SETPWM(0,0) with STBY on\n');
    fprintf('    = short brake, which damps the fall. Redo with STBY low / no battery;\n');
    fprintf('    otherwise this Jp is the braked value, not the control plant.\n');
end

%% 2b. 电机反电动势（droop）与速度前馈
fprintf('\n[2b] Motor back-EMF (droop) and speed feedforward\n');
Ac0 = [0 1 0;
       D.w0^2-D.A*Kp, -D.A*Kd, D.beta;
       D.eps*Kp-D.gamma, D.eps*Kd, -D.delta];
p0  = eig(Ac0);
Ac1 = [0 1 0;
       D.w0^2-D.A*Kp, -D.A*Kd, 0;
       D.eps*Kp-D.gamma, D.eps*Kd, 0];
p1  = eig(Ac1);
fprintf('    extra wheel-speed state: delta = %.1f 1/s (%.0f ms), beta = %.0f, gamma = %.2f\n', ...
        D.delta, 1e3/D.delta, D.beta, D.gamma);
fprintf('    feedforward k_ff = (tau_stall/v_nl)/Kt = %.0f PWM counts per (m/s)\n', D.k_ff);
fprintf('    CL poles WITHOUT feedforward: ');
fprintf('%.2f%+.2fj  ', real(p0), imag(p0)); fprintf('\n');
fprintf('    CL poles WITH    feedforward: ');
fprintf('%.2f%+.2fj  ', real(p1), imag(p1)); fprintf('\n');
fprintf('    -> %s\n', ternary(max(real(p0)) < 0, ...
        'design OK without ff', 'NO-ff design is UNSTABLE: add common += k_ff*v'));

%% 2c. 与固件参数同步（robot_config.h -> 仿真）
Kp_sim = P.kp_pitch*180/pi;        % 固件用 PWM/deg，仿真内部用 PWM/rad
Kd_sim = P.kd_pitch*180/pi;
if Kp_sim <= 0, Kp_sim = Kp; end
if Kd_sim <= 0, Kd_sim = Kd; end
fprintf('\n[2c] Firmware sync (source: prj/elet2026/tasks/robot_config.h)\n');
fprintf('    att  Kp     code %8.1f PWM/deg | design %8.1f | used %8.1f\n', ...
        P.kp_pitch, Kp*pi/180, Kp_sim*pi/180);
fprintf('    att  Kd     code %8.2f PWM/dps | design %8.2f | used %8.2f\n', ...
        P.kd_pitch, Kd*pi/180, Kd_sim*pi/180);
fprintf('    att  ff     code %8.0f PWM/(m/s)\n', P.ff_speed);
fprintf('    speed Kp    code %8.1f deg/(m/s)   pos Kp %5.2f 1/s\n', P.kp_speed, P.kp_pos);
fprintf('    yaw  Kp/Ki  code %8.1f / %6.1f      yaw angle Kp %.1f 1/s\n', ...
        P.kp_yaw, P.ki_yaw, P.kp_angle);
fprintf('    limits      max_pwm=%.0f max_diff=%.0f authority=%.2f deadband=%.0f\n', ...
        P.u_max, P.max_diff, P.authority, P.u_dead);
fprintf('    periods     inner %.0f ms (%.0f Hz) | mid %.0f ms (%.0f Hz) | out %.0f ms (%.0f Hz)\n', ...
        P.T_inner*1e3, 1/P.T_inner, P.T_mid*1e3, 1/P.T_mid, P.T_out*1e3, 1/P.T_out);
fprintf('    delays      Td_imu %.0f ms | Td_enc %.0f ms | comp %.0f ms | enc LSB %.2f mm/s\n', ...
        P.Td_imu*1e3, P.Td_enc*1e3, P.t_delay_comp*1e3, P.enc_lsb*1000);
if P.kp_pitch <= 0
    fprintf('    WARNING: firmware gains are 0 -> falling back to design values\n');
end

fprintf('\n[3] Max stable Kp vs loop delay (Kd fixed at design value)\n');
fprintf('    delay[ms]   max Kp[PWM/deg]   Kp_design/limit\n');
for T = [0 5 10 20 30 50]
    Kp_max = max_kp_for_delay(D, Kd, T/1000);
    fprintf('    %8.0f   %14.1f   %16.2f\n', T, Kp_max*pi/180, Kp/Kp_max);
end

%% 4. 电池电压 / 转动惯量 敏感性
fprintf('\n[4] Sensitivity: battery voltage and body inertia\n');
fprintf('    Vbat  v_nl   a_max   w0[Hz]  Kp[deg]  Kd[dps]  Tdelay[ms]\n');
for V = [7.4 8.4 11.1 12.0]
    Pv = params(V); Dv = derive(Pv);
    [kpv, kdv] = design_pd(Dv, 2.0, 0.7);
    [~, ~, Tmv] = delay_margin(Dv, kpv, kdv);
    fprintf('    %4.1f  %.3f  %5.1f  %6.2f  %7.1f  %7.2f  %9.1f\n', ...
            V, Dv.v_nl, Dv.a_max, Dv.w0/2/pi, kpv*pi/180, kdv*pi/180, Tmv*1e3);
end
fprintf('    (Jp is an ESTIMATE; the fall test 5deg->20deg pins it: t = 2.063/w0)\n');
for Jp = [0.0015 0.0030 0.0045]
    Pj = params(12.0); Pj.Jp = Jp; Dj = derive(Pj);
    [kpj, kdj] = design_pd(Dj, 2.0, 0.7);
    [~, ~, Tmj] = delay_margin(Dj, kpj, kdj);
    fprintf('    Jp=%.4f -> l_eff=%.3f m w0=%.2f Hz fall=%.0f ms Kp=%.0f Kd=%.1f Tdelay=%.1f ms\n', ...
            Jp, Dj.l_eff, Dj.w0/2/pi, 2.063/Dj.w0*1e3, kpj*pi/180, kdj*pi/180, Tmj*1e3);
end


%% 5. 时间域仿真（严格按固件频率：内环 250 Hz / 中环 50 Hz / 外环 10 Hz）
Td0 = P.Td_imu;                   % 固件/实测总延迟
fprintf('\n[5] Time-domain simulation (fw rates %.0f/%.0f/%.0f Hz, ZOH, u_dead=%.0f, dead_comp=%d, Td=%.0f ms)\n', ...
        1/P.T_inner, 1/P.T_mid, 1/P.T_out, P.u_dead, P.dead_comp, Td0*1e3);
for Td = [0 0.005 Td0]
    R = sim_balance(P,D,Kp_sim,Kd_sim,Td,5.0,0,'position',P.kp_speed,P.kp_pos,0.0,3.0);
    fprintf('    release 5deg, delay %4.0f ms: max|tilt|=%.2f deg max|x|=%.1f cm x@3s=%.1f cm peak|u|=%.0f\n', ...
            Td*1e3, R.max_tilt, R.max_x*100, R.x_3s*100, R.peak_u);
end
R = sim_balance(P,D,Kp_sim,Kd_sim,Td0,5.0,0,'position',P.kp_speed,P.kp_pos,0.0,3.0,true);
fprintf('    release 5deg, delay %2.0f ms + pitch prediction: max|tilt|=%.2f deg max|x|=%.1f cm x@3s=%.1f cm\n', ...
        Td0*1e3, R.max_tilt, R.max_x*100, R.x_3s*100);
for Td = [0 Td0]
    R = sim_balance(P,D,Kp_sim,Kd_sim,Td,0.0,0.30,'position',P.kp_speed,P.kp_pos,0.0,4.0);
    fprintf('    push 0.30 m/s, delay %4.0f ms: max|x|=%.1f cm x@4s=%.1f cm peak|u|=%.0f\n', ...
            Td*1e3, R.max_x*100, R.x_end*100, R.peak_u);
end
for kps = [8 12 19.5]
    R = sim_balance(P,D,Kp_sim,Kd_sim,Td0,0.0,0,'position',kps,P.kp_pos,0.30,4.0);
    fprintf('    position step 0.30 m (kp_speed=%4.1f, delay %2.0f ms): rise=%.2f s overshoot=%.1f%% err@4s=%.2f cm\n', ...
            kps, Td0*1e3, R.rise, R.overshoot, (R.x_end-0.30)*100);
end
for kps = [8 19.5]
    R = sim_balance(P,D,Kp_sim,Kd_sim,Td0,0.0,0,'speed',kps,P.kp_pos,0.40,3.0);
    fprintf('    speed step 0->0.40 m/s (SPEED mode, kp_speed=%4.1f): rise=%.2f s overshoot=%.1f%% v_end=%.3f m/s\n', ...
            kps, R.rise, R.overshoot, R.v_end);
end

%% 5b. 死区/摩擦补偿对比（位置保持处的极限环 + 零交叉抖动）
fprintf('\n[5b] Friction compensation (u_dead=%.0f PWM, eps=%.0f, pos step 0.30 m, Td=%.0f ms)\n', ...
        P.u_dead, P.dead_eps, Td0*1e3);
fprintf('    plant  comp  shape   max|tilt|  pitch p-p    v p-p   |du|/s    err@4s\n');
cfgs = {'zone',0,'-'; 'fric',0,'-'; 'fric',1,'sign'; 'fric',1,'tanh'};
for q = 1:size(cfgs,1)
    Pc = P; Pc.plant = cfgs{q,1}; Pc.dead_comp = cfgs{q,2};
    Pc.dead_smooth = strcmp(cfgs{q,3},'tanh');
    R = sim_balance(Pc,D,Kp_sim,Kd_sim,Td0,0.0,0,'position',P.kp_speed,P.kp_pos,0.30,4.0);
    i1 = find(R.t >= 3.0, 1);
    if isempty(i1), i1 = 1; end
    pp_th = max(R.th(i1:end)) - min(R.th(i1:end));
    pp_v  = (max(R.v(i1:end)) - min(R.v(i1:end)))*1000;
    tv_u  = sum(abs(diff(R.u(i1:end))))/max(R.t(end)-R.t(i1),1e-9);   % 输出总变差/s = 抖动
    fprintf('    %-6s %4d  %-6s %7.2f deg %10.2f deg %6.0f mm/s %8.0f %7.2f cm\n', ...
            cfgs{q,1}, cfgs{q,2}, cfgs{q,3}, R.max_tilt, pp_th, pp_v, tv_u, (R.x_end-0.30)*100);
end
fprintf('    eps sweep (fric + tanh, %.0f PWM deadband):\n', P.u_dead);
fprintf('    eps[PWM]  pitch p-p    v p-p   |du|/s    err@4s\n');
for e = [5 10 20 40]
    Pc = P; Pc.dead_eps = e; Pc.dead_smooth = true;
    R = sim_balance(Pc,D,Kp_sim,Kd_sim,Td0,0.0,0,'position',P.kp_speed,P.kp_pos,0.30,4.0);
    i1 = find(R.t >= 3.0, 1); if isempty(i1), i1 = 1; end
    pp_th = max(R.th(i1:end)) - min(R.th(i1:end));
    pp_v  = (max(R.v(i1:end)) - min(R.v(i1:end)))*1000;
    tv_u  = sum(abs(diff(R.u(i1:end))))/max(R.t(end)-R.t(i1),1e-9);
    fprintf('    %7.0f  %10.2f deg %6.0f mm/s %8.0f %7.2f cm\n', ...
            e, pp_th, pp_v, tv_u, (R.x_end-0.30)*100);
end

%% 6. 偏航环：一阶对象 + PI
fprintf('\n[6] Yaw rate loop (first-order plant; tau_yaw UNKNOWN -> 3 cases)\n');
fprintf('    tau[ms]  kp_yaw[PWM/(deg/s)]  ki_yaw[PWM/deg]  PM[deg]  Tdelay[ms]\n');
for ty = [0.05 0.08 0.15]
    Y = design_yaw(D, ty, 2*pi*4);
    fprintf('    %6.0f  %21.1f  %16.1f  %7.1f  %10.1f\n', ...
            ty*1e3, Y.kp_deg, Y.ki_deg, Y.pm, Y.Tm*1e3);
end
Y = design_yaw(D, 0.08, 2*pi*4);
if P.kp_yaw > 0 && P.ki_yaw > 0            % 优先用固件里的偏航增益
    Y.kp = P.kp_yaw*180/pi; Y.ki = P.ki_yaw*180/pi;
    Y.kp_deg = P.kp_yaw;   Y.ki_deg = P.ki_yaw;
end
R = sim_yaw(D, Y, 0.08, 45.0, 2.0, Td0);
fprintf('    step 45 deg/s (tau=80ms, Td=%.0fms): rise=%.0f ms overshoot=%.1f%% steady=%.1f deg/s\n', ...
        Td0*1e3, R.rise*1e3, R.overshoot, R.w_end);

%% 7. 弧线需求 vs 差模权限
fprintf('\n[7] Arc R=0.40 m at v=0.40 m/s\n');
dv = 0.40 * P.track / 0.40;
diff_need = dv*P.u_max/(2*D.v_nl);
fprintf('    outer/inner wheel = %.3f / %.3f m/s (v_nl=%.3f)\n', ...
        0.40+dv/2, 0.40-dv/2, D.v_nl);
fprintf('    diff needed = %.0f PWM | integral authority 0.3*%.0f = %.0f | max_diff=%.0f\n', ...
        diff_need, P.max_diff, 0.3*P.max_diff, P.max_diff);
fprintf('    outer wheel usage = %.0f%% of no-load -> %s\n', ...
        (0.40+dv/2)/D.v_nl*100, ternary((0.40+dv/2) < 0.85*D.v_nl, 'OK', 'TOO FAST'));

%% 8. 汇总
fprintf('\n[8] Verdict\n');
checks = {
  'zeta in [0.6,0.8]',          zeta > 0.6 && zeta < 0.8;
  'PM >= 45 deg (no delay)',    pm_deg >= 45;
  'PM at 10 ms delay >= 40 deg',(pm_deg - w_c*0.010*180/pi) >= 40;
  'arc outer < 85% v_nl',       (0.40+dv/2) < 0.85*D.v_nl;
};
for i = 1:size(checks,1)
    fprintf('    %-28s %s\n', checks{i,1}, ternary(checks{i,2}, 'PASS', 'FAIL'));
end

%% 9. 姿态环单独工作 + 环境干扰（只跑内环；与 HOLD/POSITION 对比）
fprintf('\n[9] Attitude-only stand with disturbances (inner loop only, Td=%.0f ms)\n', Td0*1e3);
rng(1);
noise = [0.15*pi/180, 1.0*pi/180];        % 0.15 deg / 1.0 deg/s rms sensor noise
d1 = struct('push', [0.5 0.65 1.5], 'noise', noise);   % 1.5 N for 150 ms
d2 = struct('push', [0.5 0.65 3.0], 'noise', noise);   % 3.0 N for 150 ms
d3 = struct('bias', 0.4,          'noise', noise);     % steady 0.4 N (wind/slope)
cases = {'push 1.5 N / 0.15 s', d1;
         'push 3.0 N / 0.15 s', d2;
         'steady 0.4 N wind  ', d3};
fprintf('    %-22s %-9s %9s %8s %8s %9s\n', ...
        'disturbance', 'mode', 'max tilt', 'settle', 'peakPWM', 'drift');
R_att = []; R_attpos = [];
for q = 1:size(cases,1)
  for md = 1:2
    if md == 1, mode = 'att'; else, mode = 'position'; end
    R = sim_balance(P,D,Kp_sim,Kd_sim,Td0,0,0,mode,P.kp_speed,P.kp_pos,0,4.0,true,cases{q,2});
    fprintf('    %-22s %-9s %7.2fdeg %7.2fs %8.0f %8.0fcm\n', ...
            cases{q,1}, mode, R.max_tilt, R.settle, R.peak_u, R.x_end*100);
    if q == 1 && md == 1, R_att = R; end
    if q == 1 && md == 2, R_attpos = R; end
  end
end

%% 10. 要求 1：原地站立（HOLD = POSITION(0,0)）—— 先给激励，再回站立
%  判据：3 s 内 |x| ≤ 15 cm（赛题要求 1），且不倾倒（|pitch| < 20°）
fprintf('\n[10] Requirement 1: stand still (HOLD = POSITION 0,0), 3 s window, limit 15 cm\n');
fprintf('     case                  max|pitch|   max|x|    x@3s   back<2cm  verdict\n');
rng(7);
stand = {
  'release 3 deg tilt',  3.0, 0.00, [];
  'release 5 deg tilt',  5.0, 0.00, [];
  'push   0.20 m/s    ', 0.0, 0.20, [];
  'push   1.5N/0.15s  ', 0.0, 0.00, struct('push',[0.0 0.15 1.5],'noise',noise);
  'push   3.0N/0.15s  ', 0.0, 0.00, struct('push',[0.0 0.15 3.0],'noise',noise);
};
R_stand = [];
for q = 1:size(stand,1)
    R = sim_balance(P,D,Kp_sim,Kd_sim,Td0, stand{q,2}, stand{q,3}, 'position', ...
                    P.kp_speed, P.kp_pos, 0.0, 3.0, true, stand{q,4});
    ix = find(abs(R.x) > 0.02, 1, 'last');
    if isempty(ix), tb = 0; else, tb = R.t(ix); end
    ok = (R.max_x <= 0.15) && (R.max_tilt < 20);
    fprintf('     %-20s %8.2f deg %6.1f cm %6.1f cm %8.2f s   %s\n', ...
            stand{q,1}, R.max_tilt, R.max_x*100, R.x_end*100, tb, ternary(ok,'PASS','FAIL'));
    if q == 4, R_stand = R; end
end
fprintf('     (limit 15 cm; "back<2cm" = last time |x| > 2 cm)\n');

%% 10b. 恒定扰动（坡/风）下的静差 + 位置积分（设计研究）
fprintf('\n[10b] Constant disturbance (0.4 N ~ 2 deg slope) and position integral\n');
fprintf('     kp_pos  ki_pos | 0.4N: x@6s   | 0.30m step: rise  overshoot  err@6s\n');
for kpp = [0.7 1.2]
    for ki = [0 0.2 0.5]
        Pk = P; Pk.ki_pos = ki;
        Rb = sim_balance(Pk,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, kpp, ...
                         0.0, 6.0, true, struct('bias',0.4,'noise',noise));
        Rs = sim_balance(Pk,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, kpp, ...
                         0.30, 6.0, true, []);
        fprintf('     %6.2f  %6.2f | %7.1f cm   | %6.2f s %8.1f%% %8.2f cm\n', ...
                kpp, ki, Rb.x_end*100, Rs.rise, Rs.overshoot, (Rs.x_end-0.30)*100);
    end
end

fprintf('     kp_pos sweep (1.5 N push | 0.30 m step):\n');
fprintf('     kp_pos  max|x|  back<2cm   step rise  overshoot   err@4s\n');
for kpp = [0.7 1.0 1.5 2.0]
    Rp = sim_balance(P,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, kpp, ...
                     0.0, 3.0, true, struct('push',[0.0 0.15 1.5],'noise',noise));
    ix = find(abs(Rp.x) > 0.02, 1, 'last');
    if isempty(ix), tb = 0; else, tb = Rp.t(ix); end
    Rs = sim_balance(P,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, kpp, 0.30, 4.0);
    fprintf('     %6.2f  %5.1f cm %9.2f s %8.2f s %9.1f%% %8.2f cm\n', ...
            kpp, Rp.max_x*100, tb, Rs.rise, Rs.overshoot, (Rs.x_end-0.30)*100);
end
fprintf('     max_pitch sweep (PID_SPEED_MAX_PITCH_DEG, 1.5 N push, kp_pos=%.1f):\n', P.kp_pos);
fprintf('     max_pitch  max|x|   max|pitch|  back<2cm   peakPWM\n');
for mp = [10 15 20]
    Pm = P; Pm.max_pitch = mp;
    Rp = sim_balance(Pm,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, P.kp_pos, ...
                     0.0, 3.0, true, struct('push',[0.0 0.15 1.5],'noise',noise));
    ix = find(abs(Rp.x) > 0.02, 1, 'last');
    if isempty(ix), tb = 0; else, tb = Rp.t(ix); end
    fprintf('     %8.0f  %6.1f cm %9.2f deg %8.2f s %8.0f\n', ...
            mp, Rp.max_x*100, Rp.max_tilt, tb, Rp.peak_u);
end

%% 10c. 速度环积分（PI 速度环）—— 消除坡/风下的静差，且不牺牲阶跃
fprintf('\n[10c] Speed-loop integral (learns the lean needed on a slope)\n');
fprintf('     kp_pos  ki_speed | 0.4N x@6s | step: rise overshoot err@6s | 1.5N push max|x|\n');
for kpp = [0.7 1.2]
    for ki = [0 1 2 4]
        Pk = P; Pk.ki_speed = ki;
        Rb = sim_balance(Pk,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, kpp, ...
                         0.0, 6.0, true, struct('bias',0.4,'noise',noise));
        Rs = sim_balance(Pk,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, kpp, ...
                         0.30, 6.0, true, []);
        Rp = sim_balance(Pk,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, kpp, ...
                         0.0, 3.0, true, struct('push',[0.0 0.15 1.5],'noise',noise));
        fprintf('     %6.2f  %8.1f | %8.1f cm | %6.2f s %8.1f%% %6.2f cm | %10.1f cm\n', ...
                kpp, ki, Rb.x_end*100, Rs.rise, Rs.overshoot, (Rs.x_end-0.30)*100, Rp.max_x*100);
    end
end
fprintf('     refinement (kp_pos=1.2, longer bias run):\n');
fprintf('     ki_speed | 0.4N: x@12s  max|pitch| | step overshoot  err@6s\n');
for ki = [4 8 16]
    Pk = P; Pk.ki_speed = ki;
    Rb = sim_balance(Pk,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, 1.2, ...
                     0.0, 12.0, true, struct('bias',0.4,'noise',noise));
    Rs = sim_balance(Pk,D,Kp_sim,Kd_sim,Td0, 0.0, 0.0, 'position', P.kp_speed, 1.2, ...
                     0.30, 6.0, true, []);
    fprintf('     %8.1f | %9.1f cm %9.2f deg | %9.1f%% %6.2f cm\n', ...
            ki, Rb.x_end*100, Rb.max_tilt, Rs.overshoot, (Rs.x_end-0.30)*100);
end

%% 11. 保存一份仿真快照给 vis_balance.m / vis_attitude.m / vis_stand.m
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
logdir = fullfile(here, 'logs');
if ~exist(logdir, 'dir'), mkdir(logdir); end
S = struct('P', P, 'D', D, 'Kp', Kp, 'Kd', Kd);
S.sim     = sim_balance(P, D, Kp_sim, Kd_sim, P.Td_imu, 0.0, 0, 'speed', P.kp_speed, P.kp_pos, 0.40, 2.5, true);
S.sim_pos = sim_balance(P, D, Kp_sim, Kd_sim, P.Td_imu, 0.0, 0, 'position', P.kp_speed, P.kp_pos, 0.25, 4.0, true);
S.sim_att = R_att;
S.sim_att_pos = R_attpos;
S.dist1 = d1;
S.stand = R_stand;
save(fullfile(logdir, 'balance_model.mat'), '-struct', 'S');
fprintf('\n[11] snapshot saved: matlab/logs/balance_model.mat\n');

if make_figs
    figs(P, D, Kp, Kd, pm_deg, T_margin);
end
fprintf('\nDONE\n');
end

%% ======================= local functions =======================
function P = params(V)
C = read_robot_config();                % 参数唯一来源：prj/elet2026/tasks/robot_config.h
P.Vbat   = V;                          % [DATASHEET] 12 V rated (working 3-13 V)
P.Vrated = 12.0;                       % [DATASHEET]
P.rpm_nl = 170;                        % [DATASHEET] no-load speed at output, rpm
P.tau_max_kgcm = 3.3;                  % [DATASHEET] max torque at output
P.R      = C.WHEEL_DIAMETER_M/2;       % [CODE/MEASURED] 76 mm wheel
P.track  = C.WHEEL_TRACK_M;            % [CODE/MEASURED] 170 mm
P.l      = C.CG_HEIGHT_ABOVE_AXLE_M;   % [CODE/MEASURED] 40 mm
P.t_fall_meas = 0.200;                 % [MEASURED, 按 200 ms 计] 5deg->20deg 自由倒下
% （先前的 700 ms 判断为 HOLD 短路刹车下的值，不用于控制对象）
% mass estimate (no scale yet): motors 2x90 g + plates 110 g + standoffs 40 g
% + battery 100 g + F407/OLED 50 g + 16 sensors/wiring 50 g  ~= 0.62 kg
P.m_body = 0.62;                       % [ESTIMATE] chassis without wheels
P.m_wheel= 0.10;                       % [ESTIMATE] two 76 mm rubber wheels, ~50 g each
P.Jp     = 0.0031;                     % [MEASURED] 由 200 ms 倒下反推（Jp = Ieff + (m l)^2/M）
P.u_dead = C.MOTOR_DEADBAND_PWM;       % [TYPICAL] 4% of full scale (measure!)
P.dead_comp = C.MOTOR_DEADBAND_COMP_ENABLE;  % 固件的死区补偿开关
P.dead_eps  = C.MOTOR_DEADBAND_SMOOTH_EPS;   % 固件的 tanh 平滑区宽度（PWM 计数）
P.dead_smooth = 1;                     % 1 = tanh（与固件一致）/ 0 = 硬 sign（对比用）
P.plant  = 'fric';                     % 'fric' = 静摩擦对象（默认）/ 'zone' = 旧执行器死区模型
P.v_eps  = 0.010;                      % 静摩擦平滑速度（m/s）：Coulomb 项的 tanh 带宽
% ---- 环频率：直接取固件 ----
P.T_inner = C.PID_TASK_PERIOD_MS/1000; % 4 ms  -> 250 Hz（内环）
P.T_mid   = 1/C.PID_MIDDLE_LOOP_HZ;    % 20 ms -> 50 Hz（中环/编码器）
P.T_out   = 1/C.PID_OUTER_LOOP_HZ;     % 100 ms -> 10 Hz（外环）
% ---- 各环增益/限幅：直接取固件 ----
P.u_max    = C.PID_ATT_MAX_PWM;        % 4199
P.max_diff = C.PID_ATT_MAX_DIFF_PWM;   % 2100
P.kp_pitch = C.PID_ATT_KP_PITCH;       % PWM/deg
P.kd_pitch = C.PID_ATT_KD_PITCH;       % PWM/(deg/s)
P.ff_speed = C.PID_ATT_FF_SPEED;       % PWM per (m/s)
P.kp_yaw   = C.PID_YAW_KP;             % PWM/(deg/s)
P.ki_yaw   = C.PID_YAW_KI;             % PWM/deg
P.kp_angle = C.PID_YAW_ANGLE_KP;       % 1/s
P.kp_speed = C.PID_SPEED_KP;           % deg per (m/s)
P.kp_pos   = C.PID_POS_KP_DISTANCE;    % 1/s
P.ki_pos   = 0.0;                      % 位置积分（设计研究用，固件未实现；拟新增 PID_POS_KI_DISTANCE）
P.ki_speed = 0.0;                      % 速度积分 deg/(m·s)（设计研究用，固件未实现；拟新增 PID_SPEED_KI）
P.max_pitch= C.PID_SPEED_MAX_PITCH_DEG; % 中环目标倾角限幅（deg）
P.authority= C.PID_YAW_INTEGRAL_AUTHORITY;
P.t_delay_comp = C.PID_DELAY_COMP_S;
% ---- 延迟（用户给的分解：IMU 2 + 传输/运算 6 + 编码器 2 = 10 ms）----
P.Td_imu = 0.010;
P.Td_enc = 0.010;
% ---- 50 Hz 编码器速度量化 LSB ----
P.enc_lsb = pi*C.WHEEL_DIAMETER_M/C.ENCODER_COUNTS_PER_REV / P.T_mid;
end

function D = derive(P)
scale = P.Vbat/P.Vrated;
D.v_nl    = P.rpm_nl/60*2*pi*P.R*scale;
D.tau_stall = P.tau_max_kgcm*0.0980665*scale;
D.Kt_count  = D.tau_stall/P.u_max;
D.Jw      = 2*0.5*(P.m_wheel/2)*P.R^2;
D.M       = P.m_body + P.m_wheel + D.Jw/P.R^2;
D.Ieff    = P.Jp - (P.m_body*P.l)^2/D.M;
D.w0      = sqrt(P.m_body*9.81*P.l/D.Ieff);
D.l_eff   = 9.81/D.w0^2;
D.A       = (P.m_body*P.l/(D.M*P.R*D.Ieff))*(2*D.Kt_count);
D.a_max   = 2*D.tau_stall/(P.R*D.M);
D.fall_t  = 2.063/D.w0;                % 5deg -> 20deg free fall
D.track   = P.track;
% motor back-EMF ("droop"): force = (2/R)(Kt*u - c*v), c = tau_stall/v_nl
D.c       = D.tau_stall/D.v_nl;        % N.m per (m/s)
D.b       = 2*D.c/P.R;                 % N per (m/s)
D.k_ff    = D.c/D.Kt_count;            % feedforward PWM counts per (m/s)
det       = D.M*P.Jp - (P.m_body*P.l)^2;
D.beta    = (P.m_body*P.l)*D.b/det;    % d(theta_dd)/dv
D.gamma   = (P.m_body*P.l)^2*9.81/det; % -d(v_dot)/d(theta)
D.delta   = P.Jp*D.b/det;              % -d(v_dot)/dv
D.eps     = P.Jp*(2*D.Kt_count/P.R)/det;
end

function [Kp,Kd] = design_pd(D,k,zeta)
Kp = (k^2+1)*D.w0^2/D.A;
Kd = 2*zeta*k*D.w0/D.A;
end

function [wc,pm_deg,Tm] = delay_margin(D,Kp,Kd)
% exact: on the jw axis, A*sqrt(Kp^2+(Kd w)^2) = w^2+w0^2 ; Tm = atan2(Kd wc,Kp)/wc
f  = @(w) D.A*sqrt(Kp^2+(Kd*w).^2) - (w.^2 + D.w0^2);
wc = fzero(f, [1e-3, 1e3*max(1,D.w0)]);
pm_deg = atan2(Kd*wc,Kp)*180/pi;
Tm     = atan2(Kd*wc,Kp)/wc;
end

function Kp_max = max_kp_for_delay(D,Kd,T)
% bisection: find Kp such that delay_margin == T
if T <= 0, Kp_max = Inf; return; end
lo = (D.w0^2/D.A)*1.001;               % stability floor (k=1)
hi = lo*1e4;
for it = 1:40
  mid = sqrt(lo*hi);
  [~,~,Tm] = delay_margin(D,mid,Kd);
  if Tm > T, lo = mid; else, hi = mid; end
end
Kp_max = sqrt(lo*hi);
end

function y = ternary(c,a,b)
if c, y = a; else, y = b; end
end


function R = sim_balance(P,D,Kp,Kd,Td,th0_deg,xd0,mode,kp_speed,kp_pos,ref,Tsim,use_pred,dist)
if nargin < 13, use_pred = false; end
if nargin < 14, dist = []; end
% 严格按固件频率：内环 250 Hz、中环/编码器 50 Hz、外环 10 Hz；控制量零阶保持
dt = 0.001; n = round(Tsim/dt); nd = max(1,round(Td/dt));
n_in  = max(1,round(P.T_inner/dt));    % 4   -> 250 Hz
n_mid = max(1,round(P.T_mid/dt));      % 20  -> 50 Hz
n_out = max(1,round(P.T_out/dt));      % 100 -> 10 Hz
th = th0_deg*pi/180; thd = 0; x = 0; xd = xd0;
hth = th*ones(1,nd+1); hthd = thd*ones(1,nd+1);
th_ref = 0; v_ref = 0; v_meas = 0; u_hold = 0; i_pos = 0; i_v = 0;
pos_mode = strcmp(mode,'position'); spd_mode = strcmp(mode,'speed') || pos_mode;
m=P.m_body; l=P.l; Jp=P.Jp; Rr=P.R; M=D.M; g=9.81;
rec_t=zeros(n,1); rec_th=zeros(n,1); rec_v=xd0*ones(n,1); rec_x=zeros(n,1); rec_u=zeros(n,1);
rec_F=zeros(n,1);
peak_u = 0;
for i = 1:n
  t = (i-1)*dt;
  % ---- external disturbance: horizontal force at the CoG (N) ----
  Fd = 0;
  if ~isempty(dist)
     if isfield(dist,'push') && ~isempty(dist.push)
        for q = 1:size(dist.push,1)
           if (t >= dist.push(q,1)) && (t < dist.push(q,2))
              Fd = Fd + dist.push(q,3);
           end
        end
     end
     if isfield(dist,'bias'), Fd = Fd + dist.bias; end
  end
  % ---- 10 Hz outer loop ----
  if pos_mode && mod(i-1,n_out)==0
     e_pos = ref - x;
     if isfield(P,'ki_pos') && (P.ki_pos > 0)
        % 位置积分（设计研究用；固件尚未实现）。抗饱和：积分贡献限到速度上限
        i_pos  = i_pos + e_pos*P.T_out;
        i_term = max(min(P.ki_pos*i_pos, 0.5), -0.5);
        i_pos  = i_term/P.ki_pos;
        v_ref  = kp_pos*e_pos + i_term;
     else
        v_ref = kp_pos*e_pos;
     end
     v_ref = max(min(v_ref,0.5),-0.5);
  elseif strcmp(mode,'speed')
     v_ref = ref;
  end
  % ---- 50 Hz encoder + speed loop: 20 ms window ending 2 ms ago, quantised ----
  if mod(i-1,n_mid)==0
     i2 = max(1, i - round(0.002/dt));
     i1 = max(1, i2 - n_mid);
     if i2 > i1
        v_m = (rec_x(i2) - rec_x(i1)) / ((i2-i1)*dt);
     else
        v_m = xd0;
     end
     if P.enc_lsb > 0, v_m = round(v_m/P.enc_lsb)*P.enc_lsb; end
     v_meas = v_m;
     if spd_mode
        e_v = v_ref - v_meas;
        if isfield(P,'ki_speed') && (P.ki_speed > 0)
           % 速度环积分（设计研究用；固件尚未实现）：学“维持速度所需的倾角”，
           % 从而在坡上也能把速度压到 0，位置环 P 自然收敛到 0 误差。
           sat = ((P.ki_speed*i_v >= P.max_pitch) && (e_v > 0)) || ...
                 ((P.ki_speed*i_v <= -P.max_pitch) && (e_v < 0));
           if ~sat
              i_v = i_v + e_v*P.T_mid;
           end
           i_lim = P.max_pitch/P.ki_speed;
           i_v = max(min(i_v, i_lim), -i_lim);
           th_deg = kp_speed*e_v + P.ki_speed*i_v;
        else
           th_deg = kp_speed*e_v;
        end
        th_deg = max(min(th_deg, P.max_pitch), -P.max_pitch);
        th_ref = th_deg*pi/180;
     end
  end
  % ---- 250 Hz inner loop (ZOH: u held between ticks) ----
  if mod(i-1,n_in)==0
     th_m = hth(1); thd_m = hthd(1);
     if ~isempty(dist) && isfield(dist,'noise')
        th_m  = th_m  + dist.noise(1)*randn;
        thd_m = thd_m + dist.noise(2)*randn;
     end
     if use_pred
        th_used = th_m + thd_m*Td;
     else
        th_used = th_m;
     end
     u  = Kp*(th_used-th_ref) + Kd*thd_m + P.ff_speed*v_meas;
     u  = max(min(u,P.u_max),-P.u_max);
     % ---- 摩擦前馈补偿（与固件 PIDTask_ApplyDeadband 逐字一致）----
     uc = u;
     if P.dead_comp
        if P.dead_smooth
           uc = u + P.u_dead*tanh(u/P.dead_eps);     % 固件默认：平滑符号函数
        elseif u > 0
           uc = u + P.u_dead;                        % 对比：硬 sign
        elseif u < 0
           uc = u - P.u_dead;
        end
        uc = max(min(uc,P.u_max),-P.u_max);
     end
     if strcmp(P.plant,'zone')
        % 旧模型：执行器死区（|u|<dead -> 转矩 0），只用于对比
        ue = uc; if abs(ue) < P.u_dead, ue = 0; end
     else
        ue = uc;                       % 摩擦模型：死区体现在对象里的静摩擦
     end
     u_hold = ue;
     peak_u = max(peak_u,abs(u));
  end
  tau_m = D.Kt_count*u_hold - (D.tau_stall/D.v_nl)*xd;   % 电机：力矩 + 反电动势
  if strcmp(P.plant,'fric')
     % 减速箱静摩擦（Coulomb + 平滑静摩擦）：幅值 = 与实测死区等效的力矩
     tau = tau_m - P.u_dead*D.Kt_count*tanh(xd/P.v_eps);
  else
     tau = tau_m;
  end
  F   = 2*tau/Rr + Fd;                                  % external force acts on the cart
  Am  = [M, m*l*cos(th); m*l*cos(th), Jp];
  bb  = [F + m*l*thd^2*sin(th); m*g*l*sin(th) + Fd*l*cos(th)];
  acc = Am\bb;
  xd = xd + acc(1)*dt; x = x + xd*dt;
  thd = thd + acc(2)*dt; th = th + thd*dt;
  hth = [hth(2:end), th]; hthd = [hthd(2:end), thd];
  rec_t(i)=t; rec_th(i)=th*180/pi; rec_v(i)=xd; rec_x(i)=x; rec_u(i)=u_hold; rec_F(i)=Fd;
end
R.t=rec_t; R.th=rec_th; R.v=rec_v; R.x=rec_x; R.u=rec_u; R.F=rec_F;
R.peak_u = peak_u;
R.max_tilt = max(abs(rec_th));
R.max_x = max(abs(rec_x));
R.v_end = xd;
R.x_end = x;
idx = find(abs(rec_th) > 1.0, 1, 'last');
if isempty(idx), R.settle = 0; else, R.settle = rec_t(min(idx+1,n)); end
i3 = min(n, round(3.0/dt)); R.x_3s = rec_x(i3);
if pos_mode && abs(ref) > 1e-6
  i10 = find(rec_x >= 0.1*ref,1); i90 = find(rec_x >= 0.9*ref,1);
  if isempty(i10) || isempty(i90), R.rise = NaN; else, R.rise = (i90-i10)*dt; end
  R.overshoot = max(0,(max(rec_x)-ref)/abs(ref)*100);
elseif strcmp(mode,'speed') && abs(ref) > 1e-6
  i10 = find(rec_v >= 0.1*ref,1); i90 = find(rec_v >= 0.9*ref,1);
  if isempty(i10) || isempty(i90), R.rise = NaN; else, R.rise = (i90-i10)*dt; end
  R.overshoot = max(0,(max(rec_v)-ref)/abs(ref)*100);
else
  R.rise = NaN; R.overshoot = 0;
end
end

function Y = design_yaw(D,tau,wc)
K  = 2*D.v_nl/(4199*D.track);        % rad/s per count (no slip)
kp = wc*tau/K;                       % count per (rad/s): cancels plant pole
ki = kp/tau;                         % count per rad
Y.K=K; Y.kp=kp; Y.ki=ki;
Y.kp_deg = kp*pi/180;                % code unit: PWM per (deg/s)
Y.ki_deg = ki*pi/180;                % code unit: PWM per deg
L = K*(kp + ki/(1j*wc))/(1+1j*wc*tau);
Y.pm = 180 + angle(L)*180/pi;
Y.Tm = (Y.pm*pi/180)/wc;
end

function R = sim_yaw(D,Y,tau,wref_deg,Tsim,Td)
dt = 1e-4; n = round(Tsim/dt); u_max = 2100; imax = 0.3*u_max/max(Y.ki,eps);
nd = max(1,round(Td/dt));
w = 0; I = 0; rec = zeros(n,1); wbuf = zeros(1,nd+1);
for i = 1:n
  w_m = wbuf(1);                      % delayed yaw-rate measurement
  e = wref_deg*pi/180 - w_m;
  u = Y.kp*e + Y.ki*I;
  u = max(min(u,u_max),-u_max);
  if ~((u>=u_max && e>0) || (u<=-u_max && e<0))
     I = I + e*dt;                   % conditional integration
  end
  I = max(min(I,imax),-imax);        % integral contribution <= 0.3*u_max
  w = w + dt*(Y.K*u - w)/tau;
  wbuf = [wbuf(2:end), w];
  rec(i) = w*180/pi;
end
R.w_end = rec(end);
i10 = find(rec >= 0.1*wref_deg,1); i90 = find(rec >= 0.9*wref_deg,1);
if isempty(i10) || isempty(i90), R.rise = NaN; else, R.rise = (i90-i10)*dt; end
R.overshoot = max(0,(max(rec)-wref_deg)/wref_deg*100);
end

function figs(P,D,Kp,Kd,~,~)
w = logspace(0,3,1500); s = 1j*w;
L = -D.A*(Kp+Kd*s)./(s.^2-D.w0^2);
figure('Name','Balance loop L(jw)');
subplot(2,1,1); semilogx(w,20*log10(abs(L))); grid on; ylabel('|L| [dB]');
subplot(2,1,2); semilogx(w,unwrap(angle(L))*180/pi); grid on;
ylabel('arg L [deg]'); xlabel('rad/s');
R = sim_balance(P,D,Kp,Kd,0.01,5,0,'position',P.kp_speed,P.kp_pos,0,3.0);
figure('Name','Release 5 deg, 10 ms delay');
plot(R.t,R.th); grid on; xlabel('t [s]'); ylabel('pitch [deg]');
end

