function autotune_pid(method, budget, max_time_s)
%AUTOTUNE_PID  在仿真里自动整定共模三环的 4 个 PID 参数
%
%   method：
%     'pattern'       坐标模式搜索（默认，**不需要任何工具箱**，可移植）
%     'patternsearch' Global Optimization Toolbox（带边界，推荐的正规做法）
%     'fmincon'       Optimization Toolbox（SQP，带边界）
%     'fminsearch'    基座 MATLAB 的 Nelder-Mead（无约束，用罚函数限边界）
%   三种工具箱方法都在 log2 空间优化，与 'pattern' 的目标函数完全一致。
%
%   调参对象（与固件宏一一对应）：
%     kp_pitch  PID_ATT_KP_PITCH      平衡内环 P   [PWM/deg]
%     kd_pitch  PID_ATT_KD_PITCH      平衡内环 D   [PWM/(deg/s)]
%     kp_speed  PID_SPEED_KP          速度中环 P   [deg/(m/s)]
%     kp_pos    PID_POS_KP_DISTANCE   位置外环 P   [1/s]
%
%   用法：
%     autotune_pid                        % pattern, 200 evals / 420 s
%     autotune_pid(400, 600)              % 兼容旧用法：budget, max_time
%     autotune_pid('patternsearch')       % 用 Global Optimization Toolbox
%     autotune_pid('fmincon', 400, 600)
%
%   输出：matlab/logs/tuned_pid.txt（可直接抄进 robot_config.h）

if nargin < 1 || isempty(method), method = 'pattern'; end
if isnumeric(method)                       % 兼容 autotune_pid(budget, max_time)
    if nargin >= 2, max_time_s = budget; else, max_time_s = 420; end
    budget = method;
    method = 'pattern';
else
    if nargin < 2 || isempty(budget),     budget = 200;     end
    if nargin < 3 || isempty(max_time_s), max_time_s = 420; end
end

api = balance_model('api');            % 复用主脚本的 params/derive/sim
P   = api.params(12.0);
D   = api.derive(P);
Td  = P.Td_imu;

cases = {'stand','step','speed','push','bias'};
W.stand = 1.2; W.step = 1.5; W.speed = 1.0; W.push = 1.2; W.bias = 0.5;

x0    = [P.kp_pitch, P.kd_pitch, P.kp_speed, P.kp_pos];
lo    = [60,  2,  4,  0.30];
hi    = [600, 40, 40, 4.00];
names = {'kp_pitch','kd_pitch','kp_speed','kp_pos'};

fprintf('==============================================================\n');
fprintf(' PID auto-tune   method = %s\n', method);
fprintf(' start: kp_pitch=%.1f kd_pitch=%.2f kp_speed=%.1f kp_pos=%.2f\n', x0);
fprintf(' bound: kp %.0f..%.0f, kd %.0f..%.0f, kps %.0f..%.0f, kpp %.1f..%.1f\n', ...
        lo(1),hi(1),lo(2),hi(2),lo(3),hi(3),lo(4),hi(4));
fprintf(' budget=%d evals, max %.0f s\n', budget, max_time_s);
fprintf('--------------------------------------------------------------\n');

t_start = tic;
evals = 1;
best  = x0;
Jbest = total_cost(best, api, P, D, Td, cases, W);
fprintf(' [%3d] J=%9.4f  (start)\n', evals, Jbest);

if strcmpi(method, 'pattern')

step    = 0.5;      % log2 步长
minstep = 0.04;
while (step >= minstep) && (evals < budget) && (toc(t_start) < max_time_s)
    improved = false;
    for i = 1:numel(best)
        for s = [1 -1]
            if (evals >= budget) || (toc(t_start) >= max_time_s), break; end
            xc = best;
            xc(i) = best(i) * 2^(s*step);
            xc = min(max(xc, lo), hi);
            if xc(i) == best(i), continue; end

            Jc = total_cost(xc, api, P, D, Td, cases, W);
            evals = evals + 1;
            if Jc < Jbest - 1e-5
                best = xc; Jbest = Jc; improved = true;
                fprintf(' [%3d] J=%9.4f  %-8s %s  ->  kp=%.1f kd=%.2f kps=%.1f kpp=%.2f\n', ...
                        evals, Jbest, names{i}, ternary_str(s>0,'x2','/2'), best);
                break;
            end
        end
    end
    if ~improved
        step = step / 2;
        fprintf('       (no improvement, step -> 2^%.2f)\n', step);
    end
end

% ---------- 末段细化：±3% / ±6% 局部搜索 ----------
for pass = 1:2
    for i = 1:numel(best)
        for f = [1.03 1.06 0.97 0.94]
            if evals >= budget, break; end
            xc = best;
            xc(i) = best(i)*f;
            xc = min(max(xc, lo), hi);
            if xc(i) == best(i), continue; end
            Jc = total_cost(xc, api, P, D, Td, cases, W);
            evals = evals + 1;
            if Jc < Jbest - 1e-6
                best = xc; Jbest = Jc;
                fprintf(' [%3d] J=%9.4f  %-8s x%.2f ->  kp=%.1f kd=%.2f kps=%.1f kpp=%.2f\n', ...
                        evals, Jbest, names{i}, f, best);
            end
        end
    end
end

else
    % ---------------- 工具箱方法（目标函数与 'pattern' 完全相同） ----------------
    lb  = log2(lo);
    ub  = log2(hi);
    obj = @(z) total_cost(2.^z, api, P, D, Td, cases, W);
    out = struct('funcCount', NaN);

    switch lower(method)
        case 'patternsearch'
            opts = optimoptions('patternsearch', 'Display', 'iter', ...
                                'MaxFunctionEvaluations', budget, ...
                                'MaxTime', max_time_s, ...
                                'StepTolerance', 1e-4, ...
                                'PollMethod', 'GPSPositiveBasis2N', ...
                                'UseCompletePoll', true);
            [z, Jbest, ~, out] = patternsearch(obj, log2(x0), [],[],[],[], lb, ub, [], opts);

        case 'fmincon'
            opts = optimoptions('fmincon', 'Display', 'iter', ...
                                'Algorithm', 'sqp', ...
                                'MaxFunctionEvaluations', budget, ...
                                'MaxIterations', 200, ...
                                'StepTolerance', 1e-6);
            [z, Jbest, ~, out] = fmincon(obj, log2(x0), [],[],[],[], lb, ub, [], opts);

        case 'fminsearch'
            pen = @(z) obj(z) + 1e3*sum(max(0, lb - z).^2 + max(0, z - ub).^2);
            o = optimset('Display', 'iter', 'MaxFunEvals', budget, 'MaxIter', 400, ...
                         'TolX', 1e-4, 'TolFun', 1e-4);
            [z, Jbest, ~, out] = fminsearch(pen, log2(x0), o);

        otherwise
            error('unknown method ''%s'' (pattern | patternsearch | fmincon | fminsearch)', method);
    end

    best  = min(max(2.^z, lo), hi);
    Jbest = total_cost(best, api, P, D, Td, cases, W);   % 用裁剪后的真值重算
    if isfield(out, 'funcCount') && isfinite(out.funcCount), evals = out.funcCount; end
    fprintf('--------------------------------------------------------------\n');
    fprintf(' %s -> kp=%.1f kd=%.2f kps=%.1f kpp=%.2f   J=%.4f\n', ...
            lower(method), best, Jbest);
end

J0 = total_cost(x0, api, P, D, Td, cases, W);
fprintf('--------------------------------------------------------------\n');
fprintf(' evals=%d  time=%.1f s  J: %.4f -> %.4f  (%.0f%% lower)\n', ...
        evals, toc(t_start), J0, Jbest, 100*(1-Jbest/J0));

% ---------- 噪声下的最终验证与对比 ----------
fprintf('\n validation (sensor noise 0.15 deg / 1 deg/s rms):\n');
fprintf('   baseline: kp=%.1f kd=%.2f kps=%.1f kpp=%.2f\n', x0);
fprintf('   tuned   : kp=%.1f kd=%.2f kps=%.1f kpp=%.2f\n', best);

for c = 1:numel(cases)
    Rb = run_case(api,P,D,Td,x0,  cases{c}, true);
    Rt = run_case(api,P,D,Td,best,cases{c}, true);
    [vb, nm] = case_metrics(Rb, cases{c});
    vt = case_metrics(Rt, cases{c});
    fprintf(' %s:\n', cases{c});
    for k = 1:numel(vb)
        fprintf('   %-18s %10.2f  ->  %10.2f\n', nm{k}, vb(k), vt(k));
    end
end

% ---------- 相位裕度 / 延迟裕度校验 ----------
% delay_margin 返回 [wc(rad/s), pm(deg), Tm(s)]
[wc_b, pm_b, tm_b] = api.delay_margin(D, x0(1)*180/pi,   x0(2)*180/pi);
[wc_t, pm_t, tm_t] = api.delay_margin(D, best(1)*180/pi, best(2)*180/pi);
pm10_b = pm_b - wc_b*Td*180/pi;    % 10 ms 延迟后的相位裕度
pm10_t = pm_t - wc_t*Td*180/pi;
fprintf('\n gain crossover  : %.1f -> %.1f rad/s\n', wc_b, wc_t);
fprintf(' phase margin    : %.1f -> %.1f deg     (no delay)\n', pm_b, pm_t);
fprintf(' PM at %.0f ms    : %.1f -> %.1f deg     (limit 40)\n', Td*1e3, pm10_b, pm10_t);
fprintf(' delay margin    : %.1f -> %.1f ms      (limit %.0f)\n', tm_b*1e3, tm_t*1e3, Td*1e3);
fprintf(' verdict         : %s\n', ternary_str((pm10_t >= 40) && (tm_t >= Td), 'PASS', 'FAIL'));

% ---------- 输出建议 ----------
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
logdir = fullfile(here, 'logs');
if ~exist(logdir,'dir'), mkdir(logdir); end
fname = fullfile(logdir, 'tuned_pid.txt');
fid = fopen(fname, 'w');
fprintf(fid, '%% autotune_pid (%s): balance_model, 12 V, Td=%.0f ms, %d evals, J %.4f -> %.4f\n', ...
        method, Td*1e3, evals, J0, Jbest);
fprintf(fid, '#define PID_ATT_KP_PITCH          %.2ff\n', best(1));
fprintf(fid, '#define PID_ATT_KD_PITCH          %.2ff\n', best(2));
fprintf(fid, '#define PID_SPEED_KP              %.2ff\n', best(3));
fprintf(fid, '#define PID_POS_KP_DISTANCE       %.2ff\n', best(4));
fprintf(fid, '%% baseline: %.2f / %.2f / %.2f / %.2f\n', x0);
fclose(fid);
fprintf('\n written: %s\n', fname);
end

% ======================= local helpers =======================
function s = ternary_str(c, a, b)
if c, s = a; else, s = b; end
end

function J = total_cost(x, api, P, D, Td, cases, W)
J = 0;
for c = 1:numel(cases)
    R = run_case(api, P, D, Td, x, cases{c}, false);   % 调参时无噪声
    J = J + W.(cases{c}) * case_cost(R, cases{c}, P);
end
end

function R = run_case(api, P, D, Td, x, which, noisy)
% 注意：sim_balance 内部把 th 当弧度，所以 Kp 用 PWM/rad、Kd 用 PWM/(rad/s)，
% 而固件宏（以及 x）用 PWM/deg、PWM/(deg/s) —— 这里必须乘 180/pi。
kp  = x(1) * 180/pi;
kd  = x(2) * 180/pi;
kps = x(3);
kpp = x(4);
if noisy
    d = struct('noise', [0.15*pi/180, 1.0*pi/180]);
else
    d = struct();
end
switch which
    case 'stand'
        R = api.sim(P, D, kp, kd, Td, 5.0, 0.0, 'position', kps, kpp, 0.0, 3.0, true, d);
    case 'step'
        R = api.sim(P, D, kp, kd, Td, 0.0, 0.0, 'position', kps, kpp, 0.30, 4.0, true, d);
    case 'speed'
        R = api.sim(P, D, kp, kd, Td, 0.0, 0.0, 'speed',   kps, kpp, 0.40, 3.0, true, d);
    case 'push'
        d.push = [0.0 0.15 1.5];
        R = api.sim(P, D, kp, kd, Td, 0.0, 0.0, 'position', kps, kpp, 0.0, 4.0, true, d);
    case 'bias'
        d.bias = 0.4;
        R = api.sim(P, D, kp, kd, Td, 0.0, 0.0, 'position', kps, kpp, 0.0, 5.0, true, d);
    otherwise
        error('unknown case %s', which);
end
end

function J = case_cost(R, which, P)
% 归一化：每个指标除以“理想值”，超过理想值产生平方惩罚；倾倒直接判 100
if R.max_tilt > 20, J = 100; return; end
switch which
    case 'stand'
        J = (R.max_tilt/6)^2 + (R.max_x/0.06)^2 + (R.settle/0.6)^2 + (abs(R.x_end)/0.015)^2;
    case 'step'
        J = (R.overshoot/4)^2 + (R.rise/0.7)^2 + (abs(R.x_end-0.30)/0.008)^2;
    case 'speed'
        J = (R.overshoot/6)^2 + (R.rise/0.35)^2 + (abs(R.v_end-0.40)/0.01)^2;
    case 'push'
        J = (R.max_tilt/10)^2 + (R.max_x/0.10)^2 + (abs(R.x_end)/0.015)^2;
    case 'bias'
        J = (abs(R.x_end)/0.10)^2;
    otherwise
        J = 0;
end
if J > 25, J = 25; end                       % 单工况封顶，避免一项压死全部
if R.peak_u > 0.97*P.u_max, J = J + 3; end   % 贴近饱和再罚
end

function [v, nm] = case_metrics(R, which)
switch which
    case 'stand'
        v  = [R.max_tilt, R.max_x*100, R.x_end*100, R.settle];
        nm = {'max|tilt| [deg]','max|x| [cm]','x@3s [cm]','settle [s]'};
    case 'step'
        v  = [R.overshoot, R.rise, (R.x_end-0.30)*100];
        nm = {'overshoot [%]','rise [s]','err@4s [cm]'};
    case 'speed'
        v  = [R.overshoot, R.rise, R.v_end];
        nm = {'overshoot [%]','rise [s]','v_end [m/s]'};
    case 'push'
        v  = [R.max_tilt, R.max_x*100, R.x_end*100];
        nm = {'max|tilt| [deg]','max|x| [cm]','x@4s [cm]'};
    case 'bias'
        v  = [R.x_end*100, R.max_tilt];
        nm = {'x@5s [cm]','max|tilt| [deg]'};
    otherwise
        v = 0; nm = {''};
end
end
