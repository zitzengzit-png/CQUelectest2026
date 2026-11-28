function vis_tuned
%VIS_TUNED  用 autotune_pid 的最优参数重跑仿真并可视化
%   读取 logs/tuned_pid.txt 里的建议值（不写死），覆盖 params 后重跑：
%     位置阶跃 0.30 m / 速度阶跃 0→0.40 m/s / 5° 释放站立 / 1.5 N 推力
%   输出（matlab/logs/，git 忽略）：
%     tuned_strobe.png    位置阶跃的频闪侧视 + pitch/速度
%     tuned_compare.png   固件当前值 vs 最优值（pitch / x / v / 推力）
%     tuned_anim.gif      位置阶跃动画（侧视，轮子滚动、CoG 轨迹）
%   用法：先跑 autotune_pid，再跑 vis_tuned
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
logdir = fullfile(here, 'logs');
fpar = fullfile(logdir, 'tuned_pid.txt');
if ~exist(fpar, 'file')
    error('run autotune_pid first (missing %s)', fpar);
end
T  = read_tuned(fpar);
api = balance_model('api');
P   = api.params(12.0);
Pb  = P;                                  % 基线 = 固件当前值
P.kp_pitch = T.kp_pitch;
P.kd_pitch = T.kd_pitch;
P.kp_speed = T.kp_speed;
P.kp_pos   = T.kp_pos;
D   = api.derive(P);
Td  = P.Td_imu;
Kp  = P.kp_pitch * 180/pi;                % sim_balance 内部用 rad
Kd  = P.kd_pitch * 180/pi;
Kp0 = Pb.kp_pitch * 180/pi;
Kd0 = Pb.kd_pitch * 180/pi;
noise = struct('noise', [0.15*pi/180, 1.0*pi/180]);

fprintf('tuned : kp=%.1f kd=%.2f kps=%.1f kpp=%.2f\n', ...
        P.kp_pitch, P.kd_pitch, P.kp_speed, P.kp_pos);
fprintf('base  : kp=%.1f kd=%.2f kps=%.1f kpp=%.2f\n', ...
        Pb.kp_pitch, Pb.kd_pitch, Pb.kp_speed, Pb.kp_pos);

% ---- 仿真（strobe/GIF 不带噪声，compare 带噪声）----
S.step_t  = api.sim(P,D,Kp,Kd,Td, 0,0,'position',P.kp_speed, P.kp_pos, 0.30, 4.0, true, []);
S.step_b  = api.sim(Pb,D,Kp0,Kd0,Td, 0,0,'position',Pb.kp_speed,Pb.kp_pos,0.30, 4.0, true, []);
S.v_t     = api.sim(P,D,Kp,Kd,Td, 0,0,'speed',   P.kp_speed, P.kp_pos, 0.40, 2.5, true, []);
S.v_b     = api.sim(Pb,D,Kp0,Kd0,Td, 0,0,'speed',Pb.kp_speed,Pb.kp_pos,0.40, 2.5, true, []);
S.stand   = api.sim(P,D,Kp,Kd,Td, 5.0,0,'position',P.kp_speed, P.kp_pos, 0.0, 3.0, true, []);
S.stand_b = api.sim(Pb,D,Kp0,Kd0,Td, 5.0,0,'position',Pb.kp_speed,Pb.kp_pos,0.0, 3.0, true, []);
d = noise; d.push = [0.0 0.15 1.5];
S.push_t  = api.sim(P,D,Kp,Kd,Td, 0,0,'position',P.kp_speed, P.kp_pos, 0.0, 4.0, true, d);
S.push_b  = api.sim(Pb,D,Kp0,Kd0,Td, 0,0,'position',Pb.kp_speed,Pb.kp_pos,0.0, 4.0, true, d);
S.P = P; S.Pb = Pb; S.T = T;
save(fullfile(logdir,'balance_model_tuned.mat'), '-struct', 'S');

fprintf(' step : rise %.2f -> %.2f s, overshoot %.1f -> %.1f %%, err %.2f -> %.2f cm\n', ...
        S.step_b.rise, S.step_t.rise, S.step_b.overshoot, S.step_t.overshoot, ...
        (S.step_b.x_end-0.30)*100, (S.step_t.x_end-0.30)*100);
fprintf(' speed: overshoot %.1f -> %.1f %%, v_end %.3f -> %.3f m/s\n', ...
        S.v_b.overshoot, S.v_t.overshoot, S.v_b.v_end, S.v_t.v_end);
fprintf(' stand: max|x| %.2f -> %.2f cm, settle %.2f -> %.2f s (no noise)\n', ...
        S.stand_b.max_x*100, S.stand.max_x*100, S.stand_b.settle, S.stand.settle);
fprintf(' push : max|x| %.2f -> %.2f cm, x@4s %.2f -> %.2f cm (with noise)\n', ...
        S.push_b.max_x*100, S.push_t.max_x*100, S.push_b.x_end*100, S.push_t.x_end*100);

draw_strobe(S, logdir);
draw_compare(S, logdir);
draw_gif(S, logdir);
end

% ======================= local helpers =======================
function T = read_tuned(f)
txt = fileread(f);
T.kp_pitch = getdef(txt, 'PID_ATT_KP_PITCH');
T.kd_pitch = getdef(txt, 'PID_ATT_KD_PITCH');
T.kp_speed = getdef(txt, 'PID_SPEED_KP');
T.kp_pos   = getdef(txt, 'PID_POS_KP_DISTANCE');
end

function v = getdef(txt, name)
tok = regexp(txt, ['#define\s+' name '\s+([-+0-9.eE]+)'], 'tokens', 'once');
if isempty(tok)
    error('missing %s in tuned_pid.txt', name);
end
v = str2double(tok{1});
end

function draw_strobe(S, logdir)
sim = S.step_t; P = S.P;
t = sim.t(:); th = sim.th(:)*pi/180; x = sim.x(:); v = sim.v(:);
kend = find(t <= 2.6, 1, 'last');
if isempty(kend), kend = numel(t); end

f  = figure('Visible','off','Position',[60 60 1020 820],'Color','w');
ax1 = subplot(3,1,[1 2]); hold(ax1,'on'); box(ax1,'on'); grid(ax1,'on');
idx = unique(round(linspace(1, kend, 9)));
blue = [0.80 0.86 0.93; 0.68 0.77 0.88; 0.56 0.68 0.83; 0.44 0.59 0.77;
        0.34 0.50 0.70; 0.26 0.41 0.62; 0.20 0.33 0.54; 0.14 0.26 0.46; 0.09 0.18 0.36];
for q = 1:numel(idx)
    k = idx(q);
    bm_draw_car(ax1, P, x(k), th(k), blue(min(q,size(blue,1)),:), false);
end
plot(ax1, x(1:kend) + P.l*sin(th(1:kend)), P.R + P.l*cos(th(1:kend)), '-', ...
     'Color', [0.85 0.2 0.2 0.6], 'LineWidth', 1.2);
bm_draw_car(ax1, P, x(kend), th(kend), [0.09 0.18 0.36], true);
title(ax1, sprintf(['TUNED: kp=%.0f kd=%.1f kps=%.0f kpp=%.2f   ' ...
      'POSITION 0 -> %.2f m'], P.kp_pitch, P.kd_pitch, P.kp_speed, P.kp_pos, max(x)));
xlabel(ax1,'x [m]'); ylabel(ax1,'y [m]');
axis(ax1,'equal');
xlim(ax1, [min(x)-0.06, max(x)+0.10]);
ylim(ax1, [0 0.24]);

ax2 = subplot(3,1,3); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
yyaxis(ax2,'left');
plot(ax2, t, sim.th, 'b-', 'LineWidth', 1.4);
ylabel(ax2,'pitch [deg]');
yyaxis(ax2,'right');
plot(ax2, t, v*1000, 'r-', 'LineWidth', 1.2);
ylabel(ax2,'v [mm/s]');
xlabel(ax2,'t [s]');
title(ax2, 'pitch (blue, left) and forward speed (red, right)');

exportgraphics(f, fullfile(logdir,'tuned_strobe.png'), 'Resolution', 140);
close(f);
fprintf('written: %s\n', fullfile(logdir,'tuned_strobe.png'));
end

function draw_compare(S, logdir)
P = S.P; Pb = S.Pb;
cg = [0.55 0.55 0.55];      % 基线灰
f  = figure('Visible','off','Position',[60 60 1120 880],'Color','w');

% (1) 位置阶跃：pitch
ax = subplot(2,2,1); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax, S.step_b.t, S.step_b.th, '-', 'Color', cg, 'LineWidth', 1.3);
plot(ax, S.step_t.t, S.step_t.th, 'b-', 'LineWidth', 1.7);
xlabel(ax,'t [s]'); ylabel(ax,'pitch [deg]');
title(ax, sprintf('0.30 m step - pitch   (rise %.2f -> %.2f s)', S.step_b.rise, S.step_t.rise));
legend(ax, {sprintf('firmware  kp=%.0f kd=%.1f kpp=%.2f', Pb.kp_pitch, Pb.kd_pitch, Pb.kp_pos), ...
            sprintf('tuned  kp=%.0f kd=%.1f kpp=%.2f', P.kp_pitch, P.kd_pitch, P.kp_pos)}, ...
       'Location','northeast');

% (2) 位置阶跃：x（30 cm 目标）
ax = subplot(2,2,2); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax, [0 S.step_t.t(end)], [30 30], 'k--');
plot(ax, S.step_b.t, S.step_b.x*100, '-', 'Color', cg, 'LineWidth', 1.3);
plot(ax, S.step_t.t, S.step_t.x*100, 'r-', 'LineWidth', 1.8);
xlabel(ax,'t [s]'); ylabel(ax,'x [cm]');
title(ax, sprintf('position: overshoot %.1f -> %.1f %%, err %.2f -> %.2f cm', ...
      S.step_b.overshoot, S.step_t.overshoot, ...
      (S.step_b.x_end-0.30)*100, (S.step_t.x_end-0.30)*100));

% (3) 速度阶跃：v
ax = subplot(2,2,3); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
plot(ax, [0 S.v_t.t(end)], [400 400], 'k--');
plot(ax, S.v_b.t, S.v_b.v*1000, '-', 'Color', cg, 'LineWidth', 1.3);
plot(ax, S.v_t.t, S.v_t.v*1000, 'r-', 'LineWidth', 1.8);
xlabel(ax,'t [s]'); ylabel(ax,'v [mm/s]');
title(ax, sprintf('0 -> 0.40 m/s: overshoot %.1f -> %.1f %%', ...
      S.v_b.overshoot, S.v_t.overshoot));

% (4) 1.5 N 推力（带噪声）：x + 15 cm 要求带
ax = subplot(2,2,4); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
hp = patch(ax, [0 S.push_t.t(end) S.push_t.t(end) 0], [-15 -15 15 15], ...
           [0.85 0.93 0.85], 'EdgeColor','none', 'FaceAlpha', 0.7);
uistack(hp,'bottom');
plot(ax, [0 0.15 0.15 0], [-15 -15 15 15], 'Color', [1.0 0.9 0.6], 'LineWidth', 0.1);
plot(ax, S.push_b.t, S.push_b.x*100, '-', 'Color', cg, 'LineWidth', 1.3);
plot(ax, S.push_t.t, S.push_t.x*100, 'r-', 'LineWidth', 1.8);
xlabel(ax,'t [s]'); ylabel(ax,'x [cm]');
title(ax, sprintf('1.5 N / 0.15 s push: max|x| %.1f -> %.1f cm', ...
      S.push_b.max_x*100, S.push_t.max_x*100));

exportgraphics(f, fullfile(logdir,'tuned_compare.png'), 'Resolution', 140);
close(f);
fprintf('written: %s\n', fullfile(logdir,'tuned_compare.png'));
end

function draw_gif(S, logdir)
sim = S.step_t; P = S.P;
t = sim.t(:); th = sim.th(:)*pi/180; x = sim.x(:);
gif = fullfile(logdir,'tuned_anim.gif');
if exist(gif,'file'), delete(gif); end
frames = 1:50:numel(t);              % 50 ms/帧（慢放）
f2 = figure('Visible','off','Position',[80 80 900 400],'Color','w');
for q = 1:numel(frames)
    k = frames(q);
    clf(f2);
    ax = axes(f2); hold(ax,'on'); box(ax,'on');
    plot(ax, [x(k)-1.2, x(k)+1.2], [0 0], 'k-', 'LineWidth', 2);
    plot(ax, x(1:k) + P.l*sin(th(1:k)), P.R + P.l*cos(th(1:k)), '-', ...
         'Color', [0.85 0.10 0.10 0.75], 'LineWidth', 1.6);
    bm_draw_car(ax, P, x(k), th(k), [0.10 0.30 0.55], true);
    axis(ax,'equal');
    xlim(ax, [x(k)-0.18, x(k)+0.30]);
    ylim(ax, [0 0.22]);
    title(ax, sprintf('TUNED  t=%.2f s   pitch=%+.1f deg   v=%.0f mm/s   x=%.2f m', ...
          t(k), sim.th(k), sim.v(k)*1000, x(k)), 'FontSize', 11);
    xlabel(ax,'x [m]');
    exportgraphics(f2, gif, 'Append', q>1, 'Resolution', 90);
end
close(f2);
fprintf('written: %s\n', gif);
end
