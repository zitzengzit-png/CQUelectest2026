function vis_balance
%VIS_BALANCE  GA25-370 两轮平衡车运动可视化（侧视）
%   读取 balance_model.m 生成的快照，输出：
%     logs/balance_strobe.png   频闪侧视 + pitch/速度曲线（0.10 m 位置阶跃）
%     logs/balance_anim.gif     侧视动画：轮子滚动、车体俯仰、CoG 轨迹（0 -> 0.40 m/s）
%   用法：先跑 balance_model，再跑 vis_balance

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
logdir = fullfile(here, 'logs');
matf = fullfile(logdir, 'balance_model.mat');
if ~exist(matf, 'file')
    error('run balance_model first (missing %s)', matf);
end
S = load(matf);
P = S.P;

%% ---------- static: strobe + traces (0.10 m position step) ----------
sim = S.sim_pos;
t = sim.t(:); th = sim.th(:)*pi/180; x = sim.x(:); v = sim.v(:);
n = numel(t);

f1 = figure('Visible','off','Position',[80 80 1020 780],'Color','w');
ax1 = subplot(3,1,[1 2]); hold(ax1,'on'); box(ax1,'on'); grid(ax1,'on');

kend = find(t <= 2.6, 1, 'last');
if isempty(kend), kend = n; end
idx = unique(round(linspace(1, kend, 9)));
blue = [0.80 0.86 0.93; 0.68 0.77 0.88; 0.56 0.68 0.83; 0.44 0.59 0.77;
        0.34 0.50 0.70; 0.26 0.41 0.62; 0.20 0.33 0.54; 0.14 0.26 0.46; 0.09 0.18 0.36];
for q = 1:numel(idx)
    k = idx(q);
    draw_car(ax1, P, x(k), th(k), blue(min(q,size(blue,1)),:), false);
end
plot(ax1, x + P.l*sin(th), P.R + P.l*cos(th), '-', ...
     'Color', [0.85 0.2 0.2 0.6], 'LineWidth', 1.2);
draw_car(ax1, P, x(kend), th(kend), [0.09 0.18 0.36], true);
title(ax1, sprintf(['Strobe: POSITION 0 -> %.2f m   (Td=%.0f ms + pitch prediction, ' ...
      'Kp=%.0f Kd=%.1f Kspeed=%.0f)'], max(x), 1e3*P.Td_imu, ...
      P.kp_pitch, P.kd_pitch, P.kp_speed));
xlabel(ax1,'x [m]'); ylabel(ax1,'y [m]');
axis(ax1,'equal');
xlim(ax1, [min(x)-0.06, max(x)+0.10]);
ylim(ax1, [0 0.24]);

ax2 = subplot(3,1,3);
yyaxis(ax2,'left'); plot(ax2, t, sim.th, 'b-', 'LineWidth', 1.2);
ylabel(ax2,'pitch [deg]');
yyaxis(ax2,'right'); plot(ax2, t, v*1000, 'r-', 'LineWidth', 1.2);
ylabel(ax2,'v [mm/s]');
xlabel(ax2,'t [s]'); grid(ax2,'on');
title(ax2,'pitch (blue) and forward speed (red)');

exportgraphics(f1, fullfile(logdir,'balance_strobe.png'), 'Resolution', 140);
close(f1);
fprintf('written: %s\n', fullfile(logdir,'balance_strobe.png'));

%% ---------- animation: 0 -> 0.40 m/s ----------
sim = S.sim;
t = sim.t(:); th = sim.th(:)*pi/180; x = sim.x(:); v = sim.v(:);
n = numel(t);
gif = fullfile(logdir,'balance_anim.gif');
if exist(gif,'file'), delete(gif); end

frames = 1:40:n;                     % 40 ms per frame (slow motion)
f2 = figure('Visible','off','Position',[80 80 900 400],'Color','w');
for q = 1:numel(frames)
    k = frames(q);
    clf(f2);
    ax = axes(f2); hold(ax,'on'); box(ax,'on');
    ground(ax, x(k)-1.2, x(k)+1.2);
    plot(ax, x(1:k) + P.l*sin(th(1:k)), P.R + P.l*cos(th(1:k)), '-', ...
         'Color', [0.85 0.10 0.10 0.75], 'LineWidth', 1.6);
    draw_car(ax, P, x(k), th(k), [0.12 0.27 0.50], true);
    axis(ax,'equal');
    xlim(ax, [x(k)-0.18, x(k)+0.30]);
    ylim(ax, [0 0.22]);
    title(ax, sprintf('t = %.2f s   pitch = %+.1f deg   v = %.0f mm/s   x = %.2f m', ...
          t(k), sim.th(k), v(k)*1000, x(k)), 'FontSize', 12);
    xlabel(ax,'x [m]');
    exportgraphics(f2, gif, 'Append', q>1, 'Resolution', 95);
end
close(f2);
fprintf('written: %s\n', gif);
end


% ======================= helpers =======================
function ground(ax, x0, x1)
plot(ax, [x0 x1], [0 0], 'k-', 'LineWidth', 2);
end

function draw_car(ax, P, x, th, col, show_cog)
R = P.R; l = P.l;
up = [sin(th), cos(th)];        % body "up" in world frame
rt = [cos(th), -sin(th)];       % body "right" in world frame
axle = [x, R];

% wheel + hub + 3 spokes (rolling angle phi = -x/R)
a = linspace(0, 2*pi, 90);
plot(ax, x + R*cos(a), R + R*sin(a), '-', 'Color', col, 'LineWidth', 1.6);
plot(ax, x + 0.006*cos(a), R + 0.006*sin(a), '-', 'Color', col, 'LineWidth', 1.0);
phi = -x/R;
for k = 0:2
    ang = phi + k*2*pi/3;
    plot(ax, x + [0 R*0.82*cos(ang)], R + [0 R*0.82*sin(ang)], '-', ...
         'Color', col, 'LineWidth', 0.8);
end

% two struts
for sv = [-0.045 0.045]
    p0 = axle + sv*rt;
    p1 = axle + 0.10*up + sv*rt;
    plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', col, 'LineWidth', 1.0);
end
% top plate
p1 = axle + 0.10*up - 0.06*rt;
p2 = axle + 0.10*up + 0.06*rt;
plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], '-', 'Color', col, 'LineWidth', 2.4);

% board / battery block (rotated rectangle)
corners = [0.062 -0.032; 0.062 0.032; 0.094 0.032; 0.094 -0.032];
W = axle + corners(:,1)*up + corners(:,2)*rt;
patch(ax, W(:,1), W(:,2), col, 'FaceAlpha', 0.16, 'EdgeColor', col, 'LineWidth', 0.8);

% CoG
if show_cog
    c = axle + l*up;
    plot(ax, c(1), c(2), 'o', 'MarkerSize', 7, ...
         'MarkerFaceColor', [0.85 0.15 0.15], 'MarkerEdgeColor', 'k');
end
end
