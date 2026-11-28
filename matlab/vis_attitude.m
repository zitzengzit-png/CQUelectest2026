function vis_attitude
%VIS_ATTITUDE  姿态环单独工作 + 环境干扰（1.5 N 推 0.15 s）：静态图
%   读取 balance_model.m 的快照，输出 logs/attitude_dist.png
%   蓝 = 只跑 150/250 Hz 姿态环；红 = HOLD/POSITION(0,0)（带速度/位置外环）
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
logdir = fullfile(here, 'logs');
S = load(fullfile(logdir, 'balance_model.mat'));
if ~isfield(S, 'sim_att') || isempty(S.sim_att)
    error('run balance_model first (sim_att missing)');
end
P = S.P; A = S.sim_att; B = S.sim_att_pos;

f = figure('Visible','off','Position',[60 60 1040 820],'Color','w');

% (1) strobe of the attitude-only car
ax1 = subplot(3,1,1); hold(ax1,'on'); box(ax1,'on'); grid(ax1,'on');
t = A.t; x = A.x; th = A.th*pi/180;
kend = find(t <= 1.2, 1, 'last');
idx = unique(round(linspace(1, kend, 8)));
blue = [0.80 0.86 0.93; 0.68 0.77 0.88; 0.56 0.68 0.83; 0.44 0.59 0.77;
        0.34 0.50 0.70; 0.26 0.41 0.62; 0.18 0.30 0.50; 0.12 0.22 0.40];
for q = 1:numel(idx)
    draw_car(ax1, P, x(idx(q)), th(idx(q)), blue(min(q,8),:), false);
end
plot(ax1, x(1:kend)+P.l*sin(th(1:kend)), P.R+P.l*cos(th(1:kend)), '-', ...
     'Color', [0.85 0.2 0.2 0.6], 'LineWidth', 1.2);
draw_car(ax1, P, x(kend), th(kend), [0.12 0.22 0.40], true);
title(ax1, 'ATTITUDE loop only + 1.5 N push (0.5-0.65 s): stays upright, slides right');
xlabel(ax1,'x [m]'); ylabel(ax1,'y [m]');
axis(ax1,'equal');
xlim(ax1, [min(x)-0.06, max(x(1:kend))+0.12]);
ylim(ax1, [0 0.24]);

% (2) pitch + disturbance band
ax2 = subplot(3,1,2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
h1 = plot(ax2, A.t, A.th, 'b-', 'LineWidth', 1.3);
h2 = plot(ax2, B.t, B.th, 'r-', 'LineWidth', 1.3);
yl = ylim(ax2);
hp = patch(ax2, [0.5 0.65 0.65 0.5], [yl(1) yl(1) yl(2) yl(2)], ...
           [1.0 0.9 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.55);
uistack(hp, 'bottom');
ylabel(ax2,'pitch [deg]'); xlabel(ax2,'t [s]');
title(ax2, 'pitch response (shaded = 1.5 N push at 0.5-0.65 s)');
legend(ax2, [h1 h2], {'attitude only', 'POSITION hold'}, 'Location', 'northeast');

% (3) forward position
ax3 = subplot(3,1,3);
plot(ax3, A.t, A.x*100, 'b-', 'LineWidth', 1.3); hold(ax3,'on'); grid(ax3,'on');
plot(ax3, B.t, B.x*100, 'r-', 'LineWidth', 1.3);
xlabel(ax3,'t [s]'); ylabel(ax3,'x [cm]');
legend(ax3, {sprintf('attitude only: drifts %.0f cm', A.x_end*100), ...
             sprintf('POSITION hold: ends %.0f cm', B.x_end*100)}, 'Location', 'northwest');
title(ax3, 'forward position');

exportgraphics(f, fullfile(logdir, 'attitude_dist.png'), 'Resolution', 140);
close(f);
fprintf('written: %s\n', fullfile(logdir, 'attitude_dist.png'));
end

function draw_car(ax, P, x, th, col, show_cog)
R = P.R; l = P.l;
up = [sin(th), cos(th)]; rt = [cos(th), -sin(th)]; axle = [x, R];
a = linspace(0, 2*pi, 90);
plot(ax, x + R*cos(a), R + R*sin(a), '-', 'Color', col, 'LineWidth', 1.5);
plot(ax, x + 0.006*cos(a), R + 0.006*sin(a), '-', 'Color', col, 'LineWidth', 1.0);
phi = -x/R;
for k = 0:2
    ang = phi + k*2*pi/3;
    plot(ax, x + [0 R*0.82*cos(ang)], R + [0 R*0.82*sin(ang)], '-', ...
         'Color', col, 'LineWidth', 0.8);
end
for sv = [-0.045 0.045]
    p0 = axle + sv*rt; p1 = axle + 0.10*up + sv*rt;
    plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', col, 'LineWidth', 1.0);
end
p1 = axle + 0.10*up - 0.06*rt; p2 = axle + 0.10*up + 0.06*rt;
plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], '-', 'Color', col, 'LineWidth', 2.2);
corners = [0.062 -0.032; 0.062 0.032; 0.094 0.032; 0.094 -0.032];
W = axle + corners(:,1)*up + corners(:,2)*rt;
patch(ax, W(:,1), W(:,2), col, 'FaceAlpha', 0.16, 'EdgeColor', col, 'LineWidth', 0.8);
if show_cog
    c = axle + l*up;
    plot(ax, c(1), c(2), 'o', 'MarkerSize', 7, ...
         'MarkerFaceColor', [0.85 0.15 0.15], 'MarkerEdgeColor', 'k');
end
end
