function vis_stand
%VIS_STAND  要求 1：原地站立（HOLD = POSITION(0,0)），先给激励再回站立
%   读取 balance_model.m 的快照 S.stand（1.5 N / 0.15 s 推），输出 logs/stand_still.png
%   上图：频闪侧视 + 重心轨迹；中图：pitch；下图：前后位移（±15 cm 要求带）
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
logdir = fullfile(here, 'logs');
S = load(fullfile(logdir, 'balance_model.mat'));
if ~isfield(S, 'stand') || isempty(S.stand)
    error('run balance_model first (S.stand missing)');
end
P = S.P; A = S.stand;

f = figure('Visible','off','Position',[60 60 1040 820],'Color','w');

% (1) strobe: 0 -> 1.2 s
ax1 = subplot(3,1,1); hold(ax1,'on'); box(ax1,'on'); grid(ax1,'on');
t = A.t; x = A.x; th = A.th*pi/180;
kend = find(t <= 1.2, 1, 'last');
if isempty(kend), kend = numel(t); end
idx = unique(round(linspace(1, kend, 9)));
blue = [0.80 0.86 0.93; 0.68 0.77 0.88; 0.56 0.68 0.83; 0.44 0.59 0.77;
        0.34 0.50 0.70; 0.26 0.41 0.62; 0.18 0.30 0.50; 0.12 0.22 0.40; 0.08 0.15 0.30];
for q = 1:numel(idx)
    k = idx(q);
    draw_car(ax1, P, x(k), th(k), blue(min(q,size(blue,1)),:), false);
end
plot(ax1, x(1:kend) + P.l*sin(th(1:kend)), P.R + P.l*cos(th(1:kend)), '-', ...
     'Color', [0.85 0.2 0.2 0.6], 'LineWidth', 1.2);
draw_car(ax1, P, x(kend), th(kend), [0.08 0.15 0.30], true);
title(ax1, sprintf(['Stand still: 1.5 N push at 0-0.15 s   (Td=%.0f ms, Kp=%.0f Kd=%.1f, ' ...
      'friction ff on)'], 1e3*P.Td_imu, P.kp_pitch, P.kd_pitch));
xlabel(ax1,'x [m]'); ylabel(ax1,'y [m]');
axis(ax1,'equal');
xlim(ax1, [min(x)-0.05, max(x)+0.08]);
ylim(ax1, [0 0.24]);

% (2) pitch
ax2 = subplot(3,1,2); hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
plot(ax2, A.t, A.th, 'b-', 'LineWidth', 1.3);
plot(ax2, [0 A.t(end)], [0 0], 'k--');
yl = [-20 20];
hp = patch(ax2, [0 0.15 0.15 0], [yl(1) yl(1) yl(2) yl(2)], ...
           [1.0 0.9 0.6], 'EdgeColor', 'none', 'FaceAlpha', 0.55);
uistack(hp, 'bottom');
ylim(ax2, yl);
ylabel(ax2,'pitch [deg]'); xlabel(ax2,'t [s]');
title(ax2, sprintf('pitch (blue), shaded = push window;  max |pitch| = %.2f deg', A.max_tilt));

% (3) forward position vs the 15 cm requirement
ax3 = subplot(3,1,3); hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
hp = patch(ax3, [0 3 3 0], [-15 -15 15 15], [0.85 0.93 0.85], ...
           'EdgeColor', 'none', 'FaceAlpha', 0.7);
uistack(hp, 'bottom');
plot(ax3, [0 3], [2 2], 'k--'); plot(ax3, [0 3], [-2 -2], 'k--');
plot(ax3, A.t, A.x*100, 'r-', 'LineWidth', 1.6);
plot(ax3, A.t(end), A.x(end)*100, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
xlim(ax3, [0 A.t(end)]);
ylabel(ax3,'x [cm]'); xlabel(ax3,'t [s]');
legend(ax3, {'requirement 1 band (\pm15 cm)', '', '', 'forward position'}, ...
       'Location', 'northeast');
title(ax3, sprintf('forward position: max %.1f cm, ends %.1f cm  (%s)', ...
      A.max_x*100, A.x_end*100, ternary_local(A.max_x <= 0.15, 'PASS', 'FAIL')));

exportgraphics(f, fullfile(logdir, 'stand_still.png'), 'Resolution', 140);
close(f);
fprintf('written: %s\n', fullfile(logdir, 'stand_still.png'));
end

function out = ternary_local(c, a, b)
if c, out = a; else, out = b; end
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
