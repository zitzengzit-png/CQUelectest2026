function bm_draw_car(ax, P, x, th, col, show_cog)
%BM_DRAW_CAR  侧视画一辆两轮平衡车（轮子、立柱、顶板、板卡、重心）
%   供 vis_tuned.m 等可视化脚本共用；轮子滚动角 phi = -x/R（不打滑）
R = P.R; l = P.l;
up = [sin(th), cos(th)];        % 车身“向上”方向
rt = [cos(th), -sin(th)];       % 车身“向右”方向
axle = [x, R];

a = linspace(0, 2*pi, 90);
plot(ax, x + R*cos(a), R + R*sin(a), '-', 'Color', col, 'LineWidth', 1.6);
plot(ax, x + 0.006*cos(a), R + 0.006*sin(a), '-', 'Color', col, 'LineWidth', 1.0);
phi = -x/R;
for k = 0:2
    ang = phi + k*2*pi/3;
    plot(ax, x + [0 R*0.82*cos(ang)], R + [0 R*0.82*sin(ang)], '-', ...
         'Color', col, 'LineWidth', 0.8);
end

for sv = [-0.045 0.045]
    p0 = axle + sv*rt;
    p1 = axle + 0.10*up + sv*rt;
    plot(ax, [p0(1) p1(1)], [p0(2) p1(2)], '-', 'Color', col, 'LineWidth', 1.0);
end
p1 = axle + 0.10*up - 0.06*rt;
p2 = axle + 0.10*up + 0.06*rt;
plot(ax, [p1(1) p2(1)], [p1(2) p2(2)], '-', 'Color', col, 'LineWidth', 2.4);

corners = [0.062 -0.032; 0.062 0.032; 0.094 0.032; 0.094 -0.032];
W = axle + corners(:,1)*up + corners(:,2)*rt;
patch(ax, W(:,1), W(:,2), col, 'FaceAlpha', 0.16, 'EdgeColor', col, 'LineWidth', 0.8);

if show_cog
    c = axle + l*up;
    plot(ax, c(1), c(2), 'o', 'MarkerSize', 7, ...
         'MarkerFaceColor', [0.85 0.15 0.15], 'MarkerEdgeColor', 'k');
end
end
