function V2V_plot_geometry(p)
% V2V_plot_geometry Plots the inital geometry being simulated.
%
% Input parameters:
% p     V2V parameter structure
% 
% Output:
% A new figure containing the geometry
%

tailLength = 50;

xRx0 = p.xRx0(1,:);
xTx0 = p.xTx0(1,:);

figure; hold on

plot(p.xD(:,1),     p.xD(:,2),   'k.', 'MarkerSize', 8, 'LineWidth', 1.5)
plot(p.xMD0(:,1),   p.xMD0(:,2), 'b*', 'MarkerSize', 8, 'LineWidth', 1.5)
plot(p.xSD(:,1),    p.xSD(:,2),  'ms', 'MarkerSize', 8, 'LineWidth', 1.5)

plot(xTx0(1), xTx0(2), 'rv', 'MarkerSize', 8, 'LineWidth', 1.5)
plot(xRx0(1), xRx0(2), 'rd', 'MarkerSize', 8, 'LineWidth', 1.5)

plot([xTx0(1) xTx0(1)+sign(p.vTx(1))*tailLength], [xTx0(2) xTx0(2)], 'r-', 'LineWidth', 1.5)
plot([xRx0(1) xRx0(1)+sign(p.vRx(1))*tailLength], [xRx0(2) xRx0(2)], 'r-', 'LineWidth', 1.5)

plot([p.xMD0(:,1) p.xMD0(:,1)+sign(p.vMD(:,1))*tailLength]',[p.xMD0(:,2) p.xMD0(:,2)]', 'b-', 'LineWidth', 1.5)

set(gca, ...
    'FontSize', 14, ...
    'FontName', 'Times', ...
    'LineWidth', 1 ...
    )
xlabel('x [m]')
xlabel('y [m]')
grid on
title('Initial geometry')

drawnow
