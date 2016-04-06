function V2V_plot_geometry(p)
% V2V_plot_geometry Plots the inital geometry being simulated.
%
% Input parameters:
% p     V2V parameter structure
% 
% Output:
% A new figure containing the geometry
%

xRx0 = p.xRx0(1,:);
xTx0 = p.xTx0(1,:);

figure; hold on

plot(p.xD(:,1),     p.xD(:,2),   'k.','markersize', 8, 'linewidth', 1.5)
plot(p.xMD0(:,1),   p.xMD0(:,2), 'b*','markersize', 8, 'linewidth', 1.5)
plot(p.xSD(:,1),    p.xSD(:,2),  'ms','markersize', 8, 'linewidth', 1.5)

plot(xTx0(1),xTx0(2),'rv','markersize',8,'linewidth',1.5)
plot(xRx0(1),xRx0(2),'rd','markersize',8,'linewidth',1.5)

if p.vTx(1) > 0
    plot([xTx0(1) xTx0(1)+50], [xTx0(2) xTx0(2)], 'r-', 'linewidth',1.5)
else
    plot([xTx0(1) xTx0(1)-50],[xTx0(2) xTx0(2)], 'r-', 'linewidth',1.5)
end

if p.vRx(1) > 0
    L_line = line([xRx0(1) xRx0(1)+50],[xRx0(2) xRx0(2)]); set(L_line,'linestyle','-','color','r','linewidth',1.5)
else
    L_line = line([xRx0(1) xRx0(1)-50],[xRx0(2) xRx0(2)]); set(L_line,'linestyle','-','color','r','linewidth',1.5)
end



for ctr_MD = 1:p.N_MD
    if p.vMD(ctr_MD,1) < 0
        L_line = line([p.xMD0(ctr_MD,1) p.xMD0(ctr_MD,1)-50],[p.xMD0(ctr_MD,2) p.xMD0(ctr_MD,2)]); set(L_line,'linestyle','-','color','b','linewidth',1.5)
    else
        L_line = line([p.xMD0(ctr_MD,1) p.xMD0(ctr_MD,1)+50],[p.xMD0(ctr_MD,2) p.xMD0(ctr_MD,2)]); set(L_line,'linestyle','-','color','b','linewidth',1.5)
    end
end

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
