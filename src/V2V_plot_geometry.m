% V2V_plot_geometry ... Plots inital geometry
%
% Input parameters:
% p ... V2V parameter structure
% 
% Output:
% A new figure containing the geometry
%

function V2V_plot_geometry(p)

xRxsav = p.xRx0;
xTxsav = p.xTx0;

xRx0 = p.xRx0(1,:);
xTx0 = p.xTx0(1,:);


figure;

% subplot(2,1,1)
plot(xTx0(1),xTx0(2),'rv','markersize',8,'linewidth',1.5)
hold on
if p.vTx(1) > 0
    L_line = line([xTx0(1) xTx0(1)+50],[xTx0(2) xTx0(2)]); set(L_line,'linestyle','-','color','r','linewidth',1.5)
else
    L_line = line([xTx0(1) xTx0(1)-50],[xTx0(2) xTx0(2)]); set(L_line,'linestyle','-','color','r','linewidth',1.5)
end
plot(xRx0(1),xRx0(2),'rd','markersize',8,'linewidth',1.5)
if p.vRx(1) > 0
    L_line = line([xRx0(1) xRx0(1)+50],[xRx0(2) xRx0(2)]); set(L_line,'linestyle','-','color','r','linewidth',1.5)
else
    L_line = line([xRx0(1) xRx0(1)-50],[xRx0(2) xRx0(2)]); set(L_line,'linestyle','-','color','r','linewidth',1.5)
end
for ctr_D = 1:p.N_D
    plot(p.xD(ctr_D,1),p.xD(ctr_D,2),'k.','markersize',8,'linewidth',1.5)
end
for ctr_MD = 1:p.N_MD
    plot(p.xMD0(ctr_MD,1),p.xMD0(ctr_MD,2),'b*','markersize',8,'linewidth',1.5)
    if p.vMD(ctr_MD,1) < 0
        L_line = line([p.xMD0(ctr_MD,1) p.xMD0(ctr_MD,1)-50],[p.xMD0(ctr_MD,2) p.xMD0(ctr_MD,2)]); set(L_line,'linestyle','-','color','b','linewidth',1.5)
    else
        L_line = line([p.xMD0(ctr_MD,1) p.xMD0(ctr_MD,1)+50],[p.xMD0(ctr_MD,2) p.xMD0(ctr_MD,2)]); set(L_line,'linestyle','-','color','b','linewidth',1.5)
    end
end
for ctr_SD = 1:p.N_SD
    plot(p.xSD(ctr_SD,1),p.xSD(ctr_SD,2),'ms','markersize',8,'linewidth',1.5)
end
set(gca,'fontsize',14,'fontname','times','linewidth',1)
xlabel('x [m]','fontsize',14,'fontname','times')
xlabel('y [m]','fontsize',14,'fontname','times')
grid on
title('Initial geometry','fontsize',14,'fontname','times')

xRx0 = xRxsav;
xTx0 = xTxsav;

drawnow

%%
% figure;
% plot(d_LOS,1:length(d_LOS),'r-','linewidth',1.5)
% hold on
% for ctr_MD = 1:N_MD
%     plot(squeeze(d_MD(1,ctr_MD,:))/300,T,'b--','linewidth',1.5)
% end
% for ctr_SD = 1:N_SD
%     plot(squeeze(d_SD(1,ctr_SD,:))/300,T,'m-.','linewidth',1.5)
% end
% set(gca,'fontsize',14,'fontname','times','linewidth',1)
% xlabel('Delay [\mu s]','fontsize',14,'fontname','times')
% ylabel('Time [s]','fontsize',14,'fontname','times')
% grid on
% title('Theoretic time-delay plot','fontsize',14,'fontname','times')
% axis([0 10 0 0.2338])
% 
