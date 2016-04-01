function plotTheoTimeDelay(p, d_LOS, d_MD, d_SD)

    figure; hold on
        % Easy solution to get the legend right
        plot(squeeze(d_MD(1,1,:))/300, p.T, 'b--', 'LineWidth', 1.5)
        plot(squeeze(d_SD(1,1,:))/300, p.T, 'm-.', 'LineWidth', 1.5)
        plot(d_LOS/300, p.T, 'r-','linewidth',1.5)
        
        % Plot the rest
        plot(squeeze(d_MD(1,2:end,:))/300, p.T, 'b--', 'LineWidth', 1.5)
        plot(squeeze(d_SD(1,2:end,:))/300, p.T, 'm-.', 'LineWidth', 1.5)

        % Set some axes settings
        set(gca, ...
            'FontSize', 14, ...
            'FontName', 'Times', ...
            'LineWidth', 1 ...
            )
        xlabel('Delay [\mu s]')
        ylabel('Time [s]')
        grid on
        title('Theoretic time-delay plot')
        legend('MD', 'SD', 'LOS')
        axis([0 10 0 0.2338])

    drawnow