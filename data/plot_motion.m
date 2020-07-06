function [] = plot_motion(filename, fig_handle)

try
    M      = csvread(filename);
catch
    warning(strcat(filename,' not found'));
    return;
end
    
    ref   = M(1:end,1);
    pos   = M(1:end,2);
    vel   = M(1:end,3);
    time  = M(1:end,4);
    
    figure(fig_handle)
    ha(1) = subplot(2,1,1);
    plot(time, ref); hold on;
    plot(time, pos); hold on;
    legend('Ref','Act');
    xlabel('Time [s]');
    ylabel('Pos [m]');
    
    ha(2) = subplot(2,1,2);
    plot(time, vel);
    xlabel('Time [s]');
    ylabel('Vel [m/s]');

end