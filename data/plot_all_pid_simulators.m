function [] = plot_all_pid_simulators()

for i = 1:3
    plot_pid_simulator(strcat(...
              'C:\Users\sarvp\CLionProjects\Controls\data\controller_', ...
               num2str(i), '.csv'), i);
end