clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1600;
height = 900;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

figure;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 28)
% set(gcf, 'position',[x0,y0,width,height])

set(gcf, 'Units', 'inches', 'Position', [x0/100 y0/100 width/100 height/100]);
set(gcf, 'PaperUnits', 'inches', 'PaperSize', [width/100, height/100]);
set(gcf, 'PaperPositionMode', 'auto');

hold on;

%% 2dpR GH Degree = 4
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU times (in ms)
GPU_avg = [0.05, 0.3, 1, 1.55, 3.45, 6.8, 13.25, 20.15];
GPU_min = [0, 0, 0, 1, 3, 6, 11, 18];
GPU_max = [1, 1, 2, 4, 5, 8, 15, 22];

% CPU times (in ms)
CPU_avg = [0.3, 1.25, 3.55, 8.65, 16.65, 31.75, 96.35, 160.6];
CPU_min = [0, 1, 3, 8, 14, 29, 92, 156];
CPU_max = [1, 3, 4, 9, 22, 37, 102, 166];

ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu1 = plot(n_states, GPU_avg, '-o', 'MarkerSize', 14, 'LineWidth', 3, 'Color', 'g', ...
    'MarkerFaceColor', '[0.5, 1, 0.5]', 'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 4$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

cpu1 = plot(n_states, CPU_avg, '-o', 'MarkerSize', 14, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerFaceColor', '[1, 0.75, 0.8]', 'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 4$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');


%% 2dpR GH Degree = 10
% Using the same discretization points
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU times (in ms)
GPU_avg = [0.4, 0.75, 2.2, 2.75, 4.6, 7.4, 14.8, 23.55];
GPU_min = [0, 0, 1, 2, 3, 6, 12, 21];
GPU_max = [1, 1, 3, 4, 6, 9, 18, 27];

% CPU times (in ms)
CPU_avg = [2.15, 4.2, 9.8, 22.35, 34.35, 64.45, 147.4, 211.9];
CPU_min = [2, 4, 8, 21, 31, 59, 142, 202];
CPU_max = [3, 5, 11, 27, 37, 72, 154, 222];

ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu2 = plot(n_states, GPU_avg, '--x', 'MarkerSize', 16, 'LineWidth', 3, 'Color', 'g', ...
    'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 10$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu2 = plot(n_states, CPU_avg, '--x', 'MarkerSize', 16, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 10$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');


%% 2dpR GH Degree = 20
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU times (in ms)
GPU_avg = [1.1, 1.4, 2.6, 4.7, 7.5, 13.95, 23.05, 31.65];
GPU_min = [1, 1, 2, 4, 7, 12, 19, 29];
GPU_max = [2, 2, 4, 7, 9, 16, 26, 34];

% CPU times (in ms)
CPU_avg = [13.55, 26.3, 58.6, 106.3, 163.25, 270.75, 442.55, 596.45];
CPU_min = [11, 23, 50, 102, 158, 258, 420, 586];
CPU_max = [17, 33, 69, 112, 179, 282, 464, 616];

ratio = (CPU_avg-GPU_avg)./CPU_avg

% Uncomment the following lines to plot the GH Degree = 20 curves
gpu3 = plot(n_states, GPU_avg, '-.d', 'MarkerSize', 14, 'LineWidth', 3, 'Color', 'g', ...
    'MarkerFaceColor', [0.5, 1, 0.5], 'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 20$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu3 = plot(n_states, CPU_avg, '-.d', 'MarkerSize', 14, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerFaceColor', [1, 0.75, 0.8], 'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 20$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

xlabel("Time Discretizations")
ylabel("Cost Evaluation Time (ms)")
legend([gpu1 gpu2 gpu3 cpu1 cpu2 cpu3], 'Location', 'northwest');
% legend([gpu1 gpu2 gpu3 cpu1 cpu2], 'Location', 'northwest');
grid minor

set(gcf, 'Toolbar', 'none')
% exportgraphics(gcf, '2dpR_1_updated.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');
