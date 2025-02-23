clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1200;
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

%% 2dpR GH Degree = 4 (Optimize Data)
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU Optimize times (in s)
GPU_avg = [27.4, 48.95, 73.75, 136.75, 240.1, 456.9, 1528.6, 2624.75] / 1000;
GPU_min = [24, 43, 69, 132, 231, 424, 1500, 2588] / 1000;
GPU_max = [34, 56, 81, 141, 252, 484, 1552, 2667] / 1000;

% CPU Optimize times (in s)
CPU_avg = [41.65, 104.6, 329.55, 770.1, 1370.25, 3104.55, 8346.35, 14539] / 1000;
CPU_min = [41, 96, 304, 747, 1311, 3060, 8226, 14386] / 1000;
CPU_max = [44, 127, 359, 791, 1467, 3223, 8481, 14741] / 1000;

gpu1 = plot(n_states, GPU_avg, '-o', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'g', ...
    'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 4$)');
fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

cpu1 = plot(n_states, CPU_avg, '-o', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'r', ...
    'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 4$)');
fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

%% 2dpR GH Degree = 10 (Optimize Data)
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU Optimize times (in s)
GPU_avg = [48.05, 48.1, 89.4, 174.75, 261.9, 491.15, 1659.05, 2668.3] / 1000;
GPU_min = [39, 42, 80, 165, 253, 466, 1594, 2623] / 1000;
GPU_max = [56, 56, 98, 197, 272, 564, 1714, 2705] / 1000;

% CPU Optimize times (in s)
CPU_avg = [103.3, 327.2, 514.25, 1297.55, 2128.5, 4413.05, 10477.9, 16733] / 1000;
CPU_min = [99, 301, 495, 1239, 2084, 4325, 10278, 16487] / 1000;
CPU_max = [109, 373, 533, 1547, 2210, 4680, 10744, 16959] / 1000;

gpu2 = plot(n_states, GPU_avg, '--x', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'g', ...
    'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 10$)');
fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu2 = plot(n_states, CPU_avg, '--x', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'r', ...
    'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 10$)');
fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

%% 2dpR GH Degree = 20 (Optimize Data)
n_states = [25, 50, 100, 200, 300, 500, 750, 1000];

% GPU Optimize times (in s)
GPU_avg = [57.95, 76.1, 131.65, 237.35, 352.2, 714.85, 2087.75, 3081.75] / 1000;
GPU_min = [53, 69, 121, 222, 339, 657, 1989, 3045] / 1000;
GPU_max = [66, 82, 140, 257, 365, 761, 2142, 3127] / 1000;

% CPU Optimize times (in s)
CPU_avg = [589.95, 1172.45, 2361.75, 4805.4, 8112.8, 12542.7, 22534.6, 33110.5] / 1000;
CPU_min = [564, 1124, 2291, 4709, 7904, 12359, 22282, 32677] / 1000;
CPU_max = [622, 1271, 2608, 4898, 8358, 13011, 22679, 33573] / 1000;

gpu3 = plot(n_states, GPU_avg, '-.d', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'g', ...
    'MarkerEdgeColor', [0, 0.35, 0], 'DisplayName', 'GPU ($k_q = 20$)');
fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

% cpu3 = plot(n_states, CPU_avg, '-.d', 'MarkerSize', 14, 'LineWidth', 2.5, 'Color', 'r', ...
%     'MarkerEdgeColor', [0.35, 0, 0], 'DisplayName', 'CPU ($k_q = 20$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

xlabel("Time Discretizations")
ylabel("Optimization Time (s)")
% legend([gpu1 gpu2 gpu3 cpu1 cpu2 cpu3], 'Location', 'northwest');
legend([gpu1 gpu2 gpu3 cpu1 cpu2], 'Location', 'northwest');
grid minor

% exportgraphics(gcf, '2dpR_Optimization_2.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');
