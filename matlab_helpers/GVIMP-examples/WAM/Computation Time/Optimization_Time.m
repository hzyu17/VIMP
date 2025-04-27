clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1600;
height = 900;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

figure;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 32)
% set(gcf, 'position',[x0,y0,width,height])

set(gcf, 'Units', 'inches', 'Position', [x0/100 y0/100 width/100 height/100]);
set(gcf, 'PaperUnits', 'inches', 'PaperSize', [width/100, height/100]);
set(gcf, 'PaperPositionMode', 'auto');

hold on;

%% WAM GH Degree = 3
n_states = [25, 50, 100, 200, 300, 500, 750];

% GPU times (in ms)
GPU_avg = [76.8, 138.4, 270.4, 527.2, 951.8, 1762.6, 4059.8] / 1000;
GPU_min = [73, 137, 250, 523, 819, 1781, 3431];
GPU_max = [90, 143, 304, 544, 1004, 1898, 4436];

% CPU times (in ms)
CPU_avg = [684.2, 1442.2, 3806, 13580.2, 28218.4, 75812.8, 161814] / 1000;
CPU_min = [671, 1415, 3759, 13490, 27787, 75058, 161084];
CPU_max = [701, 1471, 3846, 13709, 28445, 77204, 162685];

% ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu1 = plot(n_states, GPU_avg, '-o', 'MarkerSize', 18, 'LineWidth', 3, 'Color', '[0, 0.5, 0]', ...
    'MarkerFaceColor', '[0.5, 1, 0.5]', 'MarkerEdgeColor', [0, 0.4, 0], 'DisplayName', 'GPU ($k_q = 3$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');

cpu1 = plot(n_states, CPU_avg, '-o', 'MarkerSize', 18, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerFaceColor', '[1, 0.75, 0.8]', 'MarkerEdgeColor', [0.4, 0, 0], 'DisplayName', 'CPU ($k_q = 3$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.15, 'HandleVisibility', 'off');


%% WAM GH Degree = 5
% Using the same discretization points
n_states = [25, 50, 100, 200, 300, 500, 750];

% GPU times (in ms)
GPU_avg = [177.4, 314.8, 630.6, 1356.2, 2322.6, 3388.4, 5765.4] / 1000;
GPU_min = [178, 304, 628, 1476, 2059, 3399, 5630];
GPU_max = [178, 336, 644, 1361, 3016, 3404, 6036];

% CPU times (in ms)
CPU_avg = [3445.2, 6740.4, 14240.8, 33842.8, 58269, 124961, 225992] / 1000;
CPU_min = [3385, 6700, 14211, 33720, 58022, 121466, 223888];
CPU_max = [3513, 6794, 14280, 33967, 58608, 126226, 231218];

% ratio = (CPU_avg-GPU_avg)./CPU_avg

gpu2 = plot(n_states, GPU_avg, '--x', 'MarkerSize', 26, 'LineWidth', 3, 'Color', '[0, 0.5, 0]', ...
    'MarkerEdgeColor', [0, 0.4, 0], 'DisplayName', 'GPU ($k_q = 5$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu2 = plot(n_states, CPU_avg, '--x', 'MarkerSize', 26, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerEdgeColor', [0.4, 0, 0], 'DisplayName', 'CPU ($k_q = 5$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');


%% WAM GH Degree = 6
n_states = [25, 50, 100, 200, 300, 500, 750];

% GPU times (in ms)
GPU_avg = [505.6, 840.8, 1813.2, 3289.8, 4815.6, 9038.6, 12258.6] / 1000;
GPU_min = [490, 876, 1922, 3403, 4764, 8717, 12205];
GPU_max = [538, 881, 1945, 3383, 5082, 9834, 12309];

% CPU times (in ms)
CPU_avg = [1401.2, 20139.6, 40476.8, 85363, 134789, 254750, 430853] / 1000;
CPU_min = [1335, 19951, 39926, 85106, 134447, 253972, 430853];
CPU_max = [1508, 20387, 41291, 85611, 135258, 255521, 430853];

% ratio = (CPU_avg-GPU_avg)./CPU_avg

% Uncomment the following lines to plot the GH Degree = 20 curves
gpu3 = plot(n_states, GPU_avg, ':s', 'MarkerSize', 18, 'LineWidth', 3, 'Color', '[0, 0.5, 0]', ...
    'MarkerFaceColor', [0.5, 1, 0.5], 'MarkerEdgeColor', [0, 0.4, 0], 'DisplayName', 'GPU ($k_q = 6$)');
% fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

cpu3 = plot(n_states, CPU_avg, ':s', 'MarkerSize', 18, 'LineWidth', 3, 'Color', 'r', ...
    'MarkerFaceColor', [1, 0.75, 0.8], 'MarkerEdgeColor', [0.4, 0, 0], 'DisplayName', 'CPU ($k_q = 6$)');
% fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

xlabel("Time Discretizations")
ylabel("Cost Evaluation Time (s)")
legend('show', 'Location', 'northwest');
% legend([gpu1 gpu2 gpu3 cpu1 cpu2], 'Location', 'northwest');
grid minor

% set(gcf, 'Toolbar', 'none')
exportgraphics(gcf, 'WAM_Optimization_Comparison.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');
