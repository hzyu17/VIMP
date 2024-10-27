clear; clc; close all;

x0 = 500;
y0 = 500;
width = 1200;
height = 900;

%% GH Degree = 4
n_states = [10, 25, 50, 100, 150, 200, 300, 400, 500, 750, 1000];

% GPU times
GPU_avg = [2.42, 5.16, 6.44, 14.16, 21.34, 30.42, 52.90, 78.30, 111.64, 233.52, 374.50];
GPU_min = [2, 3, 5, 12, 19, 28, 48, 75, 105, 222, 354];
GPU_max = [4, 11, 9, 18, 32, 37, 61, 86, 119, 253, 416];

% CPU times
CPU_avg = [4.80, 13.50, 27.52, 54.16, 83.96, 116.82, 180.18, 254.50, 335.34, 650.88, 965.26];
CPU_min = [3, 10, 17, 40, 70, 105, 166, 231, 308, 590, 932];
CPU_max = [8, 17, 33, 74, 100, 133, 201, 279, 360, 708, 1035];

figure;
set(gca, 'FontName', 'Sans Serif', 'FontSize', 28)
set(gcf, 'position',[x0,y0,width,height])

hold on;

plot(n_states, GPU_avg, 'g-o', 'MarkerSize', 14, 'linewidth', 2.5, 'DisplayName', 'GPU Mean GH degree = 4');
fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

plot(n_states, CPU_avg, 'r-o', 'MarkerSize', 14, 'linewidth', 2.5, 'DisplayName', 'CPU Mean GH degree = 4');
fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');


%% GH Degree = 10
% GPU times
GPU_avg = [3.6, 8.86, 17.82, 36.54, 57.6, 80.88, 130.76, 185.44, 234.6, 440.68, 679.96];
GPU_min = [3, 7, 15, 32, 51, 73, 120, 164, 212, 405, 654];
GPU_max = [5, 13, 26, 54, 77, 101, 153, 214, 290, 556, 845];

% CPU times
CPU_avg = [23.2, 53.42, 100.08, 183.82, 277, 368.20, 566.94, 759.24, 957.36, 1547.14, 2138.18];
CPU_min = [19, 45, 88, 157, 239, 344, 536, 723, 909, 1422, 2055];
CPU_max = [30, 63, 123, 229, 316, 431, 619, 836, 1039, 1646, 2356];

ratio = (CPU_avg-GPU_avg)./CPU_avg

plot(n_states, GPU_avg, 'g--x', 'MarkerSize', 18, 'linewidth', 2.5, 'DisplayName', 'GPU Mean GH degree = 10');
fill([n_states, fliplr(n_states)], [GPU_min, fliplr(GPU_max)], 'g', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');

plot(n_states, CPU_avg, 'r--x', 'MarkerSize', 18, 'linewidth', 2.5, 'DisplayName', 'CPU Mean GH degree = 10');
fill([n_states, fliplr(n_states)], [CPU_min, fliplr(CPU_max)], 'r', 'FaceAlpha', 0.2, 'HandleVisibility', 'off');


%% GH Degree = 20
n_states = [10, 25, 50, 100, 150, 200, 300, 400, 500, 1000];

% GPU times
GPU_avg = [18.06, 50.46, 91.86, 193.62, 281.2, 380.68, 612.58, 821.02, 1044.48, 2420.93];
GPU_min = [14, 43, 87, 180, 269, 366, 585, 799, 1017, 2365];
GPU_max = [28, 74, 119, 217, 315, 431, 706, 839, 1092, 2519];

% CPU
CPU_avg = [110.4, 265.8, 527.42, 1052.1, 1556.34, 2103.80, 3150.78, 4198.55, 5283.55, 10764.21];
CPU_min = [87, 242, 488, 989, 1510, 2010, 3043, 4088, 5173, 10601];
CPU_max = [117, 292, 554, 1108, 1614, 2164, 3272, 4322, 5398, 10938];

ratio = (CPU_avg-GPU_avg)./CPU_avg


xlabel("Time Discretizations")
ylabel("Computation Time (ms)")
legend('show', 'Location', 'northwest');
grid minor

% exportgraphics(gcf, '2dpR_1.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');