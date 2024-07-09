function [] = plotting_one_experiment(sdfmap_file, i_exp, img_name, visible)
% read map
sdfmap = csvread(sdfmap_file);

x0 = 300;
y0 = 500;
width = 600;
height = 600;
figure('Visible', visible)
set(gcf,'position',[x0,y0,width,height])

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')

nexttile
hold on
prefix = ["map2/case"+num2str(i_exp)+"/"];

% % --- read means and covariances ---
means = csvread([prefix + "zk_sdf.csv"]);
covs = csvread([prefix + "Sk_sdf.csv"]);

plot_2d_result(sdfmap, means, covs);

xlim([-20, 25]);
ylim([-15, 22]);

saveas(gcf, img_name, 'png');

end