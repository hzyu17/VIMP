function output = plot_costs_WAM(costs, factor_costs, precisions, niters, n_states)
%% =============== plot cost for each factor and the total cost ================
fixed_prior_costs = [factor_costs(1:end, 1), factor_costs(1:end, end)];
prior_costs = [];
for i = 1:n_states-1
    prior_costs = [prior_costs, factor_costs(1:end, 1+(i-1)*2+1)];
end
obs_costs = [];
for i = 1:n_states-2
    obs_costs = [obs_costs, factor_costs(1:end, 1+(i-1)*2+2)];
end

x0 = 50;
y0 = 50;
width = 1000;
height = 500;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 3, 'TileSpacing', 'tight', 'Padding', 'tight') 
nexttile

t = title('Factored Prior Costs');
t.FontSize = 16;
hold on
grid on
plot(prior_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), prior_costs(1:niters, 1:end), 30, 'filled')
xl = xlabel('Iterations','fontweight','bold');
xl.FontSize = 16;
yl = ylabel('-log(p(x_k))','fontweight','bold');
yl.FontSize = 16;

nexttile
t = title('Factored Collision Costs');
t.FontSize = 16;
hold on
grid on
plot(obs_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), obs_costs(1:niters, 1:end), 30, 'filled')
xl = xlabel('Iterations','fontweight','bold');
xl.FontSize = 16;
yl = ylabel('-log(p(z|x_k))','fontweight','bold');
yl.FontSize = 16;

% --- entropy
entropy_costs = [];
n_dim = size(precisions, 2);
for i = 1:niters
    precision_i  = precisions((i-1)*n_dim+1: i*n_dim, 1:end);
    entropy_costs = [entropy_costs, log(det(precision_i))/2];
end

nexttile
t = title('Entropy Cost');
t.FontSize = 16;
hold on
grid on
plot(entropy_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), entropy_costs(1:niters), 30, 'filled')
xl = xlabel('Iterations', 'fontweight', 'bold');
xl.FontSize = 16;
yl = ylabel('log(|\Sigma^{-1}|)/2', 'Interpreter', 'tex', 'fontweight', 'bold');
yl.FontSize = 16;

% verify that the sum of the factored costs is the same as the total cost
sum_fact_costs = sum(factor_costs(1:niters, 1:end), 2);
diff = sum_fact_costs + entropy_costs' - costs(1:niters);

% ================ plot the total costs ================
nexttile([1 3])
t = title('Total Cost');
t.FontSize = 16;
grid on 
hold on
plot(costs(1:niters), 'LineWidth', 2.0);
scatter(linspace(1, niters, niters), costs(1:niters), 30, 'fill')
xl = xlabel('Iterations','fontweight','bold');
xl.FontSize = 16;
yl = ylabel('V(q)','fontweight','bold');
yl.FontSize = 16;
hold off

output = true;
end