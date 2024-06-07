function output = plot_costs(costs, factor_costs, precisions, niters, n_states, dim_state)
%% =============== plot cost for each factor and the total cost ================
fixed_prior_costs = [factor_costs(1, 1:end), factor_costs(end, 1:end)];

prior_costs = [];
for i = 1:n_states-1
    prior_costs = [prior_costs; factor_costs(1+(i-1)*2+1, 1:end)]; % in shape: (n_prior_costs, niters)
end
obs_costs = [];
for i = 1:n_states-2
    obs_costs = [obs_costs, factor_costs(1+(i-1)*2+2, 1:end)'];
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
plot(prior_costs', 'LineWidth', 1.5) % in shape: (n_prior_costs, niters)
for j = 1:size(prior_costs, 1)
    scatter(linspace(1, niters, niters), prior_costs(j, 1:niters), 30, 'filled')
end
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
for j = 1:size(obs_costs, 2)
    scatter(linspace(1, niters, niters), obs_costs(1:niters, j), 30, 'filled')
end
xl = xlabel('Iterations','fontweight','bold');
xl.FontSize = 16;
yl = ylabel('-log(p(z|x_k))','fontweight','bold');
yl.FontSize = 16;

% --------- 
% entropy
% ---------
entropy_costs = [];
% n_dim = size(precisions, 2);
for i_iter = 1:niters
%     precision_i  = precisions((i-1)*n_dim+1: i*n_dim, 1:end);
    precision_i  = reshape(precisions(:, i_iter), [n_states*dim_state, n_states*dim_state]);
    eig_val_i = eig(precision_i);
    entropy_costs = [entropy_costs; sum(log(eig_val_i))/2];
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
sum_fact_costs = sum(factor_costs, 1)';
disp('verify that the sum of the factored costs is the same as the total cost')
diff = sum(sum_fact_costs + entropy_costs - costs)

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