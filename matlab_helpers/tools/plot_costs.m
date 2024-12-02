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
height = 700;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact') 
nexttile

t = title('Factored Prior Costs', 'Interpreter', 'latex', 'FontSize', 20);
hold on
grid on
plot(prior_costs', 'LineWidth', 1.5) % in shape: (n_prior_costs, niters)
for j = 1:size(prior_costs, 1)
    scatter(linspace(1, niters, niters), prior_costs(j, 1:niters), 30, 'filled')
end
xlabel('Iterations','Interpreter', 'latex', 'FontSize', 20);
ylabel('$-\log(p(x_k))$','Interpreter', 'latex', 'FontSize', 20);
xlim([0, niters])

nexttile
t = title('Factored Collision Costs', 'Interpreter', 'latex', 'FontSize', 20);
hold on
grid on
plot(obs_costs, 'LineWidth', 1.5)
for j = 1:size(obs_costs, 2)
    scatter(linspace(1, niters, niters), obs_costs(1:niters, j), 30, 'filled')
end
xlabel('Iterations','Interpreter', 'latex', 'FontSize', 20);
ylabel('$-\log(p(z|x_k))$', 'Interpreter', 'latex', 'FontSize', 20);
xlim([0, niters])

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
t = title('Entropy Cost', 'Interpreter', 'latex', 'FontSize', 20);
hold on
grid on
plot(entropy_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), entropy_costs(1:niters), 30, 'filled')
xlabel('Iterations', 'Interpreter', 'latex', 'FontSize', 20);
ylabel('$\log(|\Sigma^{-1}_{\theta}|)/2$', 'Interpreter', 'latex', 'FontSize', 20);
xlim([0, niters])

% verify that the sum of the factored costs is the same as the total cost
sum_fact_costs = sum(factor_costs, 1)';
disp('verify that the sum of the factored costs is the same as the total cost')
% diff = sum(sum_fact_costs + entropy_costs - costs)

% ================ plot the total costs ================
nexttile([1 3])
t = title('Total Cost', 'Interpreter', 'latex', 'FontSize', 20);
grid on 
hold on
plot(costs(1:niters), 'LineWidth', 2.0);
scatter(linspace(1, niters, niters), costs(1:niters), 30, 'fill')
xlabel('Iterations', 'Interpreter', 'latex', 'FontSize', 20);
ylabel('$V(q)$', 'Interpreter', 'latex', 'FontSize', 20);
hold off

output = true;
end