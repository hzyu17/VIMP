%*****************************************************************************
% nwSpGr_demo: test & demo for nwspgr
% integrate function using sparse grids and simulation
%*****************************************************************************

D = 10;    % dimensions
maxk = 4;  % max. accuracy level (pol. exactness wil be 2k-1)

% integrand: some function that evaluates g(x): (R times D)->(R times 1)
func = 'prod( exp(-(x/2).^2/2)/2/sqrt(2*pi), 2)';

% calculate "correct" result of integration between 0 and 1:
%trueval=(.5*(1+erf(1./sqrt(2)/2))-.5).^D;
trueval=(.5*erf(1./sqrt(2)/2)).^D;
% note: if unknown, replace it with simulated value:
%x=rand(1e+6,D);trueval=mean(eval(func));

for k=2:maxk
    % sparse grids integration:
    [x w] = nwspgr('GQN', D, k);
  	g = eval(func);
    SGappr = g'*w;
    SGerror = sqrt((SGappr - trueval).^2)/trueval;
    % simulation with the same number of nodes, 1000 simulation repetitions
    numnodes = length(w);
    sim = zeros(1000,1);
    for r=1:1000
        x = rand(numnodes,D);
        g = eval(func);
        sim(r) = mean(g);
    end
    Simerror = sqrt(mean((sim-trueval).^2))/trueval;
    fprintf('D=%2.0f, k=%2.0f (nodes=%4.0f): SG error=%8.5f, Sim. error=%8.5f\n', ...
        [D,k,numnodes,SGerror,Simerror] )
end

