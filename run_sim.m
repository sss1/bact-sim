clear;
% this file sets global parameters and then runs basic_swarm
% run this file with the appropraite parameter values to run the simulation

% some helper functions for terrains
nf = @(x,y) sqrt(x.^2 + y.^2); % Euclidean norm in R^2
obstacles = @(x,y) min(0,-4.*cos(pi.*x./3).*cos(pi.*y./3) + 0.5); % half-cosine grid
logSource = @(x,y) min(2.5,-2.*log(nf(x - 3,y - 2).^2)); % logarithmic food source; truncate near 0 for sake of plotting
obstacle = @(x,y) min(0,-7.5 + 50.*nf((x - 0.3)./3.2, y + 2).^2); % single parabolic obstacle
% besselSource = @(x,y) 2.*besselk(0,nf(x,y)); % modified bessel food source of the second kind
% banana = @(x,y,a) -3.*(x - a).^2 - 100.*(10.*(y + 1) - (x).^2).^2 % Rosenbrock's Banana Function, centered at (a,a^2 - 1)

%%% OBJECTIVE FUNCTION %%%
% -log food source with obstacles
% terrain = @(x,y) obstacle(x - 1,y - x + 1) + obstacles(x,y) + logSource(x,y); % easy version
sargs.terrain = @(x,y) obstacle(x,y) + obstacles(x,y) + logSource(x,y);
% % -bessel food source with obstacles
% terrain = @(x,y) besselSource(x,y) + min(0,1 - nf(x,y).*cos(pi.*x./3).*cos(pi.*y./3)) + min(0,-5 + 10.*(nf(x./2 + 0.3,y + 2)));
% % Rosenbrock's Banana function, centered at [3 2]
% terrain = banana(x,y,3);

% sargs specifies simulation properties
% i.e., global properties
sargs.n = 50;                                   % number of agents to simulate
% X0 = [unifrnd(-7.7,-6.7,n,1) unifrnd(-8.2,-7.2,n,1); unifrnd(2.5,3.5,n,1) unifrnd(1.5,2.5,n,1)];
sargs.X(:,1) = unifrnd(-7,-6,sargs.n,1);        % initial agent X positions
sargs.X(:,2) = unifrnd(-9,-8,sargs.n,1);        % initial agent Y positions
sargs.dt = 0.2;	                                % time step size
sargs.num_iters = 1500;                         % number of iterations to simulate (set very large (e.g., 5000) to measure path length)
sargs.to_plot = false;                          % whether to plot simulation in real time
sargs.found_radius = 0.1;                       % distance from food source at which to terminate search (0 if never)
sargs.distance_func = @(X,c) norm(mean(X) - c); % how to determine the distance from food source c

% code for plotting mean path
% mean_path = basic_swarm(X0, terrain, sigma, RR, RO, RA, dt, num_iters, to_plot, method);
% xmin = -3;
% xmax = 7;
% ymin = -4;
% ymax = 6;
% ezcontour(terrain, [xmin,xmax,ymin,ymax], 250);
% hold all;
% plot(mean_path(:,1),mean_path(:,2));

% code for visualizing sim
sargs.to_plot = true;
basic_swarm(preset('shklarsh'), sargs);

% % code for plotting distribution of path lengths and path lengths over time
% num_trials = 300;
% lengths1 = zeros(num_trials,1);
% lengths2 = zeros(num_trials,1);
% lengths3 = zeros(num_trials,1);
% lengths4 = zeros(num_trials,1);
% dists1 = zeros(num_trials,num_iters);
% dists2 = zeros(num_trials,num_iters);
% dists3 = zeros(num_trials,num_iters);
% dists4 = zeros(num_trials,num_iters);
% parfor trial = 1:num_trials
%   if trial <= num_trials/6
%     trial
%   end
%   [lengths1(trial), dists1(trial,:)] = basic_swarm(bargs, sargs);
%   [lengths2(trial), dists2(trial,:)] = basic_swarm(X0, terrain, sigma, RR, RO, RA, dt, num_iters, to_plot, 'ShklarshW');
%   [lengths3(trial), dists3(trial,:)] = basic_swarm(X0, terrain, sigma, RR, RO, RA, dt, num_iters, to_plot, 'ShklarshD');
%   [lengths4(trial), dists4(trial,:)] = basic_swarm(X0, terrain, sigma, RR, RO, RA, dt, num_iters, to_plot, 'ShklarshDW');
% end
% 
% % Plot distribution of path lengths
% figure;
% hold all;
% [f,xi] = ksdensity(lengths1);
% plot(xi,f,'LineWidth',3);
% [f,xi] = ksdensity(lengths2);
% plot(xi,f,'LineWidth',3);
% [f,xi] = ksdensity(lengths3);
% plot(xi,f,'LineWidth',3);
% [f,xi] = ksdensity(lengths4);
% plot(xi,f,'LineWidth',3);
% xlim([200 1600]);
% h = legend('Shklarsh','Weighted','Discrete','Weighted and Discrete');
% set(h,'FontSize',20);
% xlabel('Path Length (Iterations)','FontSize',20);
% ylabel('Probability','FontSize',20);
% title('Distribution of Path Lengths for First Swarm under Each Model','FontSize',20);
% 
% % Plot path lengths over time
% figure;
% hold all;
% m = mean(dists1(:,1:1500));
% plot(m,'LineWidth',3);
% m = mean(dists2(:,1:1500));
% plot(m,'LineWidth',3);
% m = mean(dists3(:,1:1500));
% plot(m,'LineWidth',3);
% m = mean(dists4(:,1:1500));
% plot(m,'LineWidth',3);
% h = legend('Shklarsh','Weighted','Discrete','Weighted and Discrete');
% set(h,'FontSize',20);
% xlabel('Iteration','FontSize',20);
% ylabel('Distance from food source','FontSize',20);
% title('Mean Distance from Food Source over Time','FontSize',20);
% 
% % save results
% save('results_swarm2_final.mat','lengths1','lengths2','lengths3','lengths4','dists1','dists2','dists3','dists4');
