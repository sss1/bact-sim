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
sargs.terrain = @(x,y) logSource(x,y) + obstacles(x,y) + obstacle(x,y);
% % -bessel food source with obstacles
% terrain = @(x,y) besselSource(x,y) + min(0,1 - nf(x,y).*cos(pi.*x./3).*cos(pi.*y./3)) + min(0,-5 + 10.*(nf(x./2 + 0.3,y + 2)));
% % Rosenbrock's Banana function, centered at [3 2]
% terrain = banana(x,y,3);

% sargs specifies simulation properties
% i.e., global properties
sargs.n = 30;                                   % number of agents to simulate
% X0 = [unifrnd(-7.7,-6.7,n,1) unifrnd(-8.2,-7.2,n,1); unifrnd(2.5,3.5,n,1) unifrnd(1.5,2.5,n,1)];
sargs.X(:,1) = unifrnd(-7, -6, sargs.n, 1);     % initial agent X positions
sargs.X(:,2) = unifrnd(-9, -8, sargs.n, 1);     % initial agent Y positions
sargs.dt = 0.2;	                                % time step size
sargs.num_iters = 1600;                         % number of iterations to simulate (set very large (e.g., 5000) to measure path length)
sargs.to_plot = false;                          % whether to plot simulation in real time
sargs.to_record = false;                        % whether to save a video of the simulation plot; only used if sargs.to_plot
sargs.record_name = 'refactored';               % name of video file (without '.avi'); only used if sargs.to_record
sargs.found_radius = 0.9;                       % distance from food source at which to terminate search (0 if never)
% sargs.distance_func = @(X,c) norm(mean(X) - c); % how to determine the distance from food source c; distance of mean
sargs.distance_func = @(X,c) (sum(sqrt(sum(bsxfun(@minus,X,c).^2,2)) > sargs.found_radius) > sargs.n/2); % whether half the agents have found food

% code for plotting mean path
% mean_path = basic_swarm(X0, terrain, sigma, RR, RO, RA, dt, sargs.n, to_plot, method);
% xmin = -3;
% xmax = 7;
% ymin = -4;
% ymax = 6;
% ezcontour(terrain, [xmin,xmax,ymin,ymax], 250);
% hold all;
% plot(mean_path(:,1),mean_path(:,2));

% code for visualizing sim
% sargs.to_plot = true;
% basic_swarm(preset('adaptive'), sargs);

% code for plotting distribution of path lengths and path lengths over time
num_trials = 30;

% list presets to compare
bargs(1) = preset('dwexp');

% allocate space for outputs
lengths = zeros(num_trials, length(bargs));
path_dists = zeros(num_trials, length(bargs), sargs.num_iters);
inter_dists = zeros(num_trials, length(bargs), sargs.num_iters, length(pdist(sargs.X)));
Vs = zeros(num_trials, length(bargs), sargs.num_iters, sargs.n);

% run trials
for trial = 1:num_trials
  % run each method
  for method = 1:length(bargs)
    [trial, method] % report progress
    [lengths(trial,method)] = basic_swarm(bargs(method), sargs);
    % [lengths(trial,method), path_dists(trial,method,:), inter_dists(trial,method,:,:), Vs(trial,method,:,:)] = basic_swarm(bargs(method), sargs);
  end
end

% save results
if sargs.n < 10
  savenum = ['00' int2str(sargs.n)];
elseif sargs.n < 100
  savenum = ['0' int2str(sargs.n)];
else
  savenum = int2str(sargs.n);
end
save(['wd' savenum '.mat'],'lengths','path_dists','inter_dists','Vs');
