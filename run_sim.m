clear;
% this file sets global parameters, runs basic_swarm, and saves the results
% run this file with the appropriate parameter values to run the simulation

% true location of the food source
source = [3 2];

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
sargs.n = 30;                                   % number of agents per swarm
sargs.Ns = [2 4];                               % numbers of swarms
% sargs.X(:,1) = unifrnd(-7, -6, sargs.n, 1);     % initial agent X positions
% sargs.X(:,2) = unifrnd(-9, -8, sargs.n, 1);     % initial agent Y positions

sargs.dt = 0.2;	                                % time step size
sargs.num_iters = 1600;                         % number of iterations to simulate (set very large (e.g., 5000) to measure path length)
sargs.to_plot = false;                          % whether to plot simulation in real time
sargs.to_record = false;                        % whether to save a video of the simulation plot; only used if sargs.to_plot
sargs.record_name = 'refactored';               % name of video file (without '.avi'); only used if sargs.to_record
sargs.found_radius = 0.9;                       % distance from food source at which to terminate search (0 if never)
% sargs.distance_func = @(X,c) norm(mean(X) - c); % how to determine the distance from food source c; distance of mean
sargs.distance_func = @(X,c) (sum(sqrt(sum(bsxfun(@minus,X,c).^2,2)) > sargs.found_radius) > sargs.n/2); % whether half the agents have found food; for "median" path length

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
num_trials = 120;

% list presets to compare
bargs(1) = preset('dwexp');
bargs(2) = preset('norm_comm');
bargs(3) = preset('no_orient');
bargs(4) = preset('no_comm');

% allocate space for outputs
lengths = zeros(num_trials, length(bargs), length(sargs.Ns));
% path_dists = zeros(num_trials, length(bargs), sargs.num_iters);
% inter_dists = zeros(num_trials, length(bargs), sargs.num_iters, length(pdist(sargs.X)));
% Vs = zeros(num_trials, length(bargs), sargs.num_iters, sargs.n);

% run each method
for method = 1:length(bargs)
  for Ni=1:length(sargs.Ns)
    N = sargs.Ns(Ni);
    % run trials
    parfor trial = 1:num_trials

      X0 = zeros(0,2);
      for i=1:N % randomly place swarms around food source
        theta = unifrnd(0,2*pi);
        X_min = floor(15*cos(theta)) + source(1) - 0.5;
        Y_min = floor(15*sin(theta)) + source(2) - 0.5;
        new_X = [unifrnd(X_min, X_min + 1, sargs.n, 1) unifrnd(Y_min, Y_min + 1, sargs.n, 1)];
        X0 = [X0; new_X];
      end

      if trial <= 10
        [method, N, trial] % report progress
      end
      [lengths(trial,method,Ni)] = basic_swarm(bargs(method), sargs, X0);
      % [lengths(trial,method), path_dists(trial,method,:), inter_dists(trial,method,:,:), Vs(trial,method,:,:)] = basic_swarm(bargs(method), sargs);
    end
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
save(['isoDisc2to4Multi' savenum '.mat'],'lengths','bargs','sargs');% ,'path_dists','inter_dists','Vs');
