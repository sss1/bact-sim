clear;
% this file sets global parameters, runs basic_swarm, and saves the results
% run this file with the appropriate parameter values to run the simulation

% true location of the food source
% TODO: should move food to [0 0], but need to make sure to change this everywhere
source = [3 2];

% some helper functions for terrains
nf = @(x,y) sqrt(x.^2 + y.^2); % Euclidean norm in R^2
obstacles = @(x,y) min(0,-4.*cos(pi.*x./3).*cos(pi.*y./3) + 0.5); % half-cosine grid
logSource = @(x,y) min(2.5,-2.*log(nf(x - 3,y - 2).^2)); % logarithmic food source; truncate near 0 for sake of plotting
obstacle = @(x,y) min(0,-7.5 + 50.*nf((x - 0.3)./3.2, y + 2).^2); % single parabolic obstacle
% besselSource = @(x,y) besselk(0,nf(x,y)); % modified bessel food source of the second kind
% banana = @(x,y,a) -3.*(x - a).^2 - 100.*(10.*(y + 1) - (x).^2).^2 % Rosenbrock's Banana Function, centered at (a,a^2 - 1)

%%% OBJECTIVE FUNCTION %%%
% -log food source with obstacles
% terrain = @(x,y) obstacle(x - 1,y - x + 1) + obstacles(x,y) + logSource(x,y); % easy version
sargs.terrain = @(x,y) logSource(x,y) + obstacles(x,y); % + obstacle(x,y);
% % -bessel food source with obstacles
% % Rosenbrock's Banana function, centered at [3 2]
% terrain = banana(x,y,3);

% sargs specifies simulation properties
% i.e., global properties
sargs.n = 30;                                    % number of agents per swarm
sargs.Ns = 1;                                    % numbers of swarms
% sargs.X(:,1) = unifrnd(-7, -6, sargs.n, 1);     % initial agent X positions
% sargs.X(:,2) = unifrnd(-9, -8, sargs.n, 1);     % initial agent Y positions

sargs.dt = 0.5;	                                % time step size
sargs.num_iters = 5000;                         % number of iterations to simulate (set very large (e.g., 5000) to measure path length)
sargs.to_plot = true;                          % whether to plot simulation in real time
sargs.to_record = false;                        % whether to save a video of the simulation plot; only used if sargs.to_plot
sargs.record_name = 'single_agent';             % name of video file (without '.avi'); only used if sargs.to_record
sargs.found_radius = 0.9;                       % distance from food source at which to terminate search (-1 if never)
% sargs.distance_func = @(X,c) norm(mean(X) - c); % distance function from food source c; distance of mean
sargs.distance_func = @(X,c) (sum(sqrt(sum(bsxfun(@minus,X,c).^2,2)) > 0.5) > sargs.n/2); % whether half the agents have found food; for "median" path length

num_trials = 100;

% list presets to compare
bargs(1) = preset('dwexp');
% bargs(2) = preset('norm_comm');
% bargs(3) = preset('no_orient');
% bargs(4) = preset('no_comm');

% allocate space for outputs
lengths = zeros(num_trials, length(bargs), length(sargs.Ns));
% path_dists = zeros(num_trials, length(bargs), sargs.num_iters);
% inter_dists = zeros(num_trials, length(bargs), sargs.num_iters, length(pdist(sargs.X)));
% Vs = zeros(num_trials, length(bargs), sargs.num_iters, sargs.n);

silent_fracs = [0 0.4 0.6 0.8 0.9 1];
sargs.blind_frac = 0;

% run each method
for method = 1:length(bargs)

  for si=1:length(silent_fracs)
    sargs.silent_frac = silent_fracs(si);

    for Ni=1:length(sargs.Ns)
      N = sargs.Ns(Ni);

        sargs.blind = unifrnd(0,1,sargs.n*N,1) < sargs.blind_frac;
        sargs.silent = unifrnd(0,1,sargs.n*N,1) < sargs.silent_frac;

      % run trials
      for trial = 1:num_trials

        X0 = zeros(0,2);
        for i=1:N % randomly place swarms around food source
          theta = unifrnd(0,2*pi);
          X_min = floor(20*cos(theta)) + source(1) - 0.5;
          Y_min = floor(20*sin(theta)) + source(2) - 0.5;
          new_X = [unifrnd(X_min, X_min + 3, sargs.n, 1) unifrnd(Y_min, Y_min + 3, sargs.n, 1)];
          X0 = [X0; new_X];
        end

        % if trial <= 10
          [method, si, trial] % report progress
        % end
        [lengths(trial,method,si)] = basic_swarm(bargs(method), sargs, X0);
        % [lengths(trial,method), path_dists(trial,method,:), inter_dists(trial,method,:,:), Vs(trial,method,:,:)] = basic_swarm(bargs(method), sargs);
      end
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
% save(['dwexpSilents' savenum '.mat'],'lengths','bargs','sargs');% ,'path_dists','inter_dists','Vs');
