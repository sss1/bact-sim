% Inputs:
%   bargs - struct of local (bacteria) parameters; see preset.m
%   sargs - struct of global (experimental) parameters; see run_sim.m
%
% Outputs:
%  X - n-by-2 matrix of final agent positions
%  iter - number of iterations to find food
%  path_dist - distance from food source during each iteration

function [X, iter, path_dist] = basic_swarm(bargs, sargs)

  X = sargs.X;
  terrain = sargs.terrain;
  dt = sargs.dt;
  num_iters = sargs.num_iters;
  to_plot = sargs.to_plot;
  to_record = sargs.to_record;

  V = zeros(size(X)); % matrix of agent velocities; initially 0

  if to_plot
    plotter(X, terrain);
    F(1) = getframe;
    if to_record
      aviobj = avifile([sargs.record_name '.avi'], 'fps', 15);
      aviobj = addframe(aviobj, F(1));
    end
  end

  path_dist = ones(num_iters, 1);

  for iter = 1:num_iters
%    % code to add a second group half-way through
%  for iter = 1:(num_iters/2)
%    if iter == num_iters
%      X((n + 1):(2.*n),:) = X0;
%      V((n + 1):(2.*n),:) = zeros(size(X0));
%%      % add a third group
%%      X0(:,1) = X0(:,1) + 11.*ones(n,1);
%%      X((2.*n + 1):(3.*n),:) = X0;
%%      V((2.*n + 1):(3.*n),:) = zeros(size(X0));
%    end

    % objective found; terminate and return path length
    if sargs.distance_func(X, [3 2]) < sargs.found_radius
      return
    end

    % compute agent velocities and update agent positions accordingly
    V = velocities(X, V, bargs, sargs);
    X = X + dt*V;

    path_dist(iter) = sargs.distance_func(X, [3 2]);

    if to_plot && (mod(iter, 10) == 0)
      plotter(X, terrain);
      F((iter./10)+1) = getframe;
      if to_record
        aviobj = addframe(aviobj, F((iter./10)+1));
      end
    end
  end
  if to_plot
    if to_record
      aviobj = close(aviobj);
    end
    movie(F,20);
    disp('done');
  end
end

function V_next = velocities(X, V, bargs, sargs)

  % allocate space for new velocities (old velocities needed for interaction)
  V_next = zeros(size(V));

  % compute pairwise distances between agents
  D = squareform(pdist(X));

  % compute velocity for each agent
  % NOTE: maybe parallelize this step for large populations,
  % but better to parallelize trials
  for i = 1:size(X,1);
    V_next(i,:) = velocity(i, X, V, D(:,i), bargs, sargs);
  end
end

function v = velocity(i, X, V, D_i, bargs, sargs)

  sigma = bargs.sigma;
  wf = bargs.weight_fun
  dt = sargs.dt;
  terrain = sargs.terrain;

  prevX = X(i,:) - dt*V(i,:);
  dc = terrain(X(i,1),X(i,2)) - terrain(prevX(1),prevX(2)); % change in gradient

  % combine interaction and individual components, according to weight
  w = dc > 0;
  v = wf(interaction(i, X, V, D_i, bargs), V(i,:), dc);

  if norm(v) > 0
    v = v./norm(v);
  end
  v = 0.5.*v + 0.5.*normrnd(0, sigma, 1, 2); % add noise
end

% compute the normalized total effect of interactions on agent i 
function u = interaction(i, X, V, D_i, bargs)

  RR = bargs.RR;
  o_decay = bargs.orient_decay;
  a_decay = bargs.attract_decay;
  df = bargs.disc_fun;

  repulsion_idxs = D_i <= RR & D_i > 0;
  if any(repulsion_idxs) % simply avoid collision(s)
    u = -sum(bsxfun(@minus, X(repulsion_idxs,:), X(i,:)));
  else
    % compute weights for orient and attract based on input decay functions
    w_o = repmat(arrayfun(o_decay, D_i(D_i > 0)),1,2);
    size(w_o)
    w_a = repmat(arrayfun(a_decay, D_i(D_i > 0)),1,2);
    orient = sum(arrayfun(df, V(D_i > 0,:)).*w_o);
    attract = sum(arrayfun(df, bsxfun(@minus, X(D_i > 0,:), X(i,:))).*w_a);
    u = orient + attract;
  end

  % normalize interaction component
  if norm(u) > 0
    u = u/norm(u);
  end
end
