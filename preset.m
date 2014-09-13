% returns a bargs struct with the following fields set according to certain
% common methods m:
%   sigma: variance of internal noise
%   RR: radius of repulsion
%   RO: radius of orientation
%   RA: radius of attraction
%   L: number of discretization levels
%   T: maximum possible velocity
%   orient_decay: decay function for orientation
%   attract_decay: decay function for attraction
%   disc_fun: discretization function (can depend on L and T)
%   collision_delay: integer number of rounds to disable an agent upon collision (or 0 to use repulsion)
%   blind_frac: fraction of agents who cannot sense food (between 0 and 1)
%   silent_frac: fraction of agent per swarm who do not communicate (between 0 and 1)
%   weight_fun: function weighting interaction component based on gradient (typically 1 or adaptive)

function bargs = preset(m)

  bargs.sigma = 0.1;
  bargs.RR = 0.1;
  bargs.RO = 4;
  bargs.RA = 4.3;
  bargs.L = 4;
  bargs.T = 1;
  bargs.collision_delay = 0;
  bargs.blind_frac = 0;
  bargs.silent_frac = 0;

  if strcmpi(m, 'shklarsh') % original Shklarsh model
    bargs.orient_decay = @(x) (x < bargs.RO);
    bargs.attract_decay = @(x) (x < bargs.RA && x >= bargs.RO);
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + v;

  elseif strcmpi(m, 'adaptive') % adaptive 
    bargs.orient_decay = @(x) (x < bargs.RO && x >= bargs.RR);
    bargs.attract_decay = @(x) 1;
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'd') % discrete
    bargs.orient_decay = @(x) (x < RO);
    bargs.attract_decay = @(x) (x < RA);
    bargs.disc_fun = @(x) disc(x, bargs.L, bargs.T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'dw1') % discrete, 1/x weighted
    bargs.orient_decay = @(x) x^(-1);
    bargs.attract_decay = @(x) x^(-1);
    bargs.disc_fun = @(x) disc(x, bargs.L, bargs.T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'dw2') % discrete, 1/x^2 weighted
    bargs.orient_decay = @(x) x.^(-2);
    bargs.attract_decay = @(x) x.^(-2);
    bargs.disc_fun = @(x) disc(x, bargs.L, bargs.T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'wexp') % continuous, exponentially weighted; TODO: should probably make decay rates parameters
    bargs.orient_decay = @(x) exp(-2*x);
    bargs.attract_decay = @(x) exp(-x/2);
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'dwexp') % discrete, exponentially weighted; out "best" model
    bargs.orient_decay = @(x) exp(-2*x);
    bargs.attract_decay = @(x) exp(-x);
    bargs.disc_fun = @(x) disc(x, bargs.L, bargs.T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  % THE FOLLOWING MODELS ARE ALL DISCRETIZED AND WEIGHTED (where appropriate)

  elseif strcmpi(m, 'norm_comm') % no magnitude for orientation and attraction
    bargs.orient_decay = @(x) 1;
    bargs.attract_decay = @(x) 1;
    bargs.disc_fun = @(x) normalize(x);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'no_orient') % no orientation
    bargs.orient_decay = @(x) 0;
    bargs.attract_decay = @(x) exp(-x);
    bargs.disc_fun = @(x) disc(x, bargs.L, bargs.T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'no_attract') % no attraction
    bargs.orient_decay = @(x) exp(-2*x);
    bargs.attract_decay = @(x) 0;
    bargs.disc_fun = @(x) disc(x, bargs.L, bargs.T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'no_rep') % no repulsion
    bargs.orient_decay = @(x) exp(-2*x);
    bargs.attract_decay = @(x) exp(-x);
    bargs.disc_fun = @(x) disc(x, bargs.L, bargs.T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;
    bargs.collision_delay = 10;

  elseif strcmpi(m, 'no_comm') % no orientation or attraction
    bargs.orient_decay = @(x) 0;
    bargs.attract_decay = @(x) 0;
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  else
    error('Unknown preset: %s', m);
  end
end
