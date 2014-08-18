% returns a bargs struct with the following fields set according to certain
% common methods m:
%   sigma: variance of internal noise
%   RR: radius of repulsion
%   RO: radius of orientation
%   RA: radius of attraction
%   orient_decay: decay function for orientation
%   attract_decay: decay function for attraction
%   disc_fun: discretization function

function bargs = preset(m)

  bargs.sigma = 0.1;
  bargs.RR = 0.1;
  RO = 4;
  RA = 4.3;
  L = 4;
  T = 1;

  if strcmpi(m, 'shklarsh')
    bargs.orient_decay = @(x) (x < RO);
    bargs.attract_decay = @(x) (x < RA && x >= RO);
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + v;

  elseif strcmpi(m, 'adaptive')
    bargs.orient_decay = @(x) 1;
    bargs.attract_decay = @(x) 1;
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'norm_comm')
    bargs.orient_decay = @(x) 1;
    bargs.attract_decay = @(x) 1;
    bargs.disc_fun = @(x) normalize(x);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'no_orient')
    bargs.orient_decay = @(x) 0;
    bargs.attract_decay = @(x) 1;
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'no_attract')
    bargs.orient_decay = @(x) 1;
    bargs.attract_decay = @(x) 0;
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'no_comm')
    bargs.orient_decay = @(x) 0;
    bargs.attract_decay = @(x) 0;
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'd')
    bargs.orient_decay = @(x) (x < RO);
    bargs.attract_decay = @(x) (x < RA);
    bargs.disc_fun = @(x) disc(x, L, T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'dw1')
    bargs.orient_decay = @(x) x^(-1);
    bargs.attract_decay = @(x) x^(-1);
    bargs.disc_fun = @(x) disc(x, L, T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'dw2')
    bargs.orient_decay = @(x) x.^(-2);
    bargs.attract_decay = @(x) x.^(-2);
    bargs.disc_fun = @(x) disc(x, L, T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'wexp')
    bargs.orient_decay = @(x) exp(-2*x);
    bargs.attract_decay = @(x) exp(-x/2);
    bargs.disc_fun = @(x) x;
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  elseif strcmpi(m, 'dwexp')
    bargs.orient_decay = @(x) exp(-2*x);
    bargs.attract_decay = @(x) exp(-x/2);
    bargs.disc_fun = @(x) disc(x, L, T);
    bargs.weight_fun = @(u, v, dc) u + 10*(dc > 0)*v;

  else
    error('Unknown preset: %s', m);
  end
end
