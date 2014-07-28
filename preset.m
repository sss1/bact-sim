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

  bargs.sigma = 0.5;
  bargs.RR = 0.1;
  RO = 4;
  RA = 4.3;
  L = 4;
  T = 1;

  if strcmpi(m, 'shklarsh')
    bargs.orient_decay = @(x) (x < RO);
    bargs.attract_decay = @(x) (x < RA);
    bargs.disc_fun = @(x,y) [x y];

  elseif strcmpi(m, 'd')
    bargs.orient_decay = @(x) (x < RO);
    bargs.attract_decay = @(x) (x < RA);
    bargs.disc_fun = @(x,y) disc([x y], L, T);

  elseif strcmpi(m, 'dw1')
    bargs.orient_decay = @(x) x^(-1);
    bargs.attract_decay = @(x) x^(-1);
    bargs.disc_fun = @(x) disc(x, L, T);

  elseif strcmpi(m, 'dw2')
    bargs.orient_decay = @(x) x.^(-2);
    bargs.attract_decay = @(x) x.^(-2);
    bargs.disc_fun = @(x) disc(x, L, T);

  else
    error('Unknown preset: %s', m);
  end
end
