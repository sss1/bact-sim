% discretizes the magnitude and direction of x,
% using stone-age threshold T with L > 1 levels
function D = disc(x, L, T)

  l = L - 1; % due to a technicality of rounding implementation;

  % round direction to nearest of 8 cardinal vectors; TODO: add levels
  dirs = normr([-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1]);
  [~,I] = max(x(1)*dirs(:,1) + x(2)*dirs(:,2)); % take dir maximizing dot prod
  dir = dirs(I,:);

  % round magnitude to nearest of L levels between 0 and T
  mag = T.*floor(l.*min(T,norm(x))./T)./l;

  D = mag*dir;
end
