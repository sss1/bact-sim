% plots the bacteria swarm and its mean over a contour plot of the terrain
% Inputs:
%   X - n-by-2 matrix of bacteria positions
%   terrain - function mapping R X R -> R; maps (x,y) to terrain height
% Outputs:

function plotter(X, terrain)

  clf; % clear figure

  [n, ~] = size(X);

  xmin = -5;%min(X(:,1)) - 4;
  xmax = 7;%max(X(:,1)) + 4;
  ymin = -6;%min(X(:,2)) - 4;
  ymax = 6;%max(X(:,2)) + 4;

  % plot terrain
  ezcontour(terrain, [xmin,xmax,ymin,ymax], 250);

  hold all;
  r = min(xmax - xmin, ymax - ymin)/50;
  for i = 1:n
    draw_circle(X(i,1), X(i,2), r/5,'k');
  end

  % draw big red point at mean
  avg = mean(X,1);
  draw_circle(avg(1),avg(2),r./2,'r');

  drawnow;
end

% draws c-colored circle of radius r, centered at (x,y)
function draw_circle(x, y, r, c)

  thickness = 5;
  if c == 'k'
    thickness = 3;
  end

  theta=0:0.1:2*pi; 
  plot(x+r*cos(theta), y+r*sin(theta), c, 'LineWidth', thickness);

end
