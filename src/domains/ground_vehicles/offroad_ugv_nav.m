% offroad_ugv_nav.m — 2.5D heightmap traversability + off-road A* navigation.
%
%   [path, costmap, metrics] = offroad_ugv_nav(grid_size, start, goal, seed)
%
%   Builds a random 2.5D elevation heightmap, computes a traversability
%   costmap from local slope and roughness (standard deviation of elevation
%   in a 3x3 window), then plans a minimum-cost path from start to goal with
%   A* (8-connected, diagonal cost = sqrt(2)). Cells above a max slope are
%   treated as obstacles.
%
%   Returns:
%     path        : (N x 2) [row col] waypoints from start to goal (empty if
%                   no path)
%     costmap     : (grid_size x grid_size) traversability cost (inf = blocked)
%     metrics     : struct with path_cost, path_length, nodes_expanded,
%                   blocked_fraction
%
%   Pure Octave native code.

function [path, costmap, metrics] = offroad_ugv_nav(grid_size, start, goal, seed)
  if nargin < 4, seed = 0; end
  if nargin < 3, goal = [grid_size, grid_size]; end
  if nargin < 2, start = [1, 1]; end
  if nargin < 1, grid_size = 40; end

  rng(seed);
  % 2.5D elevation: layered noise-like hills via summed sines + random bumps
  [X, Y] = meshgrid(1:grid_size, 1:grid_size);
  Z = zeros(grid_size);
  for k = 1:3
    Z = Z + 8*sin(2*pi*rand()*X/grid_size + rand()*pi) ...
          .* cos(2*pi*rand()*Y/grid_size + rand()*pi);
  endfor
  for b = 1:15
    cx = randi(grid_size); cy = randi(grid_size);
    Z = Z + 6*exp(-((X-cx).^2 + (Y-cy).^2) / (2*5^2));
  endfor
  Z = Z + 0.5*randn(grid_size);

  % local slope & roughness (3x3 window)
  slope = zeros(grid_size);
  rough = zeros(grid_size);
  for i = 1:grid_size
    for j = 1:grid_size
      r1 = max(1,i-1); r2 = min(grid_size,i+1);
      c1 = max(1,j-1); c2 = min(grid_size,j+1);
      win = Z(r1:r2, c1:c2);
      % slope = max gradient magnitude to neighbors (approx)
      dz = [Z(min(i+1,grid_size),j) - Z(max(i-1,1),j), ...
            Z(i,min(j+1,grid_size)) - Z(i,max(j-1,1))];
      slope(i,j) = norm(dz);
      rough(i,j) = std(win(:));
    endfor
  endfor

  % costmap: base 1 + slope penalty + roughness penalty; blocked if too steep
  max_slope = 1.2;
  costmap = 1.0 + 2.0*slope + 3.0*rough;
  blocked = slope > max_slope;
  costmap(blocked) = inf;

  % A* 8-connected
  [path, metrics.nodes_expanded] = astar_costmap(costmap, start, goal);

  if ~isempty(path) && size(path,1) > 1
    % path cost & length (Euclidean step cost)
    pc = 0; pl = 0;
    for k = 1:size(path,1)-1
      d = norm(path(k+1,:) - path(k,:));
      pc = pc + d * costmap(path(k,1), path(k,2));
      pl = pl + d;
    endfor
    metrics.path_cost = pc;
    metrics.path_length = pl;
  else
    metrics.path_cost = inf;
    metrics.path_length = 0;
  endif
  metrics.blocked_fraction = sum(blocked(:)) / numel(blocked);
  metrics.start = start; metrics.goal = goal;
endfunction

function [path, n_expanded] = astar_costmap(cost, start, goal)
  gs = size(cost, 1);
  g = inf(gs, gs); f = inf(gs, gs);
  parent = zeros(gs, gs, 2);
  closed = false(gs, gs);
  g(start(1), start(2)) = 0;
  f(start(1), start(2)) = heuristic(start, goal);
  openx = start(1); openy = start(2);
  n_expanded = 0;
  path = [];
  dirs = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
  dcost = [1 1 1 1 sqrt(2) sqrt(2) sqrt(2) sqrt(2)];
  while true
    [cx, cy, bi] = pop_best_cost(openx, openy, f, closed);
    if isempty(cx), break; end
    closed(cx, cy) = true;
    n_expanded = n_expanded + 1;
    if cx==goal(1) && cy==goal(2)
      path = reconstruct(parent, start, goal);
      return;
    endif
    for d = 1:8
      nx = cx + dirs(d,1); ny = cy + dirs(d,2);
      if nx<1 || nx>gs || ny<1 || ny>gs || closed(nx,ny), continue; end
      if isinf(cost(nx,ny)), continue; end
      tent = g(cx,cy) + dcost(d) * cost(nx,ny);
      if tent < g(nx,ny)
        parent(nx,ny,:) = [cx, cy];
        g(nx,ny) = tent;
        f(nx,ny) = tent + heuristic([nx ny], goal);
        openx = [openx; nx]; openy = [openy; ny];
      endif
    endfor
  endwhile
endfunction

function h = heuristic(a, b)
  h = norm(a - b);
endfunction

function [cx, cy, bi] = pop_best_cost(openx, openy, f, closed)
  if isempty(openx), cx = []; cy = []; bi = 0; return; end
  n = length(openx);
  bestf = inf; bi = 0;
  for i = 1:n
    if closed(openx(i), openy(i)), continue; end
    if f(openx(i), openy(i)) < bestf
      bestf = f(openx(i), openy(i)); bi = i;
    endif
  endfor
  if bi == 0, cx = []; cy = []; return; end
  cx = openx(bi); cy = openy(bi);
  openx(bi) = []; openy(bi) = [];
endfunction

function path = reconstruct(parent, start, goal)
  path = goal;
  cx = goal(1); cy = goal(2);
  while !(cx==start(1) && cy==start(2))
    px = parent(cx,cy,1); py = parent(cx,cy,2);
    if px==0 && py==0, break; end
    path = [px py; path];
    cx = px; cy = py;
  endwhile
endfunction
