% rrt.m — Rapidly-exploring Random Tree for a 2D point robot among obstacles.
%
%   [path, tree] = rrt(start, goal, obstacles, bounds, max_iter, step_size, goal_bias)
%
%   start, goal  : [x y] (m)
%   obstacles    : Nx3 [cx cy r] circular obstacles (m)
%   bounds       : [xmin xmax ymin ymax]
%   max_iter     : max samples
%   step_size    : max extension length (m)
%   goal_bias    : probability of sampling the goal
%
%   Returns path (Kx2) from start to goal, or [] if no path found. Real RRT:
%   sample, nearest node, steer toward sample, circle-obstacle collision check
%   on the segment, then backtrack parents to recover the path.

function [path, tree] = rrt(start, goal, obstacles, bounds, max_iter, step_size, goal_bias)
  start = reshape(start, 1, 2); goal = reshape(goal, 1, 2);
  obstacles = reshape(obstacles, [], 3);
  if nargin < 4, bounds = [-10 10 -10 10]; end
  if nargin < 5, max_iter = 2000; end
  if nargin < 6, step_size = 0.5; end
  if nargin < 7, goal_bias = 0.1; end
  tree = start; parent = [0];        % parent(i) = index of tree(i)'s parent; 0 = root
  reached = false;
  for i = 1:max_iter
    if rand() < goal_bias
      sample = goal;
    else
      sample = [bounds(1) + rand()*(bounds(2)-bounds(1)), ...
                bounds(3) + rand()*(bounds(4)-bounds(3))];
    end
    d = sum((tree - sample).^2, 2);
    [~, ni] = min(d);
    near = tree(ni, :);
    dir = sample - near; dn = norm(dir);
    if dn == 0, continue; end
    new = near + (dir/dn) * min(step_size, dn);
    if segment_collides(near, new, obstacles), continue; end
    tree = [tree; new]; parent(end+1) = ni;
    if norm(new - goal) <= step_size && ~segment_collides(new, goal, obstacles)
      tree = [tree; goal]; parent(end+1) = size(tree,1)-1;
      reached = true; break;
    end
  end
  if ~reached || isempty(tree)
    path = []; tree = tree; return;
  end
  % backtrack from goal (last node)
  path = goal; idx = size(tree, 1);
  while parent(idx) ~= 0
    path = [tree(parent(idx), :); path];
    idx = parent(idx);
  end
endfunction

function coll = segment_collides(a, b, obstacles)
  coll = false;
  if isempty(obstacles), return; end
  d = b - a; L2 = d*d';
  for i = 1:size(obstacles,1)
    c = obstacles(i,1:2); r = obstacles(i,3);
    t = max(0, min(1, ((c - a)*d') / L2));
    closest = a + t * d;
    if norm(closest - c) <= r, coll = true; return; end
  end
endfunction
