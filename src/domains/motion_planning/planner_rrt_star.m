% planner_rrt_star.m — Asymptotically-optimal RRT* (2D & N-DOF C-space).
%
%   path = planner_rrt_star(start, goal, obstacles, bounds, opts)
%
%   start, goal : 1xN vectors (2D: [x y])
%   obstacles   : Kx3 [cx cy r] circles (2D) or empty
%   bounds      : 2xN [min; max] per dimension
%   opts        : struct (optional)
%     .max_iter : (default 5000)
%     .step     : max extension (default 0.5)
%     .goal_bias: prob to sample goal (default 0.1)
%     .radius   : neighbor rewiring radius (default 1.0)
%
%   Returns path (NxD) from start to goal or [] if none.
%
%   Pure Octave native code.

function path = planner_rrt_star(start, goal, obstacles, bounds, opts)
  if nargin < 5 || ~isstruct(opts), opts = struct(); end
  max_iter = pc_pick(opts, "max_iter", 5000);
  step = pc_pick(opts, "step", 0.5);
  goal_bias = pc_pick(opts, "goal_bias", 0.1);
  radius = pc_pick(opts, "radius", 1.0);
  D = numel(start);
  tree = start;          % NxD
  parent = zeros(1, 1);  % parent(1) = 0 root
  cost = zeros(1, 1);    % cost to each node
  found = false; goal_idx = 0;
  rng(0);
  for it = 1:max_iter
    if rand() < goal_bias
      x_rand = goal;
    else
      x_rand = bounds(1, :) + rand(1, D) .* (bounds(2, :) - bounds(1, :));
    endif
    % nearest neighbor
    [~, ni] = min(sum((tree - x_rand).^2, 2));
    x_near = tree(ni, :);
    % steer
    x_new = steer_rrt(x_near, x_rand, step);
    % collision-free?
    if collides_segment(x_near, x_new, obstacles), continue; end
    % neighbor search
    dists = sum((tree - x_new).^2, 2);
    near_idx = find(dists < radius^2);
    if isempty(near_idx), near_idx = ni; end
    % choose best parent
    x_min = x_near; c_min = cost(ni) + norm(x_new - x_near);
    min_idx = ni;
    for j = near_idx'
      if ~collides_segment(tree(j, :), x_new, obstacles)
        c = cost(j) + norm(x_new - tree(j, :));
        if c < c_min
          c_min = c; x_min = tree(j, :); min_idx = j;
        endif
      endif
    endfor
    % add node
    tree = [tree; x_new];
    parent = [parent; min_idx];
    cost = [cost; c_min];
    % rewire
    new_idx = rows(tree);
    for j = near_idx'
      c_new = c_min + norm(tree(j, :) - x_new);
      if c_new < cost(j) && ~collides_segment(x_new, tree(j, :), obstacles)
        cost(j) = c_new;
        parent(j) = new_idx;
      endif
    endfor
    % goal reached?
    if norm(x_new - goal) < step
      found = true; goal_idx = rows(tree); break;
    endif
  endfor

  if ~found, path = []; return; endif
  % reconstruct path
  path = goal;
  idx = goal_idx;
  guard = 0;
  while parent(idx) ~= 0 && guard < max_iter
    guard = guard + 1;
    path = [tree(parent(idx), :); path];
    idx = parent(idx);
  endwhile
  path = [start; path];
endfunction

function x_new = steer_rrt(x_from, x_to, step)
  d = x_to - x_from;
  n = norm(d);
  if n <= step
    x_new = x_to;
  else
    x_new = x_from + step * d / n;
  endif
endfunction

function c = collides_segment(a, b, obstacles)
  c = false;
  if isempty(obstacles), return; endif
  for t = 0:0.1:1
    p = a + t * (b - a);
    for i = 1:rows(obstacles)
      if norm(p - obstacles(i,1:2)) < obstacles(i,3)
        c = true; return;
      endif
    endfor
  endfor
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
