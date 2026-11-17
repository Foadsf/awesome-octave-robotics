% planner_prm.m — Probabilistic Roadmap (PRM) multi-query planner.
%
%   path = planner_prm(start, goal, obstacles, bounds, opts)
%
%   Builds a random geometric graph of collision-free samples, connects
%   k-nearest neighbors, then runs Dijkstra from start to goal.
%
%   start, goal : 1x2 [x y]
%   obstacles   : Kx3 [cx cy r] or empty
%   bounds      : 2x2 [min; max] per dimension
%   opts        : struct (optional)
%     .n_samples: (default 500)
%     .k_nearest: (default 10)
%     .radius   : connection radius (default 1.0)
%
%   Returns path (Nx2) or [] if none.
%
%   Pure Octave native code.

function path = planner_prm(start, goal, obstacles, bounds, opts)
  if nargin < 5 || ~isstruct(opts), opts = struct(); end
  n_samples = pc_pick(opts, "n_samples", 500);
  k_nearest = pc_pick(opts, "k_nearest", 10);
  radius = pc_pick(opts, "radius", 1.0);
  rng(0);
  % samples + start + goal
  samples = [start; goal];
  for i = 1:n_samples
    samples = [samples; bounds(1,:) + rand(1,2) .* (bounds(2,:)-bounds(1,:))];
  endfor
  N = rows(samples);
  % adjacency
  adj = zeros(N, N);
  for i = 1:N
    dists = sum((samples - samples(i,:)).^2, 2);
    [~, order] = sort(dists);
    cnt = 0;
    for j = 2:N
      nb = order(j);
      if nb == i, continue; end
      if dists(nb) > radius^2, break; end
      if ~collides_segment(samples(i,:), samples(nb,:), obstacles)
        adj(i, nb) = sqrt(dists(nb));
        adj(nb, i) = adj(i, nb);
        cnt = cnt + 1;
      endif
      if cnt >= k_nearest, break; end
    endfor
  endfor
  % Dijkstra from node 1 (start) to node 2 (goal)
  dist = inf(N, 1); dist(1) = 0;
  prev = zeros(N, 1);
  visited = false(N, 1);
  for it = 1:N
    % pick unvisited min dist
    ud = dist; ud(visited) = inf;
    [~, u] = min(ud);
    if dist(u) == inf, break; endif
    visited(u) = true;
    if u == 2, break; endif
    for v = 1:N
      if adj(u, v) > 0 && ~visited(v)
        alt = dist(u) + adj(u, v);
        if alt < dist(v)
          dist(v) = alt; prev(v) = u;
        endif
      endif
    endfor
  endfor
  if dist(2) == inf, path = []; return; end
  % reconstruct
  path = [2];
  u = 2;
  while prev(u) ~= 0
    path = [prev(u), path];
    u = prev(u);
  endwhile
  path = samples(path, :);
endfunction

function c = collides_segment(a, b, obstacles)
  c = false;
  if isempty(obstacles), return; end
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
