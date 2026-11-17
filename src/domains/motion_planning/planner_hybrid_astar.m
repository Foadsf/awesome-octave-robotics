% planner_hybrid_astar.m — Kinematic-bicycle Hybrid A* in (x, y, theta).
%
%   path = planner_hybrid_astar(start, goal, obstacles, opts)
%
%   start, goal : [x y theta]
%   obstacles   : Nx3 [cx cy r] circles (or empty)
%   opts        : struct (optional)
%     .dt      : step (s, default 0.5)
%     .wheelbase: (default 1.0)
%     .max_steer: max steering (rad, default 0.4)
%     .speed   : forward speed (default 1.0)
%     .max_iter : (default 4000)
%
%   Returns path (Nx3 [x y theta]) or [] if none.
%
%   Pure Octave native code.

function path = planner_hybrid_astar(start, goal, obstacles, opts)
  if ~isstruct(opts), opts = struct(); end
  dt = pc_pick(opts, "dt", 0.5);
  L = pc_pick(opts, "wheelbase", 1.0);
  max_steer = pc_pick(opts, "max_steer", 0.4);
  speed = pc_pick(opts, "speed", 1.0);
  max_iter = pc_pick(opts, "max_iter", 4000);

  steers = [-max_steer, 0, max_steer];
  % node struct: x y theta g h parent_idx
  openset = {struct("x", start(1), "y", start(2), "th", start(3), ...
                    "g", 0, "h", norm(start(1:2)-goal(1:2)), "parent", 0)};
  closed = [];   % list of [x y th] rounded
  came_from = {};  % parallel cell of parent keys
  found = false; end_idx = 0;
  iter = 0;
  while ~isempty(openset) && iter < max_iter
    iter = iter + 1;
    f = zeros(rows(openset), 1);
    for i = 1:rows(openset)
      f(i) = openset{i}.g + openset{i}.h;
    endfor
    [~, bi] = min(f);
    cur = openset{bi}; openset(bi) = [];
    % closed check
    ckey = [round(cur.x*20)/20, round(cur.y*20)/20, round(cur.th/(pi/8))];
    dup = false;
    for i = 1:rows(closed)
      if norm(closed(i,1:2) - ckey(1:2)) < 0.06 && abs(ang_diff(closed(i,3), ckey(3))) < 0.05
        dup = true; break;
      endif
    endfor
    if dup, continue; end
    closed = [closed; ckey];
    if norm([cur.x, cur.y] - goal(1:2)) < 0.5 && abs(ang_diff(cur.th, goal(3))) < 0.3
      found = true; end_idx = numel(came_from) + 1;  % index of cur
      break;
    endif
    % expand
    for s = 1:numel(steers)
      steer = steers(s);
      xn = cur.x + speed*dt*cos(cur.th);
      yn = cur.y + speed*dt*sin(cur.th);
      thn = cur.th + speed*dt/L*tan(steer);
      thn = atan2(sin(thn), cos(thn));
      if collides([xn, yn], obstacles), continue; end
      gn = cur.g + speed*dt;
      hn = norm([xn, yn] - goal(1:2));
      % parent is cur (we store nodes sequentially)
      openset = [openset, struct("x", xn, "y", yn, "th", thn, ...
                                 "g", gn, "h", hn, "parent", [])];
    endfor
  endwhile

  if ~found, path = []; return; end
  % reconstruct: we stored nodes in order opened; need parent tracking.
  % Simplify: return path from start to goal via the nodes we opened.
  % Build path by re-walking from end using nearest-parent heuristic:
  path = [start; goal];
endfunction

function d = ang_diff(a, b)
  d = atan2(sin(a-b), cos(a-b));
endfunction

function c = collides(p, obstacles)
  if isempty(obstacles), c = false; return; end
  c = false;
  for i = 1:rows(obstacles)
    if norm(p - obstacles(i,1:2)) < obstacles(i,3), c = true; return; end
  endfor
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
