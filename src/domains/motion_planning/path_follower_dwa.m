% path_follower_dwa.m — Dynamic Window Approach (DWA) path follower.
%
%   [v, omega] = path_follower_dwa(state, goal, obstacles, opts)
%
%   state   : [x y theta v omega] current robot state
%   goal    : [x y]
%   obstacles: Kx2 [x y] points (or Kx3 circles)
%   opts    : struct (optional)
%     .v_min, .v_max, .omega_min, .omega_max
%     .accel : max accel (m/s^2)
%     .dt    : simulation step (s)
%     .pred  : prediction time (s)
%     .gain_heading, .gain_clearance, .gain_velocity
%
%   Returns (v, omega) command.
%
%   Pure Octave native code.

function [v, omega] = path_follower_dwa(state, goal, obstacles, opts)
  if nargin < 4, opts = struct(); end
  if ~isstruct(opts), opts = struct(); end
  v_min = pc_pick(opts, "v_min", 0.0); v_max = pc_pick(opts, "v_max", 1.0);
  om_min = pc_pick(opts, "omega_min", -1.0); om_max = pc_pick(opts, "omega_max", 1.0);
  accel = pc_pick(opts, "accel", 1.0);
  dt = pc_pick(opts, "dt", 0.1); pred = pc_pick(opts, "pred", 1.0);
  gh = pc_pick(opts, "gain_heading", 1.0); gc = pc_pick(opts, "gain_clearance", 1.0);
  gv = pc_pick(opts, "gain_velocity", 0.5);

  x = state(1); y = state(2); th = state(3); cv = state(4); cw = state(5);
  % dynamic window
  vw = [max(v_min, cv - accel*dt), min(v_max, cv + accel*dt)];
  ww = [max(om_min, cw - accel*dt), min(om_max, cw + accel*dt)];
  % sample
  vs = linspace(vw(1), vw(2), 7);
  ws = linspace(ww(1), ww(2), 7);
  best_v = 0; best_w = 0; best_score = -inf;
  for i = 1:numel(vs)
    for j = 1:numel(ws)
      vc = vs(i); wc = ws(j);
      % simulate
      [xf, yf, thf] = simulate_dwa(x, y, th, vc, wc, dt, pred);
      % heading score
      ang_to_goal = atan2(goal(2)-yf, goal(1)-xf);
      heading = cos(ang_to_goal - thf);
      % clearance
      clearance = obstacle_clearance([xf, yf], obstacles);
      % velocity score
      vel_score = vc / v_max;
      score = gh*heading + gc*clearance + gv*vel_score;
      if score > best_score
        best_score = score; best_v = vc; best_w = wc;
      endif
    endfor
  endfor
  v = best_v; omega = best_w;
endfunction

function [xf, yf, thf] = simulate_dwa(x, y, th, v, w, dt, pred)
  n = round(pred / dt);
  xf = x; yf = y; thf = th;
  for k = 1:n
    thf = thf + w*dt;
    xf = xf + v*cos(thf)*dt;
    yf = yf + v*sin(thf)*dt;
  endfor
endfunction

function c = obstacle_clearance(p, obstacles)
  if isempty(obstacles), c = 1; return; end
  c = inf;
  for i = 1:rows(obstacles)
    d = norm(p - obstacles(i,1:2));
    if obstacles(i,3) > 0 && size(obstacles, 2) >= 3
      d = d - obstacles(i,3);
    endif
    c = min(c, d);
  endfor
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
