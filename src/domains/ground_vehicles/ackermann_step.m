% ackermann_step.m — Kinematic bicycle / Ackermann steering model, one step.
%
%   [x_next, info] = ackermann_step(x, v, delta, dt, L)
%   [x_next, info] = ackermann_step(x, v, delta, dt, L, delta_prev, delta_rate_max)
%
%   Kinematic bicycle model:
%     xdot = v * cos(theta)
%     ydot = v * sin(theta)
%     thetadot = v * tan(delta) / L
%
%   x   : [x; y; theta] vehicle state (theta = heading, rad)
%   v   : longitudinal velocity (m/s), may be negative (reverse)
%   delta: commanded front steering angle (rad), clamped to +/-30 deg
%   dt  : step size (s)
%   L   : wheelbase (m)
%   delta_prev: previous steering angle (rad) for rate limiting (optional)
%   delta_rate_max: max steering rate (rad/s) for rate limiting (optional)
%
%   Uses ode_rk4 from src/core for integration, a real numerical solution.
%   Returns x_next (3x1) and info with end_state, path, delta_used.

function [x_next, info] = ackermann_step(x, v, delta, dt, L, delta_prev, delta_rate_max)
  x = reshape(x, 3, 1);
  if nargin < 5 || isempty(L), L = 1.0; end
  if nargin < 4 || isempty(dt), dt = 0.01; end
  if nargin < 3 || isempty(delta), delta = 0.0; end
  if nargin < 2 || isempty(v), v = 0.0; end

  delta_max = pi / 6;   % 30 deg default steering limit
  delta_min = -delta_max;

  % steering-rate limit: |delta - delta_prev| <= delta_rate_max * dt
  delta_used = delta;
  if nargin >= 7 && ~isempty(delta_rate_max) && delta_rate_max > 0
    dmax = delta_rate_max * dt;
    if nargin >= 6 && ~isempty(delta_prev)
      d = delta - delta_prev;
      if d > dmax, delta_used = delta_prev + dmax;
      elseif d < -dmax, delta_used = delta_prev - dmax;
      else, delta_used = delta; end
    endif
  endif

  % clamp to angle limits
  if delta_used > delta_max, delta_used = delta_max;
  elseif delta_used < delta_min, delta_used = delta_min; end

  f = @(t, s) [v * cos(s(3));
               v * sin(s(3));
               v * tan(delta_used) / L];
  [~, s] = ode_rk4(f, [0 dt], x, 1);
  x_next = reshape(s(end, :), 3, 1);
  info.end_state = x_next;
  info.path = s;                      % (2 x 3) [start; end]
  info.delta_used = delta_used;
  info.wheelbase = L;
endfunction
