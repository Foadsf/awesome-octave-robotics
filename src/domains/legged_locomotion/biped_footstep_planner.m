% biped_footstep_planner.m — Capture-point based footstep placement (LIPM).
%
%   [footsteps, capture_points] = biped_footstep_planner(x, xd, step_length, n_steps, goal)
%
%   Linear Inverted Pendulum Model capture point:
%       c = x + xd / w,   w = sqrt(g / zc)
%   Footsteps are placed so the capture point is steered toward a walking
%   goal, each step limited to step_length. The CoM state is integrated
%   under the LIPM (xdd = w^2*(x - foot)) with the ZMP at the placed foot.
%
%   x, xd        : CoM position and velocity (scalar, 1D walking)
%   step_length  : max step length (m)
%   n_steps      : number of footsteps to plan
%   goal         : walking goal position (m)
%
%   Returns footsteps (n_steps x 1) and capture_points (n_steps x 1).
%
%   Pure Octave native code.

function [footsteps, capture_points] = biped_footstep_planner(x, xd, step_length, n_steps, goal)
  if nargin < 5, goal = 1.5; end
  if nargin < 4, n_steps = 6; end
  if nargin < 3, step_length = 0.35; end
  if nargin < 2, xd = 0.3; end
  if nargin < 1, x = 0.1; end

  g = 9.81; zc = 0.8;
  w = sqrt(g / zc);            % pendulum frequency
  footsteps = zeros(n_steps, 1);
  capture_points = zeros(n_steps, 1);
  x_cur = x; xd_cur = xd;
  foot_pos = 0;
  dt = 0.4;                    % step duration
  for k = 1:n_steps
    c = x_cur + xd_cur / w;    % capture point
    capture_points(k) = c;
    % step so the capture point is driven toward the goal
    step_des = foot_pos + (goal - c);
    if step_des - foot_pos > step_length
      step_des = foot_pos + step_length;
    elseif step_des - foot_pos < -step_length
      step_des = foot_pos - step_length;
    endif
    footsteps(k) = step_des;
    foot_pos = step_des;
    % integrate LIPM over the step with ZMP at the foot
    for s = 1:10
      xdd = w^2 * (x_cur - foot_pos);
      x_cur = x_cur + xd_cur * (dt/10);
      xd_cur = xd_cur + xdd * (dt/10);
    endfor
  endfor
endfunction
