% unicycle_step.m — Integrate unicycle / diff-drive kinematics one step (RK4).
%
%   [x_next, info] = unicycle_step(x, v, omega, dt)
%
%   x   : [x; y; theta] vehicle state (theta = heading, rad)
%   v   : linear velocity (m/s)
%   omega: angular velocity (rad/s)
%   dt  : step size (s)
%
%   Uses ode_rk4 from src/core for the integration, so the result is a real
%   numerical solution of xdot = [v cos(theta); v sin(theta); omega].
%
%   Diff-drive mapping: v = (vr+vl)/2, omega = (vr-vl)/track_width.

function [x_next, info] = unicycle_step(x, v, omega, dt)
  x = reshape(x, 3, 1);
  f = @(t, s) [v * cos(s(3)); v * sin(s(3)); omega];
  [~, s] = ode_rk4(f, [0 dt], x, 1);
  x_next = reshape(s(end, :), 3, 1);
  info.end_state = x_next;
  info.path = s;      % (2 x 3) [start; end] for plotting
endfunction
