% sim_discrete.m — Fixed-step discrete-time simulator (real-time-style loop).
%
%   [t, X, U] = sim_discrete(f, x0, T, dt, controller)
%
%   f          : function handle f(x, u) -> x_next (state transition)
%   x0         : initial state (column vector)
%   T          : simulation time (s)
%   dt         : fixed step (s)
%   controller : function handle u = controller(x, t) (may be empty)
%
%   Runs x[k+1] = f(x[k], u[k]) at fixed dt, logs time/states/controls.
%   The deterministic time-stepping core of any discrete simulator.

function [t, X, U] = sim_discrete(f, x0, T, dt, controller)
  x0 = reshape(x0, [], 1);
  if nargin < 5 || isempty(controller)
    controller = @(x, t) zeros(0,1);
  end
  u0 = controller(x0, 0);
  nu = max(1, numel(u0));
  n = max(1, round(T/dt));
  X = zeros(n, numel(x0)); U = zeros(n, nu);
  x = x0;
  for k = 1:n
    tk = (k-1)*dt;
    u = controller(x, tk);
    if isempty(u), u = zeros(nu,1); end
    X(k, :) = x'; U(k, :) = u';
    x = f(x, u);
  end
  t = (0:n-1)' * dt;
endfunction
