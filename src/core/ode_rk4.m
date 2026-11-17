% ode_rk4.m — Fixed-step 4th-order Runge-Kutta + semi-implicit (symplectic) Euler.
%
%   [t, y] = ode_rk4(f, tspan, y0, h_or_n)
%   [t, y] = semi_implicit_euler(f, tspan, y0, h_or_n)
%
%   f = @(t, y) dydt,  y0 column vector, tspan = [t0 tf].
%   h_or_n: scalar step size (real) OR number of steps (positive integer).
%   Returns t (Nx1), y (N x dim). No GUI, no pause.
%
%   semi_implicit_euler assumes y = [position; velocity] (even length) and that
%   f returns [velocity; acceleration]; it integrates velocity implicitly then
%   position with the new velocity (symplectic Euler).

function [t, y] = ode_rk4(f, tspan, y0, h_or_n)
  y0 = reshape(y0, [], 1);
  t0 = tspan(1); tf = tspan(2);
  [nsteps, h] = parse_steps(h_or_n, t0, tf);
  t = zeros(nsteps + 1, 1);
  y = zeros(nsteps + 1, numel(y0));
  t(1) = t0; y(1, :) = y0';
  nk = 1;
  for k = 1:nsteps
    tk = t(nk);
    yk = y(nk, :)';
    hk = min(h, tf - tk);
    if hk <= 0, break; end
    k1 = f(tk, yk);
    k2 = f(tk + hk/2, yk + hk/2 * k1);
    k3 = f(tk + hk/2, yk + hk/2 * k2);
    k4 = f(tk + hk, yk + hk * k3);
    ykp1 = yk + hk/6 * (k1 + 2*k2 + 2*k3 + k4);
    nk = nk + 1;
    t(nk) = tk + hk;
    y(nk, :) = ykp1';
    if t(nk) >= tf, break; end
  end
  t = t(1:nk); y = y(1:nk, :);
endfunction

function [t, y] = semi_implicit_euler(f, tspan, y0, h_or_n)
  y0 = reshape(y0, [], 1);
  n = numel(y0);
  if mod(n, 2) ~= 0
    error('semi_implicit_euler: state length must be even [position; velocity]');
  end
  np = n / 2;
  t0 = tspan(1); tf = tspan(2);
  [nsteps, h] = parse_steps(h_or_n, t0, tf);
  t = zeros(nsteps + 1, 1);
  y = zeros(nsteps + 1, n);
  t(1) = t0; y(1, :) = y0';
  nk = 1;
  for k = 1:nsteps
    tk = t(nk);
    yk = y(nk, :)';
    hk = min(h, tf - tk);
    if hk <= 0, break; end
    dy = f(tk, yk);               % [velocity; acceleration]
    acc = dy(np+1:end);
    ykp1 = yk;
    ykp1(np+1:end) = yk(np+1:end) + hk * acc;  % v_{k+1} = v_k + h*a_k
    ykp1(1:np) = yk(1:np) + hk * ykp1(np+1:end); % x_{k+1} = x_k + h*v_{k+1}
    nk = nk + 1;
    t(nk) = tk + hk;
    y(nk, :) = ykp1';
    if t(nk) >= tf, break; end
  end
  t = t(1:nk); y = y(1:nk, :);
endfunction

function [nsteps, h] = parse_steps(h_or_n, t0, tf)
  if nargin < 3 || isempty(h_or_n)
    h_or_n = 1000;
  end
  if isscalar(h_or_n) && h_or_n == floor(h_or_n) && h_or_n > 0
    nsteps = h_or_n;
    h = (tf - t0) / nsteps;
  else
    h = h_or_n;
    nsteps = ceil(max(tf - t0, 0) / h);
    if nsteps < 1, nsteps = 1; end
    h = (tf - t0) / nsteps;
  end
endfunction
