% semi_implicit_euler.m — Symplectic (semi-implicit) Euler integrator.
%
%   [t, y] = semi_implicit_euler(f, tspan, y0, h_or_n)
%
%   Splits state y = [position; velocity]; updates velocity first then
%   position using the new velocity (symplectic, conserves energy better than
%   explicit Euler for oscillatory systems).
%
%   h_or_n: scalar step size, or positive integer number of steps.

function [t, y] = semi_implicit_euler(f, tspan, y0, h_or_n)
  y0 = reshape(y0, [], 1);
  n = numel(y0);
  if mod(n, 2) ~= 0
    error('semi_implicit_euler: state length must be even [position; velocity]');
  end
  np = n / 2;
  t0 = tspan(1); tf = tspan(2);
  if nargin < 4 || isempty(h_or_n)
    h_or_n = 1000;
  end
  if isscalar(h_or_n) && h_or_n == floor(h_or_n) && h_or_n > 0
    nsteps = h_or_n; h = (tf - t0) / nsteps;
  else
    h = h_or_n; nsteps = ceil(max(tf - t0, 0) / h);
    if nsteps < 1, nsteps = 1; end
    h = (tf - t0) / nsteps;
  end
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
