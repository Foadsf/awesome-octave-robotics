% dare_lqr.m — Discrete-time Algebraic Riccati Equation (DARE) solver + LQR gain.
%
%   [P, K, info] = dare_lqr(A, B, Q, R, options)
%
%   Solves: P = A'PA - A'PB(R+B'PB)^{-1}B'PA + Q
%   LQR gain: K = (R + B'PB)^{-1} B' P A   (minimizes sum x'Qx + u'Ru)
%
%   Method: policy/value iteration on P (robust fixed-point, no toolbox).
%   Assumes (A,B) stabilizable and (Q,A) detectable.
%
%   info.iter  : iterations taken
%   info.err   : final ||P_new - P||_F / ||P_new||_F
%   info.converged : logical

function [P, K, info] = dare_lqr(A, B, Q, R, options)
  if nargin < 5 || isempty(options), options = struct(); end
  if isstruct(options) && isfield(options, 'tol'),   tol   = options.tol;   else tol   = 1e-10; end
  if isstruct(options) && isfield(options, 'maxiter'), maxit = options.maxiter; else maxit = 500; end
  [n, ~] = size(A); m = size(B, 2);
  Q = 0.5 * (Q + Q'); R = 0.5 * (R + R');
  P = Q;  % stabilizing initial guess for the value iteration
  err = inf; it = 0; converged = false;
  for it = 1:maxit
    S = R + B' * P * B;
    S = S + 1e-10 * eye(m);  % numerical safety
    K = S \ (B' * P * A);
    P_new = A' * P * A - A' * P * B * K + Q;
    err = norm(P_new - P, 'fro') / max(1, norm(P_new, 'fro'));
    P = P_new;
    if err < tol, converged = true; break; end
  end
  P = 0.5 * (P + P');
  S = R + B' * P * B;
  S = S + 1e-10 * eye(m);
  K = S \ (B' * P * A);
  info.iter = it; info.err = err; info.converged = converged;
endfunction
