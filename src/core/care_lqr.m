% care_lqr.m — Continuous-time Algebraic Riccati Equation (CARE) solver + LQR gain.
%
%   [P, K, info] = care_lqr(A, B, Q, R, options)
%
%   Solves: A'P + PA - P B R^{-1} B' P + Q = 0
%   LQR gain: K = R^{-1} B' P   (minimizes integral x'Qx + u'Ru)
%
%   Method: Kleinman iteration with a high-gain stabilizing initial guess.
%   Each iteration solves a continuous Lyapunov equation by vectorization.
%   Assumes (A,B) stabilizable and (Q,A) detectable.
%
%   info.iter  : iterations taken
%   info.err   : final ||K_new - K|| / ||K_new||
%   info.converged : logical

function [P, K, info] = care_lqr(A, B, Q, R, options)
  if nargin < 5 || isempty(options), options = struct(); end
  if isstruct(options) && isfield(options, 'tol'),   tol   = options.tol;   else tol   = 1e-9; end
  if isstruct(options) && isfield(options, 'maxiter'), maxit = options.maxiter; else maxit = 100; end
  [n, ~] = size(A); m = size(B, 2);
  Q = 0.5 * (Q + Q'); R = 0.5 * (R + R') + 1e-9 * eye(m);
  Rinv = inv(R);

  % --- stabilizing initial gain K0 = c * B' (high-gain output feedback) ---
  K = zeros(m, n);
  c = 0.1;
  stabilized = false;
  for tryc = 1:40
    Kc = c * B';
    Ac = A - B * Kc;
    if max(real(eig(Ac))) < -0.1
      K = Kc; stabilized = true; break;
    end
    c = c * 2;
  end
  if ~stabilized
    error('care_lqr: could not find stabilizing initial gain (check stabilizability)');
  end

  err = inf; it = 0; converged = false;
  for it = 1:maxit
    Ac = A - B * K;
    C = Q + K' * R * K;
    P = lyapunov_solve(Ac, C);   % solves Ac' P + P Ac = -C
    P = 0.5 * (P + P');
    K_new = Rinv * B' * P;
    err = norm(K_new - K) / max(1, norm(K_new));
    K = K_new;
    if err < tol, converged = true; break; end
  end
  P = 0.5 * (P + P');
  K = Rinv * B' * P;
  info.iter = it; info.err = err; info.converged = converged;
endfunction

function P = lyapunov_solve(Ac, C)
  % Solves Ac' P + P Ac = -C  (continuous Lyapunov), Ac assumed stable.
  n = size(Ac, 1);
  M = kron(eye(n), Ac') + kron(Ac', eye(n));
  P = reshape(M \ (-C(:)), n, n);
  P = 0.5 * (P + P');
endfunction
