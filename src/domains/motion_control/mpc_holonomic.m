% mpc_holonomic.m — Receding-horizon QP MPC for holonomic systems.
%
%   u = mpc_holonomic(x, x_ref, A, B, Q, R, horizon, opts)
%
%   x      : current state (n x 1)
%   x_ref  : reference state (n x 1)
%   A, B   : linear discrete-time dynamics matrices
%   Q, R   : state & input cost matrices
%   horizon: prediction horizon (scalar)
%   opts   : struct (optional)
%     .umin, .umax : input bounds (m x 1)
%     .xmin, .xmax : state bounds (n x 1)
%
%   Builds a dense QP over the horizon and returns the first input.
%   Uses qp_solve.m from src/core.
%
%   Pure Octave native code.

function u = mpc_holonomic(x, x_ref, A, B, Q, R, horizon, opts)
  if nargin < 8, opts = struct(); end
  if ~isstruct(opts), opts = struct(); end
  n = numel(x); m = size(B, 2);
  x = reshape(x, n, 1); x_ref = reshape(x_ref, n, 1);
  umin = pc_pick(opts, "umin", -inf * ones(m, 1));
  umax = pc_pick(opts, "umax", inf * ones(m, 1));
  xmin = pc_pick(opts, "xmin", -inf * ones(n, 1));
  xmax = pc_pick(opts, "xmax", inf * ones(n, 1));

  % prediction matrices
  % decision vector U = [u0; u1; ...; u_{H-1}] (H*m x 1)
  H = horizon;
  % build Phi, Gamma: X = Phi*x0 + Gamma*U
  Phi = zeros(H*n, n); Gamma = zeros(H*n, H*m);
  P = eye(n);
  for k = 1:H
    Phi(k*n - n + 1:k*n, :) = P;
    P = A * P;
    % Gamma column block j contributes A^(k-j) * B * u_j
    for j = 1:k
      Aj = A^(k - j);
      Gamma(k*n - n + 1:k*n, (j-1)*m + 1:j*m) = Aj * B;
    endfor
  endfor

  % cost: sum (x_k - ref)'Q(x_k - ref) + u_k'R u_k
  % X = Phi*x0 + Gamma*U ; cost = (X - refH)' Qb (X - refH) + U' Rb U
  Qb = blkdiag_repmat(Q, H);
  Rb = blkdiag_repmat(R, H);
  refH = repmat(x_ref, H, 1);
  c = Phi * x - refH;   % X = c + Gamma*U
  % Hessian
  Hqp = Gamma' * Qb * Gamma + Rb;
  % gradient
  fqp = Gamma' * Qb * c;
  Hqp = 0.5 * (Hqp + Hqp');   % symmetrize

  % constraints: umin <= u <= umax, xmin <= x <= xmax
  % build A_ineq * U <= b_ineq
  Aineq = []; bineq = [];
  % input bounds
  for k = 1:H
    block = zeros(m, H*m);
    block(:, (k-1)*m + 1:k*m) = eye(m);
    Aineq = [Aineq; block; -block];
    bineq = [bineq; umax; -umin];
  endfor
  % state bounds (x_k = Phi_k*x0 + Gamma_k*U)
  for k = 1:H
    Phik = Phi(k*n - n + 1:k*n, :);
    Gammak = Gamma(k*n - n + 1:k*n, :);
    Aineq = [Aineq; Gammak; -Gammak];
    bineq = [bineq; xmax - Phik*x; -(xmin - Phik*x)];
  endfor

  [U, ~, info] = qp_solve(Hqp, fqp, Aineq, bineq, [], [], [], [], struct());
  if ~isempty(U)
    u = U(1:m);
  else
    u = zeros(m, 1);
  endif
endfunction

function M = blkdiag_repmat(Q, n)
  m = rows(Q);
  M = zeros(n*m, n*m);
  for k = 1:n
    M((k-1)*m + 1:k*m, (k-1)*m + 1:k*m) = Q;
  endfor
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
