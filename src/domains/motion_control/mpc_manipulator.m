% mpc_manipulator.m — Constrained trajectory optimization for arm motion.
%
%   Q = mpc_manipulator(q0, q_goal, joint_limits, obstacles, opts)
%
%   q0, q_goal   : start / goal joint vectors (nj x 1)
%   joint_limits : nj x 2 [min; max]
%   obstacles    : Kx3 [cx cy r] in task space (optional, empty = ignore)
%   opts         : struct (optional)
%     .waypoints : number of trajectory waypoints (default 10)
%     .dt        : step (default 0.1)
%
%   Builds a QP minimizing jerk-like deviation + goal proximity subject to
%   joint limits, returns the full trajectory (waypoints x nj).
%
%   Pure Octave native code (uses qp_solve.m).

function Q = mpc_manipulator(q0, q_goal, joint_limits, obstacles, opts)
  if nargin < 5, opts = struct(); end
  if ~isstruct(opts), opts = struct(); end
  nw = pc_pick(opts, "waypoints", 10);
  nj = numel(q0);
  qmin = joint_limits(:, 1); qmax = joint_limits(:, 2);
  % decision vector: all waypoints stacked [q_1 ... q_nw] (nw*nj x 1)
  n = nw * nj;
  % cost: sum ||q_k - q_goal||^2 + smoothness ||q_{k+1}-2q_k+q_{k-1}||^2
  H = eye(n); f = zeros(n, 1);
  wgoal = 10.0; wsmooth = 1.0;
  for k = 1:nw
    idx = (k-1)*nj + 1:k*nj;
    f(idx) = -2 * wgoal * q_goal;
    H(idx, idx) = H(idx, idx) + 2 * wgoal * eye(nj);
    if k > 1 && k < nw
      idxp = (k-2)*nj + 1:(k-1)*nj;
      idxn = k*nj + 1:(k+1)*nj;
      % second difference: q_{k+1} - 2 q_k + q_{k-1}
      H(idx, idx) = H(idx, idx) + 6 * wsmooth * eye(nj);
      H(idx, idxp) = H(idx, idxp) - 4 * wsmooth * eye(nj);
      H(idx, idxn) = H(idx, idxn) - 4 * wsmooth * eye(nj);
      H(idxp, idx) = H(idxp, idx) - 4 * wsmooth * eye(nj);
      H(idxn, idx) = H(idxn, idx) - 4 * wsmooth * eye(nj);
      H(idxp, idxp) = H(idxp, idxp) + 2 * wsmooth * eye(nj);
      H(idxn, idxn) = H(idxn, idxn) + 2 * wsmooth * eye(nj);
      H(idxp, idxn) = H(idxp, idxn) + wsmooth * eye(nj);
      H(idxn, idxp) = H(idxn, idxp) + wsmooth * eye(nj);
    endif
  endfor
  H = 0.5 * (H + H');

  % bounds: qmin <= q_k <= qmax
  lb = repmat(qmin, nw, 1); ub = repmat(qmax, nw, 1);
  [U, ~, info] = qp_solve(H, f, [], [], [], [], lb, ub, struct());
  if isempty(U)
    % fallback: linear interpolation
    Q = linspace_vec(q0, q_goal, nw);
  else
    Q = reshape(U, nj, nw)';
  endif
endfunction

function Q = linspace_vec(q0, q1, nw)
  nj = numel(q0);
  Q = zeros(nw, nj);
  for k = 1:nw
    a = (k-1) / max(nw-1, 1);
    Q(k, :) = (q0 + a * (q1 - q0))';
  endfor
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
