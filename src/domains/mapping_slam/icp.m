% icp.m — Iterative Closest Point 2D scan matching (point-to-point, Kabsch/SVD).
%
%   [T, cost, iters] = icp(P, Q, max_iters, tol)
%
%   P, Q    : Nx2 / Mx2 point clouds; P is aligned onto Q
%   max_iters: maximum iterations
%   tol     : stop when mean distance change < tol
%
%   Returns T (3x3 homogeneous SE(2), column-vector convention p_new = T*[p;1]),
%   final RMS cost, and iterations used. Each iteration finds nearest neighbours
%   in Q for every point of P and solves the Kabsch rigid transform via SVD.

function [T, cost, iters] = icp(P, Q, max_iters, tol)
  P = reshape(P, [], 2); Q = reshape(Q, [], 2);
  N = size(P, 1); M = size(Q, 1);
  if nargin < 3, max_iters = 50; end
  if nargin < 4, tol = 1e-4; end
  Pc = P'; Qc = Q';                % 2xN / 2xM columns
  T = eye(3); cost = inf; iters = 0;
  for it = 1:max_iters
    % transform P by current estimate (column vectors)
    Pw = T(1:2, 1:2) * Pc + T(1:2, 3);   % 2xN
    % pairwise distances NxM, nearest neighbour index for each P point
    diff = reshape(Pw, 2, N, 1) - reshape(Qc, 2, 1, M);  % 2xNxM
    D = sqrt(sum(diff .* diff, 1));                       % 1xNxM
    D = squeeze(D);                                       % NxM
    [d, idx] = min(D, [], 2);                             % Nx1 nearest Q index
    % matched pairs (columns)
    Pm = Pw(:, idx'); Qm = Qc(:, idx');
    pmean = mean(Pm, 2); qmean = mean(Qm, 2);
    Pcen = Pm - pmean; Qcen = Qm - qmean;
    H = Pcen * Qcen';                          % 2x2
    [U, ~, V] = svd(H);
    R = V * diag([1, det(V * U')]) * U';   % rotation mapping Pw -> Qm
    t = qmean - R * pmean;
    T = [R t; 0 0 1] * T;                      % compose on the left
    newcost = mean(d);
    iters = it;
    if abs(cost - newcost) < tol, cost = newcost; break; end
    cost = newcost;
  end
endfunction
