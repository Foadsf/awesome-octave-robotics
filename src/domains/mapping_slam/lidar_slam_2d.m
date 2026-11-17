% lidar_slam_2d.m — 2D lidar SLAM: scan-to-scan ICP odometry + pose-graph SLAM.
%
%   poses = lidar_slam_2d(scans, opts)
%
%   scans   : cell array of N scans; each is an Mx2 matrix of (x,y) points
%   opts    : struct with optional fields
%     .max_dist : max ICP correspondence distance (default 0.5)
%     .icp_iters: ICP iterations per pair (default 50)
%     .loop_thr : distance threshold for loop-closure detection (default 1.0)
%     .opt_iters: pose-graph Gauss-Newton iterations (default 20)
%
%   Returns optimized absolute poses (Nx3 [x y theta]).
%
%   Pure Octave native code (uses icp.m from this package).

function poses = lidar_slam_2d(scans, opts)
  if nargin < 2, opts = struct(); end
  if ~isstruct(opts), opts = struct(); end
  max_dist = pc_pick(opts, "max_dist", 0.5);
  icp_iters = pc_pick(opts, "icp_iters", 50);
  loop_thr = pc_pick(opts, "loop_thr", 1.0);
  opt_iters = pc_pick(opts, "opt_iters", 20);

  N = numel(scans);
  poses = zeros(N, 3);
  % --- odometry: ICP between consecutive scans ---
  rel = zeros(N-1, 3);
  for i = 1:N-1
    [T, ~, ~] = icp(scans{i}, scans{i+1}, icp_iters, 1e-5);
    rel(i, 1) = T(1,3); rel(i, 2) = T(2,3);
    rel(i, 3) = atan2(T(2,1), T(1,1));
  endfor

  % --- build absolute trajectory from odometry ---
  poses(1, :) = [0, 0, 0];
  for i = 2:N
    th = poses(i-1, 3);
    R = [cos(th), -sin(th); sin(th), cos(th)];
    d = R * rel(i-1, 1:2)';
    poses(i, 1:2) = poses(i-1, 1:2) + d';
    poses(i, 3) = atan2(sin(poses(i-1,3)+rel(i-1,3)), cos(poses(i-1,3)+rel(i-1,3)));
  endfor

  % --- loop closure detection: nearby non-consecutive poses ---
  loop_closures = zeros(0, 4);
  for i = 1:N
    for j = i+2:N
      if norm(poses(i,1:2) - poses(j,1:2)) < loop_thr
        % estimate relative transform between scan i and j via ICP
        [T, ~, ~] = icp(scans{i}, scans{j}, icp_iters, 1e-5);
        dx = T(1,3); dy = T(2,3);
        dth = atan2(T(2,1), T(1,1));
        loop_closures = [loop_closures; i, j, dx, dth];
      endif
    endfor
  endfor

  % --- pose-graph optimization (reuse optimizer) ---
  if rows(loop_closures) > 0 || N > 1
    poses = slam_optimize(poses, rel, loop_closures, opt_iters);
  endif
endfunction

function poses = slam_optimize(poses, rel, loop_closures, iters)
  N = rows(poses);
  lambda = 1e-3;
  for it = 1:iters
    r = []; J = [];
    % odometry edges
    for i = 2:N
      a = poses(i-1,:); b = poses(i,:);
      res = relative_pose_res(a, b, rel(i-1,:)');
      [Ji, Jj] = pose_jacobians(a, b);
      r = [r; res];
      Jrow = zeros(3, 3*N);
      Jrow(:, 3*(i-1)-2:3*(i-1)) = Ji;
      Jrow(:, 3*i-2:3*i) = Jj;
      J = [J; Jrow];
    endfor
    % loop closure edges
    for k = 1:rows(loop_closures)
      i = loop_closures(k,1); j = loop_closures(k,2);
      meas = [loop_closures(k,3); 0; loop_closures(k,4)];
      a = poses(i,:); b = poses(j,:);
      res = relative_pose_res(a, b, meas);
      [Ji, Jj] = pose_jacobians(a, b);
      r = [r; res];
      Jrow = zeros(3, 3*N);
      Jrow(:, 3*i-2:3*i) = Ji;
      Jrow(:, 3*j-2:3*j) = Jj;
      J = [J; Jrow];
    endfor
    H = J' * J + lambda * eye(3*N);
    g = J' * r;
    dx = -H \ g;
    for i = 1:N
      poses(i,1:2) = poses(i,1:2) + dx(3*i-2:3*i-1)';
      poses(i,3) = atan2(sin(poses(i,3)+dx(3*i)), cos(poses(i,3)+dx(3*i)));
    endfor
    if norm(dx) < 1e-8, break; end
  endfor
endfunction

function res = relative_pose_res(a, b, meas)
  rel = relative_pose(a, b);
  res = rel - meas;
  res(3) = atan2(sin(res(3)), cos(res(3)));
endfunction

function rel = relative_pose(a, b)
  th = a(3);
  R = [cos(th), -sin(th); sin(th), cos(th)];
  dp = R' * (b(1:2)' - a(1:2)');
  rel = [dp; atan2(sin(b(3)-a(3)), cos(b(3)-a(3)))];
endfunction

function [Ji, Jj] = pose_jacobians(a, b)
  th = a(3); c = cos(th); s = sin(th);
  Ji = zeros(3,3); Jj = zeros(3,3);
  Rp = [c, s; -s, c];
  Jj(1:2,1:2) = Rp; Jj(3,3) = 1;
  dp = Rp * (b(1:2)' - a(1:2)');
  dR = [-s, -c; c, -s];
  dd = dR * (b(1:2)' - a(1:2)');
  Ji(1:2,1:2) = -Rp;
  Ji(1:2,3) = dd';
  Ji(3,3) = -1;
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
