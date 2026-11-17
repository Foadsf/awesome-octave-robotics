% visual_odom_posegraph.m — 2D visual odometry front-end + pose-graph optimization.
%
%   poses = visual_odom_posegraph(odom, loop_closures, opts)
%
%   odom : Nx3 relative motions [dx dtheta] per step (dx = forward, dtheta = yaw)
%          first row is the initial absolute pose [x y theta]
%   loop_closures : Kx4 [i j dx dtheta] relative constraint between pose i and j
%   opts   : struct with optional fields
%     .iters : Gauss-Newton iterations (default 20)
%     .lambda: damping (default 1e-3)
%
%   Returns optimized absolute poses (Nx3 [x y theta]) in world frame.
%
%   Pose-graph: minimize sum over edges of squared SE(2) error via
%   Gauss-Newton with damping. Real numerical optimization.
%
%   Pure Octave native code.

function poses = visual_odom_posegraph(odom, loop_closures, opts)
  if nargin < 3, opts = struct(); end
  if nargin < 2, loop_closures = zeros(0, 4); end
  if ~isstruct(opts), opts = struct(); end
  iters = pc_pick(opts, "iters", 20);
  lambda = pc_pick(opts, "lambda", 1e-3);

  N = rows(odom);
  % build absolute poses from odometry (first row = initial pose)
  poses = zeros(N, 3);
  poses(1, :) = odom(1, :);
  for i = 2:N
    % integrate relative motion in current frame
    th = poses(i-1, 3);
    R = [cos(th), -sin(th); sin(th), cos(th)];
    dpos = R * [odom(i, 1); 0];   % forward motion along heading
    poses(i, 1:2) = poses(i-1, 1:2) + dpos';
    poses(i, 3) = poses(i-1, 3) + odom(i, 2);
    poses(i, 3) = atan2(sin(poses(i,3)), cos(poses(i,3)));
  endfor

  % --- Gauss-Newton pose graph optimization ---
  % edges: odometry (i-1 -> i) + loop closures
  edges = [];
  for i = 2:N
    edges = [edges; i-1, i, 0, 0, 0];   % placeholder, use odom residual below
  endfor
  % store odometry edge measurements
  odo_meas = zeros(N-1, 3);
  for i = 2:N
    odo_meas(i-1, :) = [odom(i, 1), 0, odom(i, 2)];  % [dx, 0, dtheta]
  endfor

  for it = 1:iters
    % residuals and Jacobians
    r = []; J = [];
    % odometry edges
    for i = 2:N
      a = poses(i-1, :); b = poses(i, :);
      rel = relative_pose(a, b);   % [dx dy dtheta] of b relative to a
      meas = [odom(i,1); 0; odom(i,2)];
      res = rel - meas;
      res(3) = atan2(sin(res(3)), cos(res(3)));
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
      rel = relative_pose(a, b);
      res = rel - meas;
      res(3) = atan2(sin(res(3)), cos(res(3)));
      [Ji, Jj] = pose_jacobians(a, b);
      r = [r; res];
      Jrow = zeros(3, 3*N);
      Jrow(:, 3*i-2:3*i) = Ji;
      Jrow(:, 3*j-2:3*j) = Jj;
      J = [J; Jrow];
    endfor

    % solve (J'J + lambda I) dx = -J'r
    H = J' * J + lambda * eye(3*N);
    g = J' * r;
    dx = -H \ g;
    % apply update
    for i = 1:N
      poses(i, 1:2) = poses(i, 1:2) + dx(3*i-2:3*i-1)';
      poses(i, 3) = poses(i, 3) + dx(3*i);
      poses(i, 3) = atan2(sin(poses(i,3)), cos(poses(i,3)));
    endfor
    if norm(dx) < 1e-8, break; end
  endfor
endfunction

function rel = relative_pose(a, b)
  % relative pose of b w.r.t. a: [dx dy dtheta]
  th = a(3);
  R = [cos(th), -sin(th); sin(th), cos(th)];
  dp = R' * (b(1:2)' - a(1:2)');
  rel = [dp; atan2(sin(b(3)-a(3)), cos(b(3)-a(3)))];
endfunction

function [Ji, Jj] = pose_jacobians(a, b)
  % Jacobians of relative_pose(a,b) wrt a and b (3x3 each)
  th = a(3);
  c = cos(th); s = sin(th);
  % d(rel)/d(b): R'  [0];  d(dtheta)/d(b)=[0 0 1]
  Ji = zeros(3,3); Jj = zeros(3,3);
  Rp = [c, s; -s, c];  % R'
  Jj(1:2,1:2) = Rp;
  Jj(1:2,3) = zeros(2,1);
  Jj(3,3) = 1;
  % d(rel)/d(a):
  dp = Rp * (b(1:2)' - a(1:2)');   % [dx dy]
  % derivative of R' wrt theta times delta
  dR = [-s, -c; c, -s];  % d(R')/dtheta = [[-s,-c],[c,-s]]? check
  % R' = [[c,s],[-s,c]], dR'/dth = [[-s,-c],[c,-s]]
  dd = dR * (b(1:2)' - a(1:2)');
  Ji(1:2,1:2) = -Rp;
  Ji(1:2,3) = dd';
  Ji(3,1:2) = [0, 0];
  Ji(3,3) = -1;
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
