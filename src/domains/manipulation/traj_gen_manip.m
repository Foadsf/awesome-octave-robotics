% traj_gen_manip.m — Joint-space trajectory generation (quintic spline).
%
%   [q, qd, qdd] = traj_gen_manip(q_start, q_goal, T, dt, via_points)
%
%   Generates a smooth joint-space trajectory from q_start to q_goal over
%   duration T, passing through optional via_points at equal time intervals.
%   Each segment is a quintic polynomial enforcing zero velocity/acceleration
%   at segment boundaries (start, via points, goal) for jerk-limited motion.
%
%   q_start, q_goal : (n x 1) joint vectors
%   T              : total duration (s)
%   dt             : sample time (s)
%   via_points     : (n x m) via points (optional)
%
%   Returns q (K x n), qd (K x n), qdd (K x n) sampled at 0:dt:T.
%
%   Pure Octave native code.

function [q, qd, qdd] = traj_gen_manip(q_start, q_goal, T, dt, via_points)
  if nargin < 5, via_points = []; end
  if nargin < 4, dt = 0.01; end
  if nargin < 3, T = 2.0; end
  if nargin < 2, q_goal = zeros(size(q_start)); end
  q_start = reshape(q_start, [], 1); q_goal = reshape(q_goal, [], 1);
  n = length(q_start);

  % Build the sequence of waypoints and their times
  nvia = size(via_points, 2);
  seg_times = linspace(0, T, nvia + 2);
  waypts = [q_start, via_points, q_goal];   % n x (nvia+2)
  nseg = nvia + 1;

  % quintic coefficients per segment per joint: q(t)=c0+c1 t+...+c5 t^5
  % with q(0)=q0, qd(0)=0, qdd(0)=0, q(tf)=qf, qd(tf)=0, qdd(tf)=0
  K = round(T/dt) + 1;
  q = zeros(K, n); qd = zeros(K, n); qdd = zeros(K, n);
  for s = 1:nseg
    q0 = waypts(:, s);      % (n x 1)
    qf = waypts(:, s+1);    % (n x 1)
    tf = seg_times(s+1) - seg_times(s);
    % Boundary-condition matrix (same for every joint):
    % c3 tf^3 + c4 tf^4 + c5 tf^5 = dq
    % 3 c3 tf^2 + 4 c4 tf^3 + 5 c5 tf^4 = 0
    % 6 c3 tf   + 12 c4 tf^2 + 20 c5 tf^3 = 0
    M = [tf^3, tf^4, tf^5;
         3*tf^2, 4*tf^3, 5*tf^4;
         6*tf, 12*tf^2, 20*tf^3];
    Binv = M \ eye(3);   % (3x3) maps [dq;0;0] -> [c3;c4;c5]
    c3 = zeros(1, n); c4 = zeros(1, n); c5 = zeros(1, n);
    for j = 1:n
      cc = Binv * [qf(j) - q0(j); 0; 0];
      c3(j) = cc(1); c4(j) = cc(2); c5(j) = cc(3);
    endfor
    t0 = seg_times(s);
    % sample this segment
    seg_k = round(tf/dt);
    for k = 0:seg_k
      tt = t0 + k*dt;
      if tt > T, break; end
      tau = tt - t0;
      idx = k + 1 + (s-1)*seg_k;
      if idx > K, break; end
      q(idx,:) = q0' + c3*tau^3 + c4*tau^4 + c5*tau^5;
      qd(idx,:) = (3*c3*tau^2 + 4*c4*tau^3 + 5*c5*tau^4)';
      qdd(idx,:) = (6*c3*tau + 12*c4*tau^2 + 20*c5*tau^3)';
    endfor
  endfor
  % ensure endpoint exactly
  q(K,:) = q_goal';
endfunction
