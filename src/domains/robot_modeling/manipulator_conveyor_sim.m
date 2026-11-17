% manipulator_conveyor_sim.m — Manipulator picking parts from a conveyor belt.
%
%   [part_traj, ee_traj, metrics] = manipulator_conveyor_sim(T, dt, v_belt)
%
%   A part rides a conveyor at constant velocity v_belt. A 2-link planar
%   manipulator (uses fk_2link from src/domains/manipulation) tracks the part
%   with a simple proportional controller in task space and "picks" it when
%   within a threshold. Reports pick success, pick time, and tracking error.
%
%   Pure Octave native code.

function [part_traj, ee_traj, metrics] = manipulator_conveyor_sim(T, dt, v_belt)
  if nargin < 3, v_belt = 0.15; end
  if nargin < 2, dt = 0.01; end
  if nargin < 1, T = 4.0; end

  % manipulator link lengths
  l1 = 0.5; l2 = 0.5;
  q = [0.5; 0.3];                      % initial joint angles

  % conveyor part starts at x=0, y=0.6 (within reach), moves in +x
  part = [0.0; 0.6; 0.0];              % [x y theta]
  pick_threshold = 0.05;
  picked = false;
  pick_time = NaN;

  K = round(T/dt) + 1;
  part_traj = zeros(K, 3);
  ee_traj = zeros(K, 2);
  err_sum = 0;

  for k = 1:K
    t = (k-1)*dt;
    % conveyor motion
    part(1) = v_belt * t;
    part_traj(k,:) = part';
    % desired end-effector = part position
    x_des = part(1:2);
    % current forward kinematics
    [ee, ~, ~] = fk_2link(q, l1, l2);
    ee_traj(k,:) = ee';
    % task-space error
    e = x_des - ee;
    err_sum = err_sum + norm(e);
    % simple P controller -> joint velocities via Jacobian transpose
    J = [ -l1*sin(q(1)) - l2*sin(q(1)+q(2)), -l2*sin(q(1)+q(2));
           l1*cos(q(1)) + l2*cos(q(1)+q(2)),  l2*cos(q(1)+q(2)) ];
    qd = 1.0 * (J' * e);               % gain 1.0
    q = q + dt * qd;                   % integrate
    % pick check
    if ~picked && norm(e) < pick_threshold
      picked = true; pick_time = t;
    endif
  endfor

  metrics.picked = picked;
  metrics.pick_time = pick_time;
  metrics.mean_tracking_error = err_sum / K;
  metrics.final_error = norm(part_traj(K,1:2) - ee_traj(K,:));
endfunction
