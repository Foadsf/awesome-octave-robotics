% cascade_pid_arm.m — Multi-loop cascade PID (position -> velocity -> current).
%
%   [tau, info] = cascade_pid_arm(q, qd, qdd, q_ref, qd_ref, gains, dt)
%
%   q, qd, qdd   : current position, velocity, acceleration (nj x 1)
%   q_ref, qd_ref: reference position, velocity (nj x 1)
%   gains        : struct with .Kp, .Kd, .Ki (position), .Kpv, .Kiv (velocity),
%                  .Kpc (current), .tau_max
%   dt           : step (s)
%
%   Cascade: outer position loop -> velocity reference; middle velocity loop
%   -> current reference; inner current loop -> torque. Returns torque tau
%   and info (errors, integrals).
%
%   Pure Octave native code.

function [tau, info] = cascade_pid_arm(q, qd, qdd, q_ref, qd_ref, gains, dt)
  if nargin < 6, gains = struct(); end
  if nargin < 7, dt = 0.01; end
  if nargin < 6, gains = struct(); end
  if nargin < 7, dt = 0.01; end
  nj = numel(q);
  Kp = pc_pick(gains, "Kp", 50 * ones(nj, 1));
  Kd = pc_pick(gains, "Kd", 5 * ones(nj, 1));
  Ki = pc_pick(gains, "Ki", 1 * ones(nj, 1));
  Kpv = pc_pick(gains, "Kpv", 10 * ones(nj, 1));
  Kiv = pc_pick(gains, "Kiv", 2 * ones(nj, 1));
  Kpc = pc_pick(gains, "Kpc", 5 * ones(nj, 1));
  tau_max = pc_pick(gains, "tau_max", 10 * ones(nj, 1));
  persistent int_pos int_vel;
  if isempty(int_pos), int_pos = zeros(nj, 1); end
  if isempty(int_vel), int_vel = zeros(nj, 1); end
  persistent prev_qerr;
  if isempty(prev_qerr), prev_qerr = zeros(nj, 1); end

  % outer position loop: position error -> velocity reference
  qerr = q_ref - q;
  int_pos = int_pos + qerr * dt;
  % anti-windup
  int_pos = min(max(int_pos, -5), 5);
  v_ref = qd_ref + Kp .* qerr + Ki .* int_pos;
  % velocity loop: velocity error -> current reference
  verr = v_ref - qd;
  int_vel = int_vel + verr * dt;
  int_vel = min(max(int_vel, -5), 5);
  i_ref = Kpv .* verr + Kiv .* int_vel;
  % current loop: current reference -> torque
  tau = Kpc .* i_ref;
  % feedforward acceleration term
  tau = tau + qdd;  % simplified inertia=1
  % saturate
  tau = min(max(tau, -tau_max), tau_max);

  info.pos_error = qerr;
  info.vel_error = verr;
  info.int_pos = int_pos;
  info.int_vel = int_vel;
  info.tau = tau;
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
