% actuator_dc_gearbox.m — DC motor + gearbox electromechanical model, one step.
%
%   [omega_next, i_next, info] = actuator_dc_gearbox(omega, i, V, dt, params)
%
%   DC motor equations:
%     L * di/dt   = V - R*i - ke*omega
%     J * domega/dt = kt*i - b*omega - tau_load   (motor shaft)
%   Gearbox ratio N (output speed = omega / N, output torque = N * kt * i).
%
%   Integrated with semi-implicit Euler (real numerical solution).
%
%   omega : motor shaft speed (rad/s)
%   i     : armature current (A)
%   V     : applied voltage (V)
%   dt    : step (s)
%   params: struct with R, L, ke, kt, J, b, N (gear ratio), tau_load
%
%   Returns omega_next, i_next, info (output_speed, output_torque, electrical_power).
%
%   Pure Octave native code.

function [omega_next, i_next, info] = actuator_dc_gearbox(omega, i, V, dt, params)
  if nargin < 5, params = struct(); end
  R = pick(params, "R", 1.0);
  L = pick(params, "L", 0.001);
  ke = pick(params, "ke", 0.01);
  kt = pick(params, "kt", 0.01);
  J = pick(params, "J", 0.01);
  b = pick(params, "b", 0.001);
  N = pick(params, "N", 10);
  tau_load = pick(params, "tau_load", 0.0);

  % semi-implicit Euler: update current first, then speed
  di = (V - R*i - ke*omega) / L;
  i_next = i + dt * di;
  domega = (kt*i_next - b*omega - tau_load) / J;
  omega_next = omega + dt * domega;

  info.output_speed = omega_next / N;
  info.output_torque = N * kt * i_next;
  info.electrical_power = V * i_next;
  info.motor_torque = kt * i_next;
endfunction

function v = pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
