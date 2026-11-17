% actuator_dc_step_response.m — DC-motor+gearbox voltage step response.
%
%   [t, omega, i, out] = actuator_dc_step_response(V_final, T, dt, params)
%
%   Simulates a voltage step from 0 to V_final over duration T with sample
%   dt, using actuator_dc_gearbox (semi-implicit Euler). Returns time vector,
%   motor speed, armature current, and output (shaft) speed.
%
%   Pure Octave native code.

function [t, omega, i, out] = actuator_dc_step_response(V_final, T, dt, params)
  if nargin < 4, dt = 0.001; end
  if nargin < 3, T = 0.5; end
  if nargin < 2, V_final = 12; end
  if nargin < 1, V_final = 12; end
  t = 0:dt:T;
  omega = zeros(size(t)); i = zeros(size(t));
  out = zeros(size(t));
  for k = 2:length(t)
    [omega(k), i(k), info] = actuator_dc_gearbox(omega(k-1), i(k-1), V_final, dt, params);
    out(k) = info.output_speed;
  endfor
endfunction
