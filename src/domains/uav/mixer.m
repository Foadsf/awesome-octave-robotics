% mixer.m — Quadrotor control allocation (mixing) for a "+" configuration.
%
%   [speed, thrust] = mixer(wrench, M, max_thrust)
%
%   wrench     : [T; tau_phi; tau_theta; tau_psi] desired total thrust + torques
%   M          : 4x4 mixing matrix (maps 4 motor thrusts -> wrench). Default "+" config.
%   max_thrust : saturation per motor (N)
%
%   motor_thrust = pinv(M) * wrench  (Moore-Penrose least-squares allocation),
%   saturated to [0, max_thrust], motor speed ~ sqrt(thrust). Real control
%   allocation used on multirotors.

function [speed, thrust] = mixer(wrench, M, max_thrust)
  wrench = reshape(wrench, 4, 1);
  if nargin < 2 || isempty(M)
    l = 0.25;  % motor arm length (m)
    M = [1, 1, 1, 1;
         0, -l, 0, l;
         l, 0, -l, 0;
        -1, 1, -1, 1];
  end
  if nargin < 3, max_thrust = 10.0; end
  thrust = pinv(M) * wrench;      % least-squares control allocation
  thrust = min(max(thrust, 0), max_thrust);  % saturate: motors only push
  speed = sqrt(thrust);           % speed ~ sqrt(thrust) (quadrotor)
endfunction
