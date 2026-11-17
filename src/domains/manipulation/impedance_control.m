% impedance_control.m — Joint-space impedance / admittance controller.
%
%   tau = impedance_control(q, qd, q_des, qd_des, M, B, K, g)
%
%   Computes joint torques for impedance behavior:
%
%       M * (qdd_des - qdd) + B * (qd_des - qd) + K * (q_des - q) = tau
%
%   Solved for the torque command (treating qdd_des as the desired
%   acceleration and omitting measured qdd, i.e. a PD + gravity-compensation
%   impedance law):
%
%       tau = M * qdd_des + B * (qd_des - qd) + K * (q_des - q) + g(q)
%
%   q, qd        : (n x 1) measured joint position / velocity
%   q_des, qd_des: (n x 1) desired position / velocity
%   M, B, K      : (n x n) desired inertia, damping, stiffness (diagonal ok)
%   g            : (n x 1) gravity vector (or zeros)
%
%   Returns tau (n x 1).
%
%   Pure Octave native code.

function tau = impedance_control(q, qd, q_des, qd_des, M, B, K, g)
  if nargin < 8, g = zeros(size(q)); end
  if nargin < 7, K = eye(size(q)); end
  if nargin < 6, B = 2*eye(size(q)); end
  if nargin < 5, M = eye(size(q)); end
  if nargin < 4, qd_des = zeros(size(q)); end
  if nargin < 3, q_des = q; end

  q = reshape(q, [], 1); qd = reshape(qd, [], 1);
  q_des = reshape(q_des, [], 1); qd_des = reshape(qd_des, [], 1);
  g = reshape(g, [], 1);

  % desired acceleration: assume zero unless provided via qdd_des field below
  qdd_des = zeros(size(q));
  if nargin >= 9 && ~isempty(varargin{1})
    qdd_des = reshape(varargin{1}, [], 1);
  endif

  tau = M * qdd_des + B * (qd_des - qd) + K * (q_des - q) + g;
endfunction

% --- task-space impedance: F = M_d*xdd + B_d*xd + K_d*x, mapped to joints ---
function [tau, F] = impedance_control_task(J, q, qd, x_des, xd_des, M, B, K, g)
  if nargin < 8, g = zeros(size(q)); end
  if nargin < 7, K = eye(6); end
  if nargin < 6, B = 2*eye(6); end
  if nargin < 5, M = eye(6); end
  if nargin < 4, xd_des = zeros(6,1); end
  if nargin < 3, x_des = [J*q; zeros(3,1)]; end

  x = [J * q; zeros(3,1)];          % current task pose (pos + dummy orient)
  xd = J * qd;                      % current task twist
  x_err = x_des - x;
  xd_err = xd_des - xd;
  F = M * zeros(6,1) + B * xd_err + K * x_err;   % task-space force/torque
  tau = J' * F + reshape(g, [], 1);
endfunction
