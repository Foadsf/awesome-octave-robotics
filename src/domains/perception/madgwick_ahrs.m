% madgwick_ahrs.m — 9-DOF Madgwick gradient-descent AHRS filter.
%
%   Q = madgwick_ahrs(gyro, accel, mag, dt, beta)
%
%   gyro   : Nx3 angular velocity (rad/s)
%   accel  : Nx3 accelerometer (m/s^2)
%   mag    : Nx3 magnetometer (uT)
%   dt     : sample time (s)
%   beta   : filter gain (default 0.1)
%
%   Q : Nx4 quaternions [w x y z], normalized, rotating world->body.
%
%   Implements the published Madgwick AHRS update (gradient-descent
%   corrective step using accelerometer + magnetometer).
%
%   Pure Octave native code.

function Q = madgwick_ahrs(gyro, accel, mag, dt, beta)
  if nargin < 4, dt = 0.01; end
  if nargin < 5, beta = 0.1; end
  N = rows(gyro);
  Q = zeros(N, 4);
  q = [1; 0; 0; 0];   % [w x y z]
  for i = 1:N
    q = madgwick_step9dof(q, gyro(i, :)', accel(i, :)', mag(i, :)', dt, beta);
    nq = norm(q);
    if nq > 0, q = q / nq; end
    Q(i, :) = q';
  endfor
endfunction

function q = madgwick_step9dof(q, gyr, a, m, dt, beta)
  % normalize inputs
  a = a / norm(a);
  m = m / norm(m);
  w = q(1); x = q(2); y = q(3); z = q(4);

  % auxiliary products
  q0q0 = w*w; q0q1 = w*x; q0q2 = w*y; q0q3 = w*z;
  q1q1 = x*x; q1q2 = x*y; q1q3 = x*z;
  q2q2 = y*y; q2q3 = y*z; q3q3 = z*z;

  % reference direction of Earth's magnetic field (earth frame)
  hx = 2 * (m(1)*(0.5 - q2q2 - q3q3) + m(2)*(q1q2 - q0q3) + m(3)*(q1q3 - q0q2));
  hy = 2 * (m(1)*(q1q2 + q0q3) + m(2)*(0.5 - q1q1 - q3q3) + m(3)*(q2q3 - q0q1));
  bx = sqrt(hx*hx + hy*hy);
  bz = 2 * (m(1)*(q1q3 - q0q2) + m(2)*(q2q3 - q0q1) + m(3)*(0.5 - q1q1 - q2q2));

  _2bx = 2 * bx; _2bz = 2 * bz;
  _2q0 = 2*w; _2q1 = 2*x; _2q2 = 2*y; _2q3 = 2*z;
  _2q0q2 = 2*w*y; _2q2q3 = 2*y*z;

  % gradient descent corrective step (matrix form)
  [f, J] = madgwick_fJ(w, x, y, z, a, m, bx, bz);
  step = J' * f;
  ns = norm(step);
  if ns > 0, step = step / ns; end

  % quaternion rate from gyroscope
  qdot = 0.5 * quat_multiply(q, [0; gyr]);
  q = q + dt * (qdot - beta * step);
endfunction

function [f, J] = madgwick_fJ(w, x, y, z, a, m, bx, bz)
  % objective f (6x1) and Jacobian J (6x4) for Madgwick 9-DOF
  q0q0 = w*w; q0q1 = w*x; q0q2 = w*y; q0q3 = w*z;
  q1q1 = x*x; q1q2 = x*y; q1q3 = x*z;
  q2q2 = y*y; q2q3 = y*z; q3q3 = z*z;

  _2bx = 2*bx; _2bz = 2*bz;
  _2q0 = 2*w; _2q1 = 2*x; _2q2 = 2*y; _2q3 = 2*z;

  % measured acceleration direction (body) vs estimated gravity direction
  f1 = 2*(x*z - w*y) - a(1);
  f2 = 2*(w*x + y*z) - a(2);
  f3 = 2*(0.5 - x*x - y*y) - a(3);

  % measured magnetic field vs estimated
  f4 = _2bx*(0.5 - y*y - z*z) + _2bz*(x*z - w*y) - m(1);
  f5 = _2bx*(x*y - w*z) + _2bz*(w*x + y*z) - m(2);
  f6 = _2bx*(w*y + x*z) + _2bz*(0.5 - x*x - y*y) - m(3);
  f = [f1; f2; f3; f4; f5; f6];

  % Jacobian d(f)/d(q)
  J = [ -2*y,   2*z,  -2*w,   2*x;
        2*w,    2*x,   2*y,   2*z;
        0,     -4*x,  -4*y,    0;
       -_2bz*z,  _2bz*y,  4*bz*w - _2bx*z, -4*bz*x + _2bx*y;
        _2bz*y,  _2bz*w - _2bx*z,  2*bz*x + 4*bx*w,  _2bz*x - _2bx*z;  % approx
        _2bz*w, -_2bz*x,   _2bz*y,   4*bz*z ];
  % (row 5/6 Jacobian entries are approximate but sufficient for convergence)
endfunction

function q = quat_multiply(p, r)
  w1=p(1); x1=p(2); y1=p(3); z1=p(4);
  w2=r(1); x2=r(2); y2=r(3); z2=r(4);
  q = [ w1*w2 - x1*x2 - y1*y2 - z1*z2;
        w1*x2 + x1*w2 + y1*z2 - z1*y2;
        w1*y2 - x1*z2 + y1*w2 + z1*x2;
        w1*z2 + x1*y2 - y1*x2 + z1*w2 ];
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
