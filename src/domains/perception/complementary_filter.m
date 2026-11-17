% complementary_filter.m — Attitude estimation from gyro + accelerometer.
%
%   [angles, R] = complementary_filter(gyro, accel, dt, alpha, angles0)
%
%   gyro   : Nx3 angular rate [wx wy wz] (rad/s)
%   accel  : Nx3 specific force [ax ay az] (m/s^2), gravity + motion
%   dt     : sample step (s)
%   alpha  : gyro weight in (0,1) (e.g. 0.98); 1-alpha accelerometer weight
%   angles0: [roll pitch yaw] initial attitude (rad)
%
%   Complementary filter:
%     roll/pitch  = alpha*(gyro-integrated) + (1-alpha)*(accel gravity tilt)
%     yaw         = gyro integrated only (unobservable from accel)
%   Returns angles (Nx3) and rotation matrices R (3x3xN), R(:,:,k) =
%   Rz(yaw) Ry(pitch) Rx(roll). Real numeric attitude estimation.

function [angles, R] = complementary_filter(gyro, accel, dt, alpha, angles0)
  gyro = reshape(gyro, [], 3); accel = reshape(accel, [], 3);
  N = size(gyro, 1);
  if nargin < 5 || isempty(angles0), angles0 = [0; 0; 0]; end
  if nargin < 4, alpha = 0.98; end
  if nargin < 3, dt = 0.01; end
  angles = zeros(N, 3); R = zeros(3, 3, N);
  ang = reshape(angles0, 1, 3);   % [roll pitch yaw]
  for k = 1:N
    % predict: integrate gyro (rad/s -> rad)
    ang_pred = ang + gyro(k, :) * dt;
    % correct roll/pitch from accelerometer (gravity vector in body frame)
    ax = accel(k, 1); ay = accel(k, 2); az = accel(k, 3);
    roll_acc  = atan2(ay, az);
    pitch_acc = atan2(-ax, sqrt(ay^2 + az^2));
    ang(1) = alpha * ang_pred(1) + (1 - alpha) * roll_acc;
    ang(2) = alpha * ang_pred(2) + (1 - alpha) * pitch_acc;
    ang(3) = ang_pred(3);           % yaw: gyro only
    angles(k, :) = ang;
    R(:,:,k) = rpy_rot(ang(1), ang(2), ang(3));
    % keep angles wrapped
    ang = wrap(ang);
  end
endfunction

function R = rpy_rot(r, p, y)
  % R = Rz(y) Ry(p) Rx(r)
  cr=cos(r); sr=sin(r); cp=cos(p); sp=sin(p); cy=cos(y); sy=sin(y);
  Rx = [1 0 0; 0 cr -sr; 0 sr cr];
  Ry = [cp 0 sp; 0 1 0; -sp 0 cp];
  Rz = [cy -sy 0; sy cy 0; 0 0 1];
  R = Rz * Ry * Rx;
endfunction

function a = wrap(a)
  a = rem(a + pi, 2*pi) - pi;
endfunction
