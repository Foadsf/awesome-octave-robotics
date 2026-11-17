% omni_wheel_kinematics.m — Omni / Mecanum wheel forward & inverse kinematics.
%
%   [J, Jpinv] = omni_wheel_kinematics("3wheel") | ("4wheel")
%   [w]        = omni_wheel_kinematics("forward", J, vx, vy, omega, wheel_radius)
%   [vel]      = omni_wheel_kinematics("inverse", Jpinv, w1, w2, ...)
%   [J, Jpinv] = omni_wheel_kinematics(nwheels, wheel_radius, wheel_poses)
%
%   Wheel i at pose [x_i; y_i; theta_i] (theta_i = wheel rolling heading, rad)
%   contributes to the wheel speed (rad/s) via:
%
%     r * w_i = cos(theta_i)*vx + sin(theta_i)*vy
%             + omega * (sin(theta_i)*x_i - cos(theta_i)*y_i)
%
%   so   w = J * [vx; vy; omega],   [vx; vy; omega] = Jpinv * w
%
%   J (nwheels x 3) already carries the 1/r factor, so forward is just J*vel.
%
%   Presets: 3-wheel omni (120 deg apart, tangent rolling), 4-wheel Mecanum
%   (rectangle corners, wheels aligned with robot axes).

function varargout = omni_wheel_kinematics(varargin)
  if nargin >= 1 && ischar(varargin{1})
    preset = lower(varargin{1});
    if strcmp(preset, "3wheel") || strcmp(preset, "4wheel")
      nwheels = str2double(preset(1));
      [J, Jpinv] = omni_wheel_kinematics_preset(nwheels);
      varargout{1} = J; varargout{2} = Jpinv; return;
    elseif strcmp(preset, "forward")
      J = varargin{2}; vx = varargin{3}; vy = varargin{4}; omega = varargin{5};
      vel = [vx; vy; omega];
      w = J * vel;            % J already carries 1/r -> wheel angular velocity (rad/s)
      varargout{1} = w; return;
    elseif strcmp(preset, "inverse")
      Jpinv = varargin{2};
      w = [varargin{3:end}(:)];
      varargout{1} = Jpinv * w; return;
    else
      error("omni_wheel_kinematics: unknown mode '%s'. Use '3wheel'/'4wheel'/'forward'/'inverse'.", preset);
    endif
  end

  % General path: omni_wheel_kinematics(nwheels, wheel_radius, wheel_poses)
  nwheels = varargin{1};
  r = 0.05;
  if nargin >= 2 && ~isempty(varargin{2}), r = varargin{2}; end
  poses = zeros(nwheels, 3);
  if nargin >= 3 && ~isempty(varargin{3})
    poses = varargin{3};
  else
    poses = omni_wheel_kinematics_preset_poses(nwheels);
  endif

  J = zeros(nwheels, 3);
  for i = 1:nwheels
    xi = poses(i, 1); yi = poses(i, 2); ti = poses(i, 3);
    J(i, 1) = cos(ti) / r;
    J(i, 2) = sin(ti) / r;
    J(i, 3) = (sin(ti) * xi - cos(ti) * yi) / r;
  endfor
  Jpinv = pinv(J);
  varargout{1} = J; varargout{2} = Jpinv;
endfunction

function [J, Jpinv] = omni_wheel_kinematics_preset(nwheels)
  r = 0.05;              % wheel radius (m)
  R = 0.2;               % wheel mounting radius from robot center (m)
  if nwheels == 3
    angles = [0, 2*pi/3, 4*pi/3];          % mount angles
    poses = zeros(3, 3);
    for i = 1:3
      a = angles(i);
      poses(i, 1) = R * cos(a);
      poses(i, 2) = R * sin(a);
      poses(i, 3) = a + pi/2;              % rolling heading tangent to circle
    endfor
  elseif nwheels == 4
    % 4-wheel Mecanum at rectangle corners; rolling headings along robot axes.
    poses = [ R,  R, 0;
              R, -R, pi/2;
             -R,  R, pi/2;
             -R, -R, 0];                   % [x y theta]
  else
    error("omni_wheel_kinematics_preset: only 3 or 4 wheels supported.");
  endif
  [J, Jpinv] = omni_wheel_kinematics(nwheels, r, poses);
endfunction

function poses = omni_wheel_kinematics_preset_poses(nwheels)
  r = 0.05; R = 0.2;
  poses = zeros(nwheels, 3);
  if nwheels == 3
    angles = [0, 2*pi/3, 4*pi/3];
    for i = 1:3
      a = angles(i);
      poses(i, :) = [R*cos(a), R*sin(a), a + pi/2];
    endfor
  elseif nwheels == 4
    poses = [ R,  R, 0;
              R, -R, pi/2;
             -R,  R, pi/2;
             -R, -R, 0];
  else
    error("omni_wheel_kinematics_preset_poses: only 3 or 4 wheels supported.");
  endif
endfunction
