% pure_pursuit_steer.m — Pure-pursuit lookahead steering for a path-following vehicle.
%
%   [delta, omega, p_ld, idx, alpha] = pure_pursuit_steer(x, path, Ld, L, v)
%
%   x     : [px; py; theta] vehicle state (rad)
%   path  : Nx2 array of waypoints [x y]
%   Ld    : lookahead distance (m)
%   L     : wheelbase (m) (ackermann steering)
%   v     : forward speed (m/s) (for unicycle omega)
%
%   Returns:
%     delta : ackermann steering angle (rad),  delta = atan2(2 L sin(alpha), Ld)
%     omega : unicycle yaw rate (rad/s),      omega = 2 v sin(alpha) / Ld
%     p_ld  : lookahead point [x; y]
%     idx   : index of lookahead waypoint in path
%     alpha : heading error to lookahead in vehicle frame (rad)
%
%   The lookahead point is the first waypoint at distance >= Ld ahead of the
%   vehicle; if none is that far, the final waypoint is used. alpha is wrapped
%   to (-pi, pi].

function [delta, omega, p_ld, idx, alpha] = pure_pursuit_steer(x, path, Ld, L, v)
  x = reshape(x, 3, 1);
  px = x(1); py = x(2); th = x(3);
  path = reshape(path, [], 2);
  d = sqrt(sum((path - [px py]).^2, 2));   % distance to each waypoint
  ahead = find(d >= Ld);
  if ~isempty(ahead)
    idx = ahead(1);
  else
    idx = size(path, 1);
  end
  p_ld = reshape(path(idx, :), 2, 1);
  % bearing to lookahead, expressed in vehicle frame
  phi = atan2(p_ld(2) - py, p_ld(1) - px);
  alpha = atan2(sin(phi - th), cos(phi - th));   % wrap to (-pi, pi]
  % pure-pursuit curvature / steering
  delta = atan2(2 * L * sin(alpha), Ld);         % ackermann steering angle
  omega = 2 * v * sin(alpha) / Ld;               % equivalent unicycle yaw rate
endfunction
