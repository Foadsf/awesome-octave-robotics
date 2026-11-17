% stanley.m — Stanley lateral controller for automated driving (lane keeping).
%
%   [delta, cte, e_psi] = stanley(x, y, yaw, v, path, k, soft)
%
%   x, y, yaw   : vehicle pose (m, m, rad)
%   v           : forward speed (m/s)
%   path        : Nx2 reference line [x y]
%   k           : cross-track gain
%   soft        : softening speed to avoid singularity at v=0
%
%   delta = e_psi + atan(k * cte / (v + soft))   (Stanley control law)
%   where cte is the signed cross-track error to the nearest path point and
%   e_psi is the heading error to the path tangent at that point. Real,
%   production-grade lane-keeping controller.

function [delta, cte, e_psi] = stanley(x, y, yaw, v, path, k, soft)
  path = reshape(path, [], 2);
  if nargin < 6, k = 1.0; end
  if nargin < 7, soft = 0.5; end
  % nearest path point
  d = sum((path - [x y]).^2, 2);
  [~, i] = min(d);
  if i < size(path,1)
    tan = path(i+1,:) - path(i,:);
  else
    tan = path(i,:) - path(i-1,:);
  end
  e_psi = wrap_to_pi(atan2(tan(2), tan(1)) - yaw);
  % signed cross-track error
  cte = -sin(yaw)*(path(i,1) - x) + cos(yaw)*(path(i,2) - y);
  delta = e_psi + atan(k * cte / (v + soft));
endfunction

function a = wrap_to_pi(a)
  a = rem(a + pi, 2*pi) - pi;
endfunction
