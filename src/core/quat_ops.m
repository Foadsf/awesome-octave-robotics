% quat_ops.m — Unit quaternion algebra (struct of function handles).
% Convention: q = [w; x; y; z], scalar-first, Hamilton product,
% active rotation of column vectors: p' = q * p * conj(q).
%
%   Q = quat_ops();            % create quaternion operator object
%   q = Q.q(w, x, y, z);       % build quaternion (4x1)
%   q = Q.identity();          % [1;0;0;0]
%   q = Q.normalize(q);        % unit quaternion
%   qc = Q.conj(q);            % conjugate
%   qi = Q.inv(q);             % inverse (= conj for unit)
%   qm = Q.mul(q1, q2);        % Hamilton product
%   R = Q.rotmat(q);           % 3x3 rotation matrix
%   q = Q.from_rotmat(R);      % rotation matrix -> quaternion
%   q = Q.from_axis_angle(axis, angle)
%   q = Q.from_rpy(roll, pitch, yaw)
%   [axis, angle] = Q.to_axis_angle(q)
%   [r, p, y] = Q.to_rpy(q)
%   qs = Q.slerp(q1, q2, t);   % spherical linear interpolation
%   qe = Q.error(q1, q2);      % relative rotation quaternion (shortest arc)
%   [w, x, y, z] = Q.components(q)

function s = quat_ops()
  s.q              = @quat_q;
  s.identity       = @quat_identity;
  s.normalize      = @quat_normalize;
  s.conj           = @quat_conj;
  s.inv            = @quat_inv;
  s.mul            = @quat_mul;
  s.rotmat         = @quat_rotmat;
  s.from_rotmat    = @quat_from_rotmat;
  s.from_axis_angle = @quat_from_axis_angle;
  s.from_rpy       = @quat_from_rpy;
  s.to_axis_angle  = @quat_to_axis_angle;
  s.to_rpy         = @quat_to_rpy;
  s.slerp          = @quat_slerp;
  s.error          = @quat_error;
  s.components     = @quat_components;
endfunction

function q = quat_q(w, x, y, z)
  q = [w; x; y; z];
endfunction

function q = quat_identity()
  q = [1; 0; 0; 0];
endfunction

function qn = quat_normalize(q)
  q = reshape(q, 4, 1);
  n = norm(q);
  if n < 1e-14
    qn = [1; 0; 0; 0];
  else
    qn = q / n;
  end
endfunction

function qc = quat_conj(q)
  q = reshape(q, 4, 1);
  qc = [q(1); -q(2); -q(3); -q(4)];
endfunction

function qi = quat_inv(q)
  q = reshape(q, 4, 1);
  qi = quat_conj(q) / (q' * q);
endfunction

function qm = quat_mul(a, b)
  a = reshape(a, 4, 1); b = reshape(b, 4, 1);
  aw = a(1); ax = a(2); ay = a(3); az = a(4);
  bw = b(1); bx = b(2); by = b(3); bz = b(4);
  qm = [aw*bw - ax*bx - ay*by - az*bz;
        aw*bx + ax*bw + ay*bz - az*by;
        aw*by - ax*bz + ay*bw + az*bx;
        aw*bz + ax*by - ay*bx + az*bw];
endfunction

function R = quat_rotmat(q)
  q = quat_normalize(reshape(q, 4, 1));
  w = q(1); x = q(2); y = q(3); z = q(4);
  R = [1 - 2*(y^2 + z^2),     2*(x*y - w*z),     2*(x*z + w*y);
       2*(x*y + w*z),     1 - 2*(x^2 + z^2),     2*(y*z - w*x);
       2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x^2 + y^2)];
endfunction

function q = quat_from_rotmat(R)
  R = reshape(R, 3, 3);
  % Sheppard's method
  tr = trace(R);
  if tr > 0
    S = sqrt(tr + 1.0) * 2;
    w = 0.25 * S;
    x = (R(3, 2) - R(2, 3)) / S;
    y = (R(1, 3) - R(3, 1)) / S;
    z = (R(2, 1) - R(1, 2)) / S;
  elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
    w = (R(3, 2) - R(2, 3)) / S;
    x = 0.25 * S;
    y = (R(1, 2) + R(2, 1)) / S;
    z = (R(1, 3) + R(3, 1)) / S;
  elseif R(2,2) > R(3,3)
    S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
    w = (R(1, 3) - R(3, 1)) / S;
    x = (R(1, 2) + R(2, 1)) / S;
    y = 0.25 * S;
    z = (R(2, 3) + R(3, 2)) / S;
  else
    S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
    w = (R(2, 1) - R(1, 2)) / S;
    x = (R(1, 3) + R(3, 1)) / S;
    y = (R(2, 3) + R(3, 2)) / S;
    z = 0.25 * S;
  end
  q = quat_normalize([w; x; y; z]);
endfunction

function q = quat_from_axis_angle(axis, angle)
  axis = reshape(axis, 3, 1);
  axis = axis / norm(axis);
  h = angle / 2;
  q = [cos(h); axis * sin(h)];
  q = quat_normalize(q);
endfunction

function q = quat_from_rpy(roll, pitch, yaw)
  % ZYX extrinsic -> quaternion
  cr = cos(roll/2); sr = sin(roll/2);
  cp = cos(pitch/2); sp = sin(pitch/2);
  cy = cos(yaw/2); sy = sin(yaw/2);
  q = [cr*cp*cy + sr*sp*sy;
       sr*cp*cy - cr*sp*sy;
       cr*sp*cy + sr*cp*sy;
       cr*cp*sy - sr*sp*cy];
  q = quat_normalize(q);
endfunction

function [axis, angle] = quat_to_axis_angle(q)
  q = quat_normalize(reshape(q, 4, 1));
  w = q(1);
  w = max(-1, min(1, w));
  angle = 2 * acos(w);
  s = sin(angle/2);
  if s < 1e-12
    axis = [1; 0; 0];
  else
    axis = q(2:4) / s;
  end
endfunction

function [r, p, y] = quat_to_rpy(q)
  R = quat_rotmat(q);
  % ZYX extrinsic
  r = atan2(R(3,2), R(3,3));
  p = asin(max(-1, min(1, -R(3,1))));
  y = atan2(R(2,1), R(1,1));
endfunction

function qs = quat_slerp(q1, q2, t)
  q1 = quat_normalize(reshape(q1, 4, 1));
  q2 = quat_normalize(reshape(q2, 4, 1));
  % shortest path
  if q1' * q2 < 0, q2 = -q2; end
  dot = q1' * q2;
  dot = max(-1, min(1, dot));
  if abs(1 - dot) < 1e-12
    qs = q1;
    return;
  end
  theta = acos(dot);
  s = sin(theta);
  qs = (sin((1 - t) * theta) / s) * q1 + (sin(t * theta) / s) * q2;
endfunction

function qe = quat_error(q1, q2)
  % relative rotation q_err such that q2 ~= q_err * q1 (shortest arc)
  qe = quat_mul(quat_conj(q1), q2);
  qe = quat_normalize(qe);
  if qe(1) < 0, qe = -qe; end
endfunction

function [w, x, y, z] = quat_components(q)
  q = reshape(q, 4, 1);
  w = q(1); x = q(2); y = q(3); z = q(4);
endfunction
