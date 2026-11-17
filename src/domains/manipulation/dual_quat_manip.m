% dual_quat_manip.m — Dual-quaternion kinematics for manipulation (builds on core dual_quat).
%
%   [dq]       = dual_quat_manip("from_transform", T)        % (4x4) -> dual quat [r; d]
%   [T]        = dual_quat_manip("to_transform", dq)          % dual quat -> (4x4)
%   [dq_int]   = dual_quat_manip("sclerp", dq0, dq1, t)       % dual quaternion Sclerp
%   [T_interp] = dual_quat_manip("screw_motion", T0, T1, t)   % interpolate two transforms
%
%   Dual quaternion dq = [r; d] where r is the unit quaternion rotation and
%   d = 0.5 * t_quat * r encodes translation. Represents rigid-body motion
%   (rotation + translation) compactly, useful for manipulation and IK.
%
%   Pure Octave native code.

function varargout = dual_quat_manip(op, varargin)
  switch op
    case "from_transform"
      T = varargin{1};
      R = T(1:3,1:3); t = T(1:3,4);
      r = rotm_to_quat(R);
      tq = [0; t];                          % pure quaternion translation
      d = 0.5 * quat_mul(tq, r);
      varargout{1} = [r; d];
    case "to_transform"
      dq = varargin{1}; r = dq(1:4); d = dq(5:8);
      R = quat_to_rotm(r);
      % translation t = 2 * (d * r_conj)
      t = 2 * quat_mul(d, quat_conj(r));
      t = t(2:4);                           % vector part
      T = [R, t; 0 0 0 1];
      varargout{1} = T;
    case "sclerp"
      dq0 = varargin{1}; dq1 = varargin{2}; t = varargin{3};
      r0 = dq0(1:4); d0 = dq0(5:8);
      r1 = dq1(1:4); d1 = dq1(5:8);
      % ensure shortest path
      if dot(r0, r1) < 0, r1 = -r1; d1 = -d1; end
      % quaternion slerp
      om = acos(min(1,max(-1,dot(r0,r1))));
      if abs(om) < 1e-8
        rint = r0;
      else
        rint = (sin((1-t)*om)*r0 + sin(t*om)*r1) / sin(om);
      endif
      % dual part lerp + correction to stay on unit dual quaternion manifold
      dint = (1-t)*d0 + t*d1;
      % project: d = dint - 0.5*rint*(rint'*dint)  (approx re-normalization)
      rd = dot(rint, dint);
      dint = dint - 0.5 * (rd) * rint;
      varargout{1} = [rint; dint];
    case "screw_motion"
      T0 = varargin{1}; T1 = varargin{2}; t = varargin{3};
      dq0 = dual_quat_manip("from_transform", T0);
      dq1 = dual_quat_manip("from_transform", T1);
      dqint = dual_quat_manip("sclerp", dq0, dq1, t);
      varargout{1} = dual_quat_manip("to_transform", dqint);
    otherwise
      error("dual_quat_manip: unknown op '%s'.", op);
  endswitch
endfunction

function q = rotm_to_quat(R)
  % rotation matrix to unit quaternion [w; x; y; z]
  tr = trace(R);
  if tr > 0
    S = sqrt(tr + 1.0) * 2; w = 0.25 * S;
    x = (R(3,2) - R(2,3)) / S; y = (R(1,3) - R(3,1)) / S; z = (R(2,1) - R(1,2)) / S;
  elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
    w = (R(3,2) - R(2,3)) / S; x = 0.25 * S;
    y = (R(1,2) + R(2,1)) / S; z = (R(1,3) + R(3,1)) / S;
  elseif R(2,2) > R(3,3)
    S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
    w = (R(1,3) - R(3,1)) / S; x = (R(1,2) + R(2,1)) / S;
    y = 0.25 * S; z = (R(2,3) + R(3,2)) / S;
  else
    S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
    w = (R(2,1) - R(1,2)) / S; x = (R(1,3) + R(3,1)) / S;
    y = (R(2,3) + R(3,2)) / S; z = 0.25 * S;
  endif
  q = [w; x; y; z]; q = q / norm(q);
endfunction

function R = quat_to_rotm(q)
  q = q / norm(q); w=q(1); x=q(2); y=q(3); z=q(4);
  R = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
       2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
       2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
endfunction

function q = quat_conj(q)
  q = q; q(2:4) = -q(2:4);
endfunction

function c = quat_mul(a, b)
  % Hamilton product
  aw=a(1); ax=a(2); ay=a(3); az=a(4);
  bw=b(1); bx=b(2); by=b(3); bz=b(4);
  c = [aw*bw - ax*bx - ay*by - az*bz;
       aw*bx + ax*bw + ay*bz - az*by;
       aw*by - ax*bz + ay*bw + az*bx;
       aw*bz + ax*by - ay*bx + az*bw];
endfunction
