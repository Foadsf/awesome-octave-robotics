% dual_quat.m — Dual quaternion algebra for rigid-body motion (struct of handles).
% A dual quaternion dq = (qr, qd) represents rotation qr (unit quaternion) and
% translation via qd = 0.5 * t_quat * qr, where t_quat = [0; px; py; pz].
%
%   D = dual_quat();           % create dual-quaternion operator object
%   dq = D.dq(qr, qd);         % build dual quaternion [qr(4); qd(4)]
%   dq = D.identity();         % (qr=[1;0;0;0], qd=[0;0;0;0])
%   dq = D.normalize(dq);      % normalize rotation part, re-orthonormalize dual
%   dm = D.mul(dq1, dq2);      % dual quaternion (Hamilton) product
%   dc = D.conj(dq);           % conjugate
%   di = D.inv(dq);            % inverse (conjugate for unit dual quats)
%   dq = D.from_se3(T);        % SE(3) 4x4 -> dual quaternion
%   T = D.to_se3(dq);          % dual quaternion -> SE(3) 4x4
%   p2 = D.transform(dq, p);   % rigid transform point(s) p (3xN)
%   dqi = D.sclerp(dq1, dq2, t); % screw linear interpolation in [0,1]
%   [qr, qd] = D.parts(dq)

function s = dual_quat()
  s.dq        = @dq_build;
  s.identity  = @dq_identity;
  s.normalize = @dq_normalize;
  s.mul       = @dq_mul;
  s.conj      = @dq_conj;
  s.inv       = @dq_inv;
  s.from_se3  = @dq_from_se3;
  s.to_se3    = @dq_to_se3;
  s.transform = @dq_transform;
  s.sclerp    = @dq_sclerp;
  s.parts     = @dq_parts;
endfunction

function dq = dq_build(qr, qd)
  qr = reshape(qr, 4, 1); qd = reshape(qd, 4, 1);
  dq = [qr; qd];
endfunction

function dq = dq_identity()
  dq = [1; 0; 0; 0; 0; 0; 0; 0];
endfunction

function dqn = dq_normalize(dq)
  dq = reshape(dq, 8, 1);
  qr = dq(1:4); qd = dq(5:8);
  qr = qr / norm(qr);
  % re-impose qr . qd = 0 (orthogonality of rotation & translation parts)
  qd = qd - qr * (qr' * qd);
  dqn = [qr; qd];
endfunction

function dm = dq_mul(a, b)
  a = reshape(a, 8, 1); b = reshape(b, 8, 1);
  ar = a(1:4); ad = a(5:8);
  br = b(1:4); bd = b(5:8);
  Q = quat_ops();  % quaternion helper (struct of handles)
  cr = Q.mul(ar, br);
  cd = Q.mul(ar, bd) + Q.mul(ad, br);
  dm = [cr; cd];
endfunction

function dc = dq_conj(dq)
  dq = reshape(dq, 8, 1);
  Q = quat_ops();
  dc = [Q.conj(dq(1:4)); Q.conj(dq(5:8))];
endfunction

function di = dq_inv(dq)
  % inverse of a unit dual quaternion = its conjugate
  di = dq_conj(dq);
endfunction

function dq = dq_from_se3(T)
  T = reshape(T, 4, 4);
  R = T(1:3, 1:3);
  p = T(1:3, 4);
  Q = quat_ops();
  qr = Q.from_rotmat(R);
  % translation quaternion [0; p]
  tq = [0; p];
  qd = 0.5 * Q.mul(tq, qr);
  dq = [qr; qd];
endfunction

function T = dq_to_se3(dq)
  dq = reshape(dq, 8, 1);
  qr = dq(1:4); qd = dq(5:8);
  Q = quat_ops();
  R = Q.rotmat(qr);
  % translation: t = 2 * (qd * conj(qr)) vector part
  tq = 2 * Q.mul(qd, Q.conj(qr));
  p = tq(2:4);
  T = [R, p; 0 0 0 1];
endfunction

function p2 = dq_transform(dq, p)
  T = dq_to_se3(dq);
  G = se3();
  p2 = G.transform(T, p);
endfunction

function dqi = dq_sclerp(dq1, dq2, t)
  % screw linear interpolation (constant screw motion) from dq1 to dq2
  dq1 = dq_normalize(reshape(dq1, 8, 1));
  dq2 = dq_normalize(reshape(dq2, 8, 1));
  % shortest path: negate dq2 if rotation parts point opposite ways
  Q = quat_ops();
  if Q.mul(Q.conj(dq1(1:4)), dq2(1:4))(1) < 0
    dq2 = -dq2;
  end
  T1 = dq_to_se3(dq1);
  T2 = dq_to_se3(dq2);
  G = se3();
  T_rel = G.compose(G.inv(T1), T2);
  xi = G.log(T_rel);            % screw twist
  Tt = G.compose(T1, G.exp(t * xi));
  dqi = dq_from_se3(Tt);
endfunction

function [qr, qd] = dq_parts(dq)
  dq = reshape(dq, 8, 1);
  qr = dq(1:4); qd = dq(5:8);
endfunction
