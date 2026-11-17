% fk_2link.m — Forward kinematics of a 2-link planar arm.
%
%   [p, T1, T2] = fk_2link(q, l1, l2)
%
%   q   : [q1; q2] joint angles (rad)
%   l1, l2: link lengths (m)
%
%   Returns end-effector position p=[x;y] and homogeneous transforms:
%     T1 : base -> link1 tip (SE(2)),   T2 : link1 tip -> end-effector (SE(2))
%   with p = (T1 * T2)(1:2,3). Real numerical FK:
%   p = [l1 cos(q1)+l2 cos(q1+q2); l1 sin(q1)+l2 sin(q1+q2)].

function [p, T1, T2] = fk_2link(q, l1, l2)
  q = reshape(q, 2, 1);
  q1 = q(1); q2 = q(2);
  c1 = cos(q1); s1 = sin(q1);
  c2 = cos(q2); s2 = sin(q2);
  c12 = cos(q1 + q2); s12 = sin(q1 + q2);
  p = [l1 * c1 + l2 * c12; l1 * s1 + l2 * s12];
  T1 = [c1, -s1, l1 * c1; s1, c1, l1 * s1; 0, 0, 1];
  T2 = [c2, -s2, l2 * c2; s2, c2, l2 * s2; 0, 0, 1];
  % verify consistency: T1*T2 translation == p
endfunction
