% rigid_body_tree.m — Forward kinematics via a rigid-body (DH) tree.
%
%   [T, transforms] = rigid_body_tree(dh, q)
%
%   Standard Denavit-Hartenberg convention:
%     T_i = RotZ(theta_i) * TransZ(d_i) * TransX(a_i) * RotX(alpha_i)
%
%   dh     : (n x 4) table [theta, d, a, alpha] (theta in rad, alpha in rad)
%   q      : (n x 1) joint positions added to theta (revolute)
%   T      : (4 x 4) end-effector transform (base -> tip)
%   transforms: cell array {n} of link transforms (base -> link i)
%
%   Pure Octave native code.

function [T, transforms] = rigid_body_tree(dh, q)
  if nargin < 2, q = zeros(rows(dh), 1); end
  n = rows(dh);
  transforms = cell(n, 1);
  T = eye(4);
  for i = 1:n
    th = dh(i,1) + q(i);
    d  = dh(i,2);
    a  = dh(i,3);
    al = dh(i,4);
    T = T * dh_transform(th, d, a, al);
    transforms{i} = T;
  endfor
endfunction

function M = dh_transform(theta, d, a, alpha)
  ct = cos(theta); st = sin(theta);
  ca = cos(alpha); sa = sin(alpha);
  M = [ ct, -st*ca,  st*sa, a*ct;
        st,  ct*ca, -ct*sa, a*st;
        0,      sa,      ca,   d;
        0,       0,       0,   1];
endfunction
