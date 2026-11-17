% rigid_body_jacobian.m — Spatial Jacobian of the end-effector for a DH tree.
%
%   J = rigid_body_jacobian(dh, q)
%
%   dh     : (n x 4) DH table [theta, d, a, alpha]
%   q      : (n x 1) joint positions
%   J      : (6 x n) spatial Jacobian mapping joint velocities to EE twist
%            [linear velocity; angular velocity] in the base frame.
%
%   Builds on rigid_body_tree for the forward kinematics.
%
%   Pure Octave native code.

function J = rigid_body_jacobian(dh, q)
  if nargin < 2, q = zeros(rows(dh), 1); end
  [~, transforms] = rigid_body_tree(dh, q);
  n = rows(dh);
  p_ee = transforms{n}(1:3, 4);
  J = zeros(6, n);
  for i = 1:n
    T = transforms{i};
    z = T(1:3, 3);        % joint axis (z of frame i) in base frame
    p = T(1:3, 4);        % origin of frame i
    J(1:3, i) = cross(z, p_ee - p);
    J(4:6, i) = z;
  endfor
endfunction
