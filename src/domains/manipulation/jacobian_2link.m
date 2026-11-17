% jacobian_2link.m — Geometric Jacobian of a 2-link planar arm.
%
%   J = jacobian_2link(q, l1, l2)
%
%   q   : [q1; q2] joint angles (rad)
%   l1, l2: link lengths (m)
%
%   J (2x2) maps joint velocity qdot to end-effector linear velocity:
%   [xdot; ydot] = J * [q1dot; q2dot]. Enables Jacobian-transpose /
%   pseudoinverse iterative IK.

function J = jacobian_2link(q, l1, l2)
  q = reshape(q, 2, 1);
  q1 = q(1); q2 = q(2);
  s1 = sin(q1); c1 = cos(q1);
  s12 = sin(q1 + q2); c12 = cos(q1 + q2);
  J = [-l1 * s1 - l2 * s12, -l2 * s12;
        l1 * c1 + l2 * c12,  l2 * c12];
endfunction
