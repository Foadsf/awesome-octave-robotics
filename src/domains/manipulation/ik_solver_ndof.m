% ik_solver_ndof.m — Numerical inverse kinematics (damped least squares).
%
%   [q, success, iter] = ik_solver_ndof(dh, q0, T_des, max_iter, tol)
%
%   Iteratively solves f(q) = T_ee(q) - T_des = 0 using the damped
%   least-squares (Levenberg-Marquardt) step:
%       dq = J' * (J*J' + lambda^2*I)^{-1} * twist_error
%   where twist_error = [position_error; orientation_error] (axis-angle).
%
%   dh     : (n x 4) DH table [theta, d, a, alpha]
%   q0     : (n x 1) initial guess
%   T_des  : (4 x 4) desired end-effector transform
%   max_iter: max iterations (default 100)
%   tol    : position tolerance (default 1e-4 m)
%
%   Returns q (n x 1), success (bool), iter (iterations used).
%
%   Pure Octave native code.

function [q, success, iter] = ik_solver_ndof(dh, q0, T_des, max_iter, tol)
  if nargin < 5, tol = 1e-4; end
  if nargin < 4, max_iter = 100; end
  if nargin < 3, T_des = eye(4); end
  if nargin < 2, q0 = zeros(rows(dh),1); end
  n = rows(dh);
  q = q0;
  lambda = 0.1;          % damping
  success = false;
  iter = 0;
  R_des = T_des(1:3,1:3); p_des = T_des(1:3,4);
  for k = 1:max_iter
    iter = k;
    [T, transforms] = rigid_body_tree(dh, q);
    p = T(1:3,4); R = T(1:3,1:3);
    e_p = p_des - p;
    % orientation error (axis-angle): e_R = vee((R_des*R' - R*R_des')/2)
    R_err = R_des * R' - R * R_des';
    e_o = 0.5 * [R_err(3,2) - R_err(2,3);
                 R_err(1,3) - R_err(3,1);
                 R_err(2,1) - R_err(1,2)];
    e = [e_p; e_o];
    err = norm(e_p);
    if err < tol
      success = true; return;
    endif
    J = rigid_body_jacobian(dh, q);
    % damped least squares: dq = pinv_damped(J) * e
    A = J * J' + lambda^2 * eye(6);
    dq = J' * (A \ e);
    % small step
    q = q + 0.5 * dq;
  endfor
  % final check
  [T, ~] = rigid_body_tree(dh, q);
  success = norm(T_des(1:3,4) - T(1:3,4)) < tol;
endfunction
