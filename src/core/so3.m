% so3.m — SO(3) rotation group (struct of function handles, classdef-free).
%
%   S = so3();                 % create an SO(3) operator object
%   R = S.exp(w);              % exponential map: so(3) vector -> SO(3) matrix (Rodrigues)
%   w = S.log(R);              % logarithm: SO(3) matrix -> so(3) vector
%   Sx = S.hat(w);             % hat map: R^3 -> skew-symmetric 3x3
%   w = S.vee(Sx);             % vee map: skew-symmetric 3x3 -> R^3
%   R = S.Rx(a); R = S.Ry(a); R = S.Rz(a);   % elementary rotations
%   R = S.rpy(roll, pitch, yaw);             % roll-pitch-yaw (ZYX extrinsic = XYZ intrinsic)
%   R = S.compose(R1, R2);   % group composition: R1 * R2
%   Ri = S.inv(R);             % group inverse: R'
%   ok = S.isRotation(R, tol); % validity check (orthonormal, det=+1)
%
% All angles in radians. Quaternions live in quat_ops.m.

function s = so3()
  s.hat       = @so3_hat;
  s.vee       = @so3_vee;
  s.exp       = @so3_exp;
  s.log       = @so3_log;
  s.Rx        = @so3_Rx;
  s.Ry        = @so3_Ry;
  s.Rz        = @so3_Rz;
  s.rpy       = @so3_rpy;
  s.compose   = @so3_compose;
  s.inv       = @so3_inv;
  s.isRotation = @so3_isRotation;
endfunction

% --- constructors / elementary -------------------------------------------

function R = so3_Rx(a)
  ca = cos(a); sa = sin(a);
  R = [1, 0, 0; 0, ca, -sa; 0, sa, ca];
endfunction

function R = so3_Ry(a)
  ca = cos(a); sa = sin(a);
  R = [ca, 0, sa; 0, 1, 0; -sa, 0, ca];
endfunction

function R = so3_Rz(a)
  ca = cos(a); sa = sin(a);
  R = [ca, -sa, 0; sa, ca, 0; 0, 0, 1];
endfunction

function R = so3_rpy(roll, pitch, yaw)
  % ZYX extrinsic convention (roll about x, pitch about y, yaw about z)
  R = so3_Rz(yaw) * so3_Ry(pitch) * so3_Rx(roll);
endfunction

% --- hat / vee -----------------------------------------------------------

function S = so3_hat(w)
  w = reshape(w, 3, 1);
  S = [ 0,   -w(3),  w(2);
        w(3),  0,   -w(1);
       -w(2),  w(1),  0 ];
endfunction

function w = so3_vee(S)
  S = reshape(S, 3, 3);
  w = [S(3,2); S(1,3); S(2,1)];
endfunction

% --- exponential / logarithm (Rodrigues) ---------------------------------

function R = so3_exp(w)
  w = reshape(w, 3, 1);
  theta = norm(w);
  if theta < 1e-12
    R = eye(3) + so3_hat(w);
    return;
  end
  K = so3_hat(w);
  R = eye(3) + sin(theta)/theta * K + (1 - cos(theta))/theta^2 * (K * K);
endfunction

function w = so3_log(R)
  R = reshape(R, 3, 3);
  % keep original for the skew (antisymmetric) part; symmetrize only for trace
  Rorig = R;
  R = 0.5 * (R + R');
  tr = trace(R);
  theta = acos(max(-1, min(1, (tr - 1)/2)));
  if theta < 1e-10
    w = zeros(3,1);
    return;
  end
  if theta > pi - 1e-10
    % near-pi rotation: extract axis from the symmetric part
    M = R + eye(3);
    [~, i] = max(diag(M));
    axis = M(:, i) / sqrt(max(M(i, i), 1e-14));
    axis = axis / norm(axis);
    w = pi * axis;
    return;
  end
  w = theta / (2 * sin(theta)) * so3_vee(Rorig - Rorig');
endfunction

% --- composition / inverse / validity ------------------------------------

function Rc = so3_compose(R1, R2)
  Rc = R1 * R2;
endfunction

function Ri = so3_inv(R)
  Ri = R';
endfunction

function ok = so3_isRotation(R, tol)
  if nargin < 2, tol = 1e-9; end
  R = reshape(R, 3, 3);
  ok = (norm(R' * R - eye(3), 'fro') < tol) && ...
       (abs(det(R) - 1) < tol);
endfunction
