% se3.m — SE(3) rigid-body group via screw theory (struct of function handles).
% Fully self-contained (no cross-file dependence on local helpers).
%
%   G = se3();                 % create an SE(3) operator object
%   T = G.tform(R, p);         % homogeneous 4x4 transform from R (3x3), p (3x1)
%   T = G.exp(xi);             % exponential: se(3) twist [v; w] (6x1) -> SE(3)
%   xi = G.log(T);             % logarithm: SE(3) -> se(3) twist
%   Tc = G.compose(T1, T2);    % group composition T1 * T2
%   Ti = G.inv(T);             % group inverse
%   p2 = G.transform(T, p);    % apply transform to point(s) p (3xN)
%   A = G.adjoint(T);          % 6x6 adjoint acting on twists
%   S = G.hat(xi);             % 4x4 se(3) algebra matrix
%   xi = G.vee(S);             % vee: 4x4 -> twist vector

function s = se3()
  s.tform      = @se3_tform;
  s.exp        = @se3_exp;
  s.log        = @se3_log;
  s.compose    = @se3_compose;
  s.inv        = @se3_inv;
  s.transform  = @se3_transform;
  s.adjoint    = @se3_adjoint;
  s.hat        = @se3_hat;
  s.vee        = @se3_vee;
endfunction

% --- small local rotation helpers (Rodrigues) ---------------------------

function R = rodrigues_exp(w)
  w = reshape(w, 3, 1);
  theta = norm(w);
  if theta < 1e-12
    R = eye(3) + [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
    return;
  end
  K = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0] / theta;  % unit axis skew
  u = w / theta;
  Ku = [0, -u(3), u(2); u(3), 0, -u(1); -u(2), u(1), 0];
  R = eye(3) + sin(theta) * Ku + (1 - cos(theta)) * (Ku * Ku);
endfunction

function w = rodrigues_log(R)
  R = reshape(R, 3, 3);
  Rorig = R;                      % keep original for the skew part
  R = 0.5 * (R + R');             % symmetrize only for the trace
  tr = trace(R);
  theta = acos(max(-1, min(1, (tr - 1) / 2)));
  if theta < 1e-10
    w = zeros(3, 1);
    return;
  end
  if theta > pi - 1e-10
    M = R + eye(3);
    [~, i] = max(diag(M));
    axis = M(:, i) / sqrt(max(M(i, i), 1e-14));
    axis = axis / norm(axis);
    w = pi * axis;
    return;
  end
  w = theta / (2 * sin(theta)) * [Rorig(3, 2) - Rorig(2, 3); Rorig(1, 3) - Rorig(3, 1); Rorig(2, 1) - Rorig(1, 2)];
endfunction

function K = hat3(w)
  w = reshape(w, 3, 1);
  K = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
endfunction

function T = se3_tform(R, p)
  R = reshape(R, 3, 3); p = reshape(p, 3, 1);
  T = [R, p; 0 0 0 1];
endfunction

function T = se3_exp(xi)
  xi = reshape(xi, 6, 1);
  v = xi(1:3); w = xi(4:6);
  theta = norm(w);
  R = rodrigues_exp(w);
  if theta < 1e-12
    T = [eye(3), v; 0 0 0 1];
    return;
  end
  K = hat3(w);
  V = eye(3) + ((1 - cos(theta)) / theta^2) * K ...
        + ((theta - sin(theta)) / theta^3) * (K * K);
  p = V * v;
  T = [R, p; 0 0 0 1];
endfunction

function xi = se3_log(T)
  T = reshape(T, 4, 4);
  R = T(1:3, 1:3);
  p = T(1:3, 4);
  w = rodrigues_log(R);
  theta = norm(w);
  if theta < 1e-10
    xi = [p; zeros(3, 1)];   % pure translation
    return;
  end
  K = hat3(w);
  c = cos(theta); s = sin(theta);
  % V^{-1} = I - 1/2 K + (1/theta^2)(1 - theta*s/(2*(1-c))) K^2
  coeff = (1 - theta * s / (2 * (1 - c))) / theta^2;
  V_inv = eye(3) - 0.5 * K + coeff * (K * K);
  v = V_inv * p;
  xi = [v; w];
endfunction

function Tc = se3_compose(T1, T2)
  Tc = T1 * T2;
endfunction

function Ti = se3_inv(T)
  T = reshape(T, 4, 4);
  R = T(1:3, 1:3);
  p = T(1:3, 4);
  Ti = [R', -R' * p; 0 0 0 1];
endfunction

function p2 = se3_transform(T, p)
  T = reshape(T, 4, 4);
  p = reshape(p, 3, []);
  [~, n] = size(p);
  ph = [p; ones(1, n)];
  p2h = T * ph;
  p2 = p2h(1:3, :);
endfunction

function A = se3_adjoint(T)
  T = reshape(T, 4, 4);
  R = T(1:3, 1:3);
  p = T(1:3, 4);
  K = hat3(p);
  A = [R, zeros(3);
       K * R, R];
endfunction

function S = se3_hat(xi)
  xi = reshape(xi, 6, 1);
  S = [hat3(xi(4:6)), xi(1:3); 0 0 0 0];
endfunction

function xi = se3_vee(S)
  S = reshape(S, 4, 4);
  w = [S(3, 2); S(1, 3); S(2, 1)];
  v = S(1:3, 4);
  xi = [v; w];
endfunction
