% se2.m — SE(2) planar rigid-body group (struct of function handles).
%
%   G = se2();                 % create an SE(2) operator object
%   T = G.tform(x, y, theta);  % homogeneous 3x3 transform
%   xi = G.exp(xi);            % exponential: se(2) twist [vx;vy;omega] -> SE(2)
%   xi = G.log(T);             % logarithm: SE(2) -> se(2) twist
%   Tc = G.compose(T1, T2);    % group composition T1 * T2
%   Ti = G.inv(T);             % group inverse
%   p2 = G.transform(T, p);    % apply transform to point(s) p (2xN)
%   A = G.adjoint(T);          % 3x3 adjoint acting on twists
%   S = G.hat(xi);             % 3x3 se(2) algebra matrix
%   xi = G.vee(S);             % vee: 3x3 -> twist vector

function s = se2()
  s.tform     = @se2_tform;
  s.exp       = @se2_exp;
  s.log       = @se2_log;
  s.compose   = @se2_compose;
  s.inv       = @se2_inv;
  s.transform = @se2_transform;
  s.adjoint   = @se2_adjoint;
  s.hat       = @se2_hat;
  s.vee       = @se2_vee;
endfunction

function T = se2_tform(x, y, theta)
  c = cos(theta); s = sin(theta);
  T = [c, -s, x;
       s,  c, y;
       0,  0, 1];
endfunction

function T = se2_exp(xi)
  xi = reshape(xi, 3, 1);
  v = xi(1:2); w = xi(3);
  if abs(w) < 1e-12
    T = eye(3);
    T(1:2, 3) = v;
    return;
  end
  % rotation part
  c = cos(w); s = sin(w);
  R = [c, -s; s, c];
  % translation: V * v where V = sin(w)/w * I + (1-cos(w))/w^2 * hat(w)
  J = [0, -1; 1, 0];  % hat(1) in 2D
  V = (sin(w)/w) * eye(2) + ((1 - cos(w))/w^2) * J;
  p = V * v;
  T = [R, p; 0 0 1];
endfunction

function xi = se2_log(T)
  T = reshape(T, 3, 3);
  R = T(1:2, 1:2);
  p = T(1:2, 3);
  c = (R(1,1) + R(2,2)) / 2;
  c = max(-1, min(1, c));
  w = acos(c);
  if abs(w) < 1e-10
    xi = [p; 0];
    return;
  end
  S = [0, -1; 1, 0];
  V_inv = (1/w) * eye(2) - 0.5 * S + (1/w - 1/tan(w)) * (S * S);
  v = V_inv * p;
  xi = [v; w];
endfunction

function Tc = se2_compose(T1, T2)
  Tc = T1 * T2;
endfunction

function Ti = se2_inv(T)
  T = reshape(T, 3, 3);
  R = T(1:2, 1:2);
  p = T(1:2, 3);
  Ti = [R', -R' * p; 0 0 1];
endfunction

function p2 = se2_transform(T, p)
  T = reshape(T, 3, 3);
  p = reshape(p, 2, []);
  [m, n] = size(p);
  ph = [p; ones(1, n)];
  p2h = (T * ph);
  p2 = p2h(1:2, :);
endfunction

function A = se2_adjoint(T)
  T = reshape(T, 3, 3);
  R = T(1:2, 1:2);
  p = T(1:2, 3);
  % Adjoint for SE(2): Ad_T = [ R, 0; p^ R, 1 ] where p^ = [0 -py; px 0]
  P = [0, -p(2); p(1), 0];
  A = [R, zeros(2,1); P * R, 1];
endfunction

function S = se2_hat(xi)
  xi = reshape(xi, 3, 1);
  S = [ 0, -xi(3), xi(1);
        xi(3), 0,  xi(2);
        0,   0,    0   ];
endfunction

function xi = se2_vee(S)
  S = reshape(S, 3, 3);
  xi = [S(1,3); S(2,3); S(2,1)];
endfunction
