% lipm.m — Linear Inverted Pendulum Model (LIPM) for legged walking.
%
%   [com, zmp, capture, stable] = lipm(z, g, dt, T, zmp_seq, x0, xd0)
%
%   z      : CoM height (m), constant
%   g      : gravity (m/s^2)
%   dt     : integration step (s)
%   T      : step duration (s); ZMP is held constant over each step
%   zmp_seq: Mx1 or Mx2 ZMP (foot) positions per step; scalar -> 1D
%   x0, xd0: initial CoM position and velocity
%
%   Dynamics:  x_ddot = (g/z) * (x - c),  c = ZMP (constant per step)
%   Exact step solution with w = sqrt(g/z):
%     x(t)  = c + (x0-c) cosh(w t) + (xd0/w) sinh(w t)
%     xd(t) = w (x0-c) sinh(w t) + xd0 cosh(w t)
%
%   Returns com (M x 2) [position; velocity] after each step, the ZMP used,
%   capture point xc = x + xd/w (divergent component), and stable = capture
%   point within +/- half-step of the ZMP (simple support check).

function [com, zmp, capture, stable] = lipm(z, g, dt, T, zmp_seq, x0, xd0)
  zmp_seq = reshape(zmp_seq, [], 1);
  M = size(zmp_seq, 1);
  w = sqrt(g / z);
  com = zeros(M, 2); capture = zeros(M, 1); stable = false(M, 1);
  x = x0; xd = xd0;
  nsub = max(1, round(T / dt));
  h = T / nsub;
  for k = 1:M
    c = zmp_seq(k);
    for s = 1:nsub
      wh = w * h;
      ch = cosh(wh); sh = sinh(wh);
      x_new = c + (x - c) * ch + (xd / w) * sh;
      xd_new = w * (x - c) * sh + xd * ch;
      x = x_new; xd = xd_new;
    end
    com(k, :) = [x, xd];
    capture(k) = x + xd / w;        % capture point (divergent component of motion)
    stable(k) = abs(capture(k) - c) <= 0.15;  % capture point within 15 cm of ZMP
  end
  zmp = zmp_seq;
endfunction
