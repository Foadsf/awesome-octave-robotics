% lqr_track.m — LQR trajectory tracking of a unicycle along a straight line.
%
%   [t, X, U, K] = lqr_track(v, dt, Tsim, y0, theta0)
%
%   v        : forward speed (m/s)
%   dt       : step (s),  Tsim : simulation time (s)
%   y0, theta0: initial lateral offset and heading error (m, rad)
%
%   Lateral error dynamics about a straight reference at speed v:
%     [eydot; ethetadot] = [0 v; 0 0][ey; etheta] + [0; 1] * omega
%   Discretised and regulated with discrete-time LQR (dare_lqr from src/core):
%     omega = -K [ey; etheta]  (reference: y=0, heading=0)
%   The unicycle is then simulated with this feedback. Returns time,
%   states X=[x y theta], controls U=omega, and gain K.

function [t, X, U, K] = lqr_track(v, dt, Tsim, y0, theta0)
  if nargin < 5, theta0 = 0.2; end
  if nargin < 4, y0 = 0.5; end
  if nargin < 3, Tsim = 8.0; end
  if nargin < 2, dt = 0.02; end
  if nargin < 1, v = 1.0; end
  A = [0, v; 0, 0]; B = [0; 1];
  Ad = [1, v*dt; 0, 1]; Bd = [v*dt^2/2; dt];
  Q = diag([10, 1]); R = 1;
  [~, K, ~] = dare_lqr(Ad, Bd, Q, R);
  n = max(1, round(Tsim/dt));
  X = zeros(n, 3); U = zeros(n, 1);
  x = 0; y = y0; th = theta0;
  for k = 1:n
    X(k, :) = [x, y, th];
    e = [y; th];                    % lateral error + heading error (ref = 0)
    U(k) = -K * e;                  % LQR feedback
    [x_next, ~] = unicycle_step([x; y; th], v, U(k), dt);
    x = x_next(1); y = x_next(2); th = x_next(3);
  end
  t = (0:n-1)' * dt;
endfunction
