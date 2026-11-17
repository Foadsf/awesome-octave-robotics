% los_guidance.m — Line-of-sight (LOS) guidance for a marine surface vessel / AUV.
%
%   [chi_d, e_ct, e_along] = los_guidance(pos, path, Delta, lookahead_idx)
%
%   pos           : [x y] vessel position (m)
%   path          : Nx2 waypoints [x y] defining the desired track
%   Delta         : look-ahead distance (m)
%   lookahead_idx : warm-start index into path (1..N-1)
%
%   Finds the closest point on the path, computes the along-track and
%   cross-track error, places a look-ahead point Delta ahead along the track,
%   and returns the desired course chi_d = atan2(LA_y - y, LA_x - x).
%   Standard LOS guidance used in marine control (Fossen).

function [chi_d, e_ct, e_along] = los_guidance(pos, path, Delta, lookahead_idx)
  pos = reshape(pos, 1, 2); path = reshape(path, [], 2);
  N = size(path, 1);
  if nargin < 4 || isempty(lookahead_idx), lookahead_idx = 1; end
  if nargin < 3, Delta = 1.0; end
  % closest point on any path segment
  best = inf; cp = path(1, :); seg = 1; t_star = 0;
  for i = 1:N-1
    a = path(i, :); b = path(i+1, :); d = b - a; L2 = d*d';
    if L2 == 0, t = 0; else t = max(0, min(1, ((pos - a)*d')/L2)); end
    closest = a + t * d;
    dist = norm(closest - pos);
    if dist < best, best = dist; cp = closest; seg = i; t_star = t; end
  end
  % signed cross-track error (side of path)
  segdir = path(seg+1, :) - path(seg, :);
  seglen = norm(segdir); segdir = segdir / seglen;
  e_ct = -segdir(2)*(pos(1)-cp(1)) + segdir(1)*(pos(2)-cp(2));
  e_along = (seg - 1 + t_star) * seglen;  % approx along-track
  % look-ahead point Delta ahead along the path
  s_seg = t_star * seglen;   % position within segment (m)
  s_need = s_seg + Delta;
  la = cp; s_rem = s_need;
  i = seg;
  while i <= N-1 && s_rem > 0
    seglen = norm(path(i+1,:) - path(i,:));
    if s_rem <= seglen
      la = path(i,:) + (s_rem/seglen)*(path(i+1,:)-path(i,:)); s_rem = 0;
    else
      s_rem = s_rem - seglen; i = i + 1;
    end
  end
  if s_rem > 0, la = path(N,:); end   % ran past end: use last waypoint
  chi_d = atan2(la(2) - pos(2), la(1) - pos(1));
endfunction
