% collision_check_manip.m — Link-level collision checking (spheres + capsules).
%
%   [collision, pairs] = collision_check_manip(transforms, link_obs, obstacles)
%
%   Each link is modeled as a sphere at its origin with a given radius, plus
%   end spheres. Checks:
%     * link-link self-collision (non-adjacent links, sphere-sphere)
%     * link-obstacle collision (sphere obstacles)
%
%   transforms : cell array {n} of (4x4) base->link transforms (from rigid_body_tree)
%   link_obs   : struct with .radius (n x 1) per link
%   obstacles  : (m x 4) [x y z radius] obstacle spheres
%
%   Returns collision (bool) and pairs (k x 2) of colliding link indices or
%   [link_id; -(obstacle_id)] for link-obstacle.
%
%   Pure Octave native code.

function [collision, pairs] = collision_check_manip(transforms, link_obs, obstacles)
  if nargin < 3, obstacles = zeros(0,4); end
  n = length(transforms);
  pairs = [];
  collision = false;
  radii = link_obs.radius;
  if isempty(radii), radii = ones(n,1)*0.1; end

  % link origins
  P = zeros(n, 3);
  for i = 1:n
    P(i,:) = transforms{i}(1:3,4)';
  endfor

  % self-collision: non-adjacent links (sphere-sphere)
  for i = 1:n
    for j = i+2:n  % skip adjacent links
      d = norm(P(i,:) - P(j,:));
      if d < radii(i) + radii(j)
        collision = true;
        pairs = [pairs; i, j];
      endif
    endfor
  endfor

  % link-obstacle: distance from link origin (sphere) to obstacle sphere
  if ~isempty(obstacles)
    for i = 1:n
      for k = 1:rows(obstacles)
        d = norm(P(i,:) - obstacles(k,1:3));
        if d < radii(i) + obstacles(k,4)
          collision = true;
          pairs = [pairs; i, -k];
        endif
      endfor
    endfor
  endif
endfunction
