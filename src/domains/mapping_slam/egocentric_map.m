% egocentric_map.m — Robot-centric scrolling near-field occupancy grid.
%
%   [grid, origin] = egocentric_map(robot_pose, scans, grid_size, resolution)
%
%   robot_pose : [x y theta] current robot pose (world frame)
%   scans      : cell array of Mx2 scans (robot frame) accumulated over time
%   grid_size  : scalar (cells) or [rows cols]
%   resolution : grid cell size (m)
%
%   Returns an egocentric occupancy grid (probabilities 0..1) centered on
%   the robot, plus the world-frame origin of the grid.
%
%   Pure Octave native code.

function [grid, origin] = egocentric_map(robot_pose, scans, grid_size, resolution)
  if isscalar(grid_size), grid_size = [grid_size, grid_size]; end
  rx = robot_pose(1); ry = robot_pose(2); rth = robot_pose(3);
  origin = [rx - grid_size(2)*resolution/2, ry - grid_size(1)*resolution/2];
  grid = zeros(grid_size(1), grid_size(2));

  R = [cos(rth), -sin(rth); sin(rth), cos(rth)];
  % robot cell (row=ci, col=cj)
  ci_r = round((ry - origin(2)) / resolution) + 1;
  cj_r = round((rx - origin(1)) / resolution) + 1;
  for s = 1:numel(scans)
    scan = scans{s};
    if isempty(scan), continue; end
    p_world = (R * scan')' + [rx, ry];
    for k = 1:rows(p_world)
      endp = p_world(k, :);
      ci = round((endp(2) - origin(2)) / resolution) + 1;
      cj = round((endp(1) - origin(1)) / resolution) + 1;
      if ci >= 1 && ci <= grid_size(1) && cj >= 1 && cj <= grid_size(2)
        grid(ci, cj) = 0.65;   % occupied endpoint
      endif
      % free cells along ray from robot to endpoint (skip endpoint)
      ray = bresenham_raycast(cj_r, ci_r, cj, ci);
      for r = 1:rows(ray)
        ri = ray(r, 2);   % row
        cj2 = ray(r, 1);  % col
        if ri == ci && cj2 == cj, break; end   % skip endpoint
        if ri >= 1 && ri <= grid_size(1) && cj2 >= 1 && cj2 <= grid_size(2)
          grid(ri, cj2) = 0.35;   % free
        endif
      endfor
    endfor
  endfor
endfunction
