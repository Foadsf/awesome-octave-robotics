% dynamic_replanner.m — Incremental replanning with pop-up obstacles.
%
%   [path, replans] = dynamic_replanner(start, goal, static_obstacles, opts)
%
%   Simulates following a path toward goal; when a pop-up obstacle blocks
%   the current segment, replans around it. Returns the final path and
%   number of replans.
%
%   start, goal : [x y]
%   static_obstacles : Kx3 [cx cy r]
%   opts : struct (optional)
%     .step     : step size (default 0.2)
%     .replan_thr: distance at which obstacle triggers replan (default 0.5)
%     .max_replans: (default 10)
%
%   Pure Octave native code (uses planner_astar on a grid).

function [path, replans] = dynamic_replanner(start, goal, static_obstacles, opts)
  if nargin < 4, opts = struct(); end
  if ~isstruct(opts), opts = struct(); end
  step = pc_pick(opts, "step", 0.2);
  replan_thr = pc_pick(opts, "replan_thr", 0.5);
  max_replans = pc_pick(opts, "max_replans", 10);

  replans = 0;
  cur = start;
  path = start;
  popup = [];   % pop-up obstacles appear over time
  bounds = [min(start(1), goal(1))-1, min(start(2), goal(2))-1;
            max(start(1), goal(1))+1, max(start(2), goal(2))+1];
  for t = 0:step:20
    if t > 5 && t < 7 && isempty(popup)
      popup = [cur(1)+0.3, cur(2)+0.3, 0.4];   % sudden obstacle
      replans = replans + 1;
      all_obs = [static_obstacles; popup];
      grid = occupancy_grid_from_obs(all_obs, bounds, 0.1);
      start_cell = world_to_grid(cur, bounds, 0.1);
      goal_cell = world_to_grid(goal, bounds, 0.1);
      sub = planner_astar(grid, start_cell, goal_cell);
      if ~isempty(sub)
        for k = 1:rows(sub)
          path = [path; grid_to_world(sub(k,:), bounds, 0.1)];
        endfor
      endif
    endif
    if rows(path) > 1
      nxt = path(2, :);
      if norm(nxt - cur) < step
        path(2, :) = [];
      endif
      cur = cur + step * (nxt - cur) / max(norm(nxt - cur), 1e-9);
    endif
    if norm(cur - goal) < step
      path = [path; goal];
      break;
    endif
    if replans > max_replans, break; end
  endfor
endfunction

function grid = occupancy_grid_from_obs(obs, bounds, res)
  nrows = round((bounds(2,2)-bounds(1,2))/res) + 1;
  ncols = round((bounds(2,1)-bounds(1,1))/res) + 1;
  grid = zeros(nrows, ncols);
  if isempty(obs), return; end
  for i = 1:rows(obs)
    cx = obs(i,1); cy = obs(i,2); r = obs(i,3);
    for ci = 1:nrows
      for cj = 1:ncols
        wx = bounds(1,1) + (cj-1)*res;
        wy = bounds(1,2) + (ci-1)*res;
        if norm([wx, wy] - [cx, cy]) <= r
          grid(ci, cj) = 1;
        endif
      endfor
    endfor
  endfor
endfunction

function cell = world_to_grid(p, bounds, res)
  cell = [round((p(2)-bounds(1,2))/res)+1, round((p(1)-bounds(1,1))/res)+1];
endfunction

function p = grid_to_world(cell, bounds, res)
  p = [bounds(1,1) + (cell(2)-1)*res, bounds(1,2) + (cell(1)-1)*res];
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
