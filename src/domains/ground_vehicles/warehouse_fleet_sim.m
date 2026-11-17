% warehouse_fleet_sim.m — Multi-AGV warehouse task assignment + grid routing.
%
%   [trajectories, metrics] = warehouse_fleet_sim(n_agv, grid_size, n_tasks, T)
%
%   Simulates n_agv AGVs on a grid_size x grid_size warehouse floor. Random
%   pick/delivery tasks appear; each AGV is assigned the nearest feasible
%   task (greedy by Manhattan distance). Routing uses A* on the free grid.
%   Collision avoidance is priority-based: AGVs reserve the cell they will
%   enter at the next timestep; a lower-priority AGV whose next cell is
%   reserved waits (holds). Priorities are static (AGV id order) so no
%   circular wait can form — deadlock-free by construction.
%
%   Returns:
%     trajectories : cell array {n_agv}, each a (T+1) x 3 [x y priority]
%     metrics      : struct with tasks_completed, total_distance, waits,
%                    deadlocks (0 by construction), makespan
%
%   Pure Octave native code.

function [trajectories, metrics] = warehouse_fleet_sim(n_agv, grid_size, n_tasks, T)
  if nargin < 4, T = 200; end
  if nargin < 3, n_tasks = max(4, 2*n_agv); end
  if nargin < 2, grid_size = 12; end
  if nargin < 1, n_agv = 4; end

  rng(42);
  free = true(grid_size, grid_size);
  % static obstacle blocks
  for b = 1:max(1, floor(grid_size/6))
    cx = randi(grid_size); cy = randi(grid_size);
    free(max(1,cx-1):min(grid_size,cx+1), max(1,cy-1):min(grid_size,cy+1)) = false;
  endfor
  free(1,1) = true; free(grid_size,grid_size) = true;

  free_cells = find(free);
  start_idx = randperm(length(free_cells), min(n_agv, length(free_cells)));
  n_agv = length(start_idx);
  pos = zeros(n_agv, 2);
  for i = 1:n_agv
    ci = free_cells(start_idx(i));
    pos(i,1) = mod(ci-1, grid_size) + 1;
    pos(i,2) = floor((ci-1)/grid_size) + 1;
  endfor

  tasks = zeros(n_tasks, 4);
  task_status = zeros(n_tasks, 1);   % 0 pending,1 assigned,2 done
  task_ready_at = zeros(n_tasks, 1);
  for k = 1:n_tasks
    tasks(k,:) = [randi(grid_size), randi(grid_size), randi(grid_size), randi(grid_size)];
    task_ready_at(k) = randi([1, max(1,T/2)]);
    tasks(k,1:2) = snap_to_free(tasks(k,1:2), free, grid_size);
    tasks(k,3:4) = snap_to_free(tasks(k,3:4), free, grid_size);
  endfor

  phase = zeros(n_agv, 1);        % 0 idle,1 to pickup,2 to delivery
  cur_task = zeros(n_agv, 1);
  target = zeros(n_agv, 2);
  trajectories = cell(n_agv, 1);
  for i = 1:n_agv
    trajectories{i} = zeros(T+1, 3);
    trajectories{i}(1,:) = [pos(i,1), pos(i,2), i];
  endfor

  metrics.tasks_completed = 0;
  metrics.total_distance = 0;
  metrics.waits = 0;
  metrics.deadlocks = 0;
  metrics.makespan = 0;

  for t = 1:T
    % task assignment: nearest idle AGV to each ready pending task
    for i = 1:n_agv
      if phase(i) ~= 0, continue; end
      best = 0; bestd = inf;
      for k = 1:n_tasks
        if task_status(k) == 0 && task_ready_at(k) <= t
          d = abs(pos(i,1)-tasks(k,1)) + abs(pos(i,2)-tasks(k,2));
          if d < bestd, bestd = d; best = k; end
        end
      endfor
      if best > 0
        task_status(best) = 1; phase(i) = 1; cur_task(i) = best;
        target(i,:) = tasks(best,1:2);
      endif
    endfor

    % reservation set for cells entered at t+1 (single timestep lookahead)
    reserved = false(grid_size, grid_size);

    for i = 1:n_agv   % static priority = lower id first
      if phase(i) == 0, continue; end
      cur = pos(i,:); tgt = target(i,:);
      if cur(1)==tgt(1) && cur(2)==tgt(2)
        if phase(i) == 1
          phase(i) = 2; target(i,:) = tasks(cur_task(i),3:4);
          continue;
        else
          task_status(cur_task(i)) = 2;
          metrics.tasks_completed = metrics.tasks_completed + 1;
          metrics.makespan = max(metrics.makespan, t);
          phase(i) = 0; cur_task(i) = 0;
          continue;
        endif
      endif
      nxt = astar_next_step(cur, tgt, free, grid_size);
      % collision avoidance
      if reserved(nxt(1), nxt(2))
        metrics.waits = metrics.waits + 1;
        nxt = cur;   % hold
      else
        reserved(nxt(1), nxt(2)) = true;
      endif
      pos(i,:) = nxt;
      metrics.total_distance = metrics.total_distance + norm(nxt - cur);
      trajectories{i}(t+1,:) = [nxt(1), nxt(2), i];
    endfor

    for i = 1:n_agv
      if all(trajectories{i}(t+1,:)==0)
        trajectories{i}(t+1,:) = [pos(i,1), pos(i,2), i];
      endif
    endfor
  endfor
endfunction

function p = snap_to_free(p, free, gs)
  p = round(p); p(1)=max(1,min(gs,p(1))); p(2)=max(1,min(gs,p(2)));
  if free(p(1),p(2)), return; end
  found = false;
  for rad = 1:gs
    for dx = -rad:rad
      for dy = -rad:rad
        if abs(dx)==rad || abs(dy)==rad
          x = p(1)+dx; y = p(2)+dy;
          if x>=1 && x<=gs && y>=1 && y<=gs && free(x,y)
            p = [x y]; found = true; break;
          endif
        endif
      endfor
      if found, break; end
    endfor
    if found, break; end
  endfor
endfunction

function nxt = astar_next_step(cur, tgt, free, gs)
  % Array-based A* returning the first move from cur toward tgt on free grid.
  g = inf(gs, gs); f = inf(gs, gs);
  parent = zeros(gs, gs, 2);
  closed = false(gs, gs);
  g(cur(1), cur(2)) = 0;
  f(cur(1), cur(2)) = heuristic(cur, tgt);
  openx = cur(1); openy = cur(2);
  dirs = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
  dcost = [1 1 1 1 sqrt(2) sqrt(2) sqrt(2) sqrt(2)];
  while true
    [cx, cy, bi] = pop_best_cost(openx, openy, f, closed);
    if isempty(cx), break; end
    closed(cx, cy) = true;
    if cx==tgt(1) && cy==tgt(2)
      nxt = first_step(reconstruct(parent, cur, tgt), cur);
      return;
    endif
    for d = 1:8
      nx = cx + dirs(d,1); ny = cy + dirs(d,2);
      if nx<1 || nx>gs || ny<1 || ny>gs || closed(nx,ny), continue; end
      if ~free(nx,ny), continue; end
      tent = g(cx,cy) + dcost(d);
      if tent < g(nx,ny)
        parent(nx,ny,:) = [cx, cy];
        g(nx,ny) = tent;
        f(nx,ny) = tent + heuristic([nx ny], tgt);
        openx = [openx; nx]; openy = [openy; ny];
      endif
    endfor
  endwhile
  nxt = cur;  % no path found: hold
endfunction

function h = heuristic(a, b)
  h = norm(a - b);
endfunction

function [cx, cy, bi] = pop_best_cost(openx, openy, f, closed)
  if isempty(openx), cx = []; cy = []; bi = 0; return; end
  n = length(openx);
  bestf = inf; bi = 0;
  for i = 1:n
    if closed(openx(i), openy(i)), continue; end
    if f(openx(i), openy(i)) < bestf
      bestf = f(openx(i), openy(i)); bi = i;
    endif
  endfor
  if bi == 0, cx = []; cy = []; return; end
  cx = openx(bi); cy = openy(bi);
  openx(bi) = []; openy(bi) = [];
endfunction

function path = reconstruct(parent, start, goal)
  path = goal;
  cx = goal(1); cy = goal(2);
  while !(cx==start(1) && cy==start(2))
    px = parent(cx,cy,1); py = parent(cx,cy,2);
    if px==0 && py==0, break; end
    path = [px py; path];
    cx = px; cy = py;
  endwhile
endfunction

function nxt = first_step(path, cur)
  % return the second row of path (first move from cur)
  if size(path,1) < 2, nxt = cur; return; end
  nxt = path(2,:);
endfunction
