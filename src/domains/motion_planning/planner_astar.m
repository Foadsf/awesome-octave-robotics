% planner_astar.m — 8-connected grid A* with octile heuristic.
%
%   path = planner_astar(grid, start, goal)
%
%   grid : 2D logical/double occupancy grid (1 = occupied, 0 = free)
%   start, goal : [row, col] or [x, y] (row, col convention here)
%
%   Returns path (Nx2 [row col]) or [] if no path.
%
%   Pure Octave native code.

function path = planner_astar(grid, start, goal)
  [nrows, ncols] = size(grid);
  start = round(start); goal = round(goal);
  if grid(start(1), start(2)) != 0 || grid(goal(1), goal(2)) != 0
    path = []; return;
  endif
  % 8-connected moves
  moves = [-1 0 10; 1 0 10; 0 -1 10; 0 1 10; ...
           -1 -1 14; -1 1 14; 1 -1 14; 1 1 14];
  % octile heuristic
  h = @(a, b) 10 * max(abs(a(1)-b(1)), abs(a(2)-b(2))) ...
            + 4 * min(abs(a(1)-b(1)), abs(a(2)-b(2)));
  open = struct("cell", start, "g", 0, "h", h(start, goal), "parent", []);
  openset = {open};
  closed = false(nrows, ncols);
  gscore = inf(nrows, ncols);
  gscore(start(1), start(2)) = 0;
  came_from = zeros(nrows, ncols, 2);
  found = false;
  while ~isempty(openset)
    if isempty(openset), break; end
    % pop lowest f = g + h
    f = zeros(rows(openset), 1);
    for i = 1:numel(openset)
      f(i) = openset{i}.g + openset{i}.h;
    endfor
    [~, bi] = min(f);
    cur = openset{bi};
    openset(bi) = [];
    c = cur.cell;
    if c(1) == goal(1) && c(2) == goal(2)
      found = true; break;
    endif
    closed(c(1), c(2)) = true;
    for m = 1:rows(moves)
      nb = c + moves(m, 1:2);
      if nb(1) < 1 || nb(1) > nrows || nb(2) < 1 || nb(2) > ncols, continue; end
      if grid(nb(1), nb(2)) != 0 || closed(nb(1), nb(2)), continue; end
      tentative_g = cur.g + moves(m, 3);
      if tentative_g < gscore(nb(1), nb(2))
        came_from(nb(1), nb(2), 1) = c(1); came_from(nb(1), nb(2), 2) = c(2);
        gscore(nb(1), nb(2)) = tentative_g;
        % check if already in openset
        inset = false; bi2 = 0;
        for i = 1:numel(openset)
          if openset{i}.cell(1) == nb(1) && openset{i}.cell(2) == nb(2)
            inset = true; bi2 = i; break;
          endif
        endfor
        node = struct("cell", nb, "g", tentative_g, "h", h(nb, goal), "parent", c);
        if inset
          if tentative_g < openset{bi2}.g
            openset{bi2} = node;
          endif
        else
          openset = [openset, {node}];
        endif
      endif
    endfor
  endwhile

  if ~found, path = []; return; end
  % reconstruct
  path = goal;
  cur = goal;
  while !(cur(1) == start(1) && cur(2) == start(2))
    cur = [came_from(cur(1), cur(2), 1), came_from(cur(1), cur(2), 2)];
    path = [cur; path];
  endwhile
endfunction
