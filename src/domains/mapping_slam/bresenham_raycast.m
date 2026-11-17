% bresenham_raycast.m — 2D grid raycasting via Bresenham's algorithm.
%
%   cells = bresenham_raycast(x0, y0, x1, y1)
%
%   Returns integer grid cell coordinates (Nx2 [col row]) traversed by the
%   line from (x0,y0) to (x1,y1), including the endpoint. Coordinates are
%   already in integer grid-cell space.
%
%   Pure Octave native code.

function cells = bresenham_raycast(x0, y0, x1, y1)
  x0 = round(x0); y0 = round(y0); x1 = round(x1); y1 = round(y1);
  dx = abs(x1 - x0); dy = abs(y1 - y0);
  sx = bres_sx(x0, x1); sy = bres_sy(y0, y1);
  err = dx - dy;
  cells = [];
  x = x0; y = y0;
  while 1
    cells = [cells; x, y];
    if (x == x1 && y == y1), break; end
    e2 = 2 * err;
    if (e2 > -dy)
      err = err - dy; x = x + sx;
    endif
    if (e2 < dx)
      err = err + dx; y = y + sy;
    endif
  endwhile
endfunction

function s = bres_sx(a, b)
  if a < b, s = 1; else s = -1; end
endfunction

function s = bres_sy(a, b)
  if a < b, s = 1; else s = -1; end
endfunction
