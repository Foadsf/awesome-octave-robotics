% occupancy_grid.m — 2D occupancy grid from range scans (log-odds fusion).
%
%   [grid, origin] = occupancy_grid(scans, bearings, poses, res, n)
%
%   scans   : KxM ranges (m) (or Mx1 for a single scan)
%   bearings: Mx1 beam bearings (rad) relative to robot heading
%   poses   : Kx3 robot poses [x y theta] (m, m, rad)
%   res     : cell resolution (m)
%   n       : grid cells (n x n), centred on the world origin
%
%   Inverse sensor model (log-odds): cells along each beam are marked free,
%   the endpoint cell is marked occupied. Log-odds are fused across scans and
%   converted to probability p = 1/(1+exp(-l)).

function [grid, origin] = occupancy_grid(scans, bearings, poses, res, n)
  scans = reshape(scans, [], size(scans,2));  % KxM
  bearings = reshape(bearings, [], 1);
  poses = reshape(poses, [], 3);
  K = size(poses, 1); M = size(bearings, 1);
  l = zeros(n);                             % log-odds
  lo_free = log(0.4/0.6); lo_occ = log(0.6/0.4);
  origin = [-n*res/2, -n*res/2];
  w2c = @(w) round((w - origin(1))/res) + 1;   % world -> cell (1-based)
  inbounds = @(c) (c>=1 && c<=n);
  half = res/2;
  for k = 1:K
    px = poses(k,1); py = poses(k,2); th = poses(k,3);
    for b = 1:M
      r = scans(k,b);
      if ~isfinite(r) || r <= 0, continue; end
      ba = th + bearings(b);
      vx = cos(ba); vy = sin(ba);
      % occupied hit cell (computed first so the free pass can skip it)
      wxh = px + r*vx; wyh = py + r*vy;
      hit_ci = w2c(wyh); hit_cj = w2c(wxh);
      % free cells along the ray (stop short of the hit cell)
      smax = min(r - half, r);
      ns = max(1, ceil(smax / half));
      for s = half:half:smax
        wx = px + s*vx; wy = py + s*vy;
        ci = w2c(wy); cj = w2c(wx);
        if (ci ~= hit_ci || cj ~= hit_cj) && inbounds(ci) && inbounds(cj)
          l(ci,cj) = l(ci,cj) + lo_free;
        end
      end
      % occupied hit cell
      if inbounds(hit_ci) && inbounds(hit_cj), l(hit_ci,hit_cj) = l(hit_ci,hit_cj) + lo_occ; end
    end
  end
  grid = 1 ./ (1 + exp(-l));   % probability in (0,1)
endfunction
