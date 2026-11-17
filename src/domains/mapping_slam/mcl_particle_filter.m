% mcl_particle_filter.m — Monte Carlo Localization particle filter.
%
%   [particles, weights] = mcl_particle_filter(particles, weights, scan, map, u, opts)
%
%   particles : Nx3 [x y theta]
%   weights   : Nx1 (will be normalized)
%   scan      : Mx2 points in robot frame (range/bearing converted to xy)
%   map       : struct with .grid (occupancy prob 0..1), .resolution, .origin [x y]
%   u         : [dx dtheta] odometry motion since last step
%   opts      : struct (optional)
%     .alpha : motion noise [trans rot] (default [0.1 0.1])
%     .z_hit : hit prob (default 0.8)
%     .z_max : max-range prob (default 0.1)
%     .sigma_hit : hit sigma (default 0.1)
%
%   Returns resampled particles and normalized weights.
%
%   Pure Octave native code.

function [particles, weights] = mcl_particle_filter(particles, weights, scan, map, u, opts)
  if nargin < 6, opts = struct(); end
  if ~isstruct(opts), opts = struct(); end
  alpha = pc_pick(opts, "alpha", [0.1 0.1]);
  z_hit = pc_pick(opts, "z_hit", 0.8);
  z_max = pc_pick(opts, "z_max", 0.1);
  sigma_hit = pc_pick(opts, "sigma_hit", 0.1);

  N = rows(particles);
  % --- motion model: sample odometry with noise ---
  for i = 1:N
    d = u(1) + alpha(1)*randn();
    dt = u(2) + alpha(2)*randn();
    th = particles(i,3);
    particles(i,1) = particles(i,1) + d*cos(th);
    particles(i,2) = particles(i,2) + d*sin(th);
    particles(i,3) = atan2(sin(th+dt), cos(th+dt));
  endfor

  % --- sensor model: likelihood-field ---
  for i = 1:N
    w = 1;
    % transform scan into map frame
    th = particles(i,3);
    R = [cos(th), -sin(th); sin(th), cos(th)];
    scan_map = (R * scan')' + particles(i,1:2);
    % subsample scan for speed
    step = max(1, round(rows(scan_map)/40));
    for k = 1:step:rows(scan_map)
      p = scan_map(k,:);
      prob = map_likelihood(map, p);
      w = w * prob;
    endfor
    weights(i) = w;
  endfor

  % normalize
  ws = sum(weights);
  if ws > 0, weights = weights / ws; else weights = ones(N,1)/N; end

  % --- systematic resampling ---
  particles = systematic_resample(particles, weights);
  weights = ones(N, 1) / N;
endfunction

function p = map_likelihood(map, pt)
  % occupancy probability at point pt (interpolated nearest cell)
  res = map.resolution; ox = map.origin(1); oy = map.origin(2);
  grid = map.grid;
  [ny, nx] = size(grid);
  ci = round((pt(2) - oy) / res) + 1;
  cj = round((pt(1) - ox) / res) + 1;
  if ci < 1 || ci > ny || cj < 1 || cj > nx
    p = 0.5; return;  % unknown
  endif
  occ = grid(ci, cj);
  % likelihood: high if free, low if occupied
  p = 1 - occ;
  p = max(0.01, min(0.99, p));
endfunction

function particles = systematic_resample(particles, weights)
  N = rows(particles);
  r = rand() / N;
  c = weights(1);
  i = 1;
  particles_new = zeros(N, 3);
  for m = 1:N
    u = r + (m - 1) / N;
    while u > c && i < N
      i = i + 1;
      c = c + weights(i);
    endwhile
    particles_new(m, :) = particles(i, :);
  endfor
  particles = particles_new;
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
