% pointcloud_proc.m — Point cloud processing: voxel-grid downsampling,
% statistical outlier removal, and RANSAC ground-plane segmentation.
%
%   [down, idx] = pointcloud_proc(P, leaf_size)             % voxel downsample
%   [inl, out] = pointcloud_proc(P, [], k_std)              % statistical outlier
%   [plane, inliers] = pointcloud_proc(P, [], [], ransac)   % RANSAC plane
%
%   P      : Nx3 point cloud (rows are points)
%   leaf_size : voxel edge length (m); [] = skip downsampling
%   k_std  : std-dev multiplier for statistical outlier removal; [] = skip
%   ransac : struct with fields .iterations, .distance_threshold; [] = skip
%
%   Returns depend on which stage is requested. When multiple stages given,
%   they are chained: downsample -> outlier removal -> RANSAC plane fit.
%
%   Pure Octave native code.

function varargout = pointcloud_proc(P, leaf_size, k_std, ransac)
  if nargin < 2, leaf_size = []; end
  if nargin < 3, k_std = []; end
  if nargin < 4, ransac = []; end

  Q = P;
  % --- stage 1: voxel-grid downsampling ---
  if ~isempty(leaf_size) && leaf_size > 0
    Q = pc_voxel_downsample(Q, leaf_size);
  endif

  % --- stage 2: statistical outlier removal ---
  inl = []; out = [];
  if ~isempty(k_std) && k_std > 0
    [inl, out] = pc_statistical_outlier(Q, k_std);
    Q = Q(inl, :);
  endif

  % --- stage 3: RANSAC ground-plane segmentation ---
  plane = []; inliers = [];
  if ~isempty(ransac) && isfield(ransac, "iterations") && ransac.iterations > 0
    [plane, inliers] = pc_ransac_plane(Q, ransac.iterations, ...
        pc_pick(ransac, "distance_threshold", 0.05));
  endif

  varargout{1} = Q;
  if nargout >= 2, varargout{2} = inl; end
  if nargout >= 3, varargout{3} = out; end
  if nargout >= 4, varargout{4} = plane; end
  if nargout >= 5, varargout{5} = inliers; end
endfunction

function Q = pc_voxel_downsample(P, leaf)
  keys = floor(P ./ leaf);
  n = rows(P);
  keybuf = cell(n, 1);
  for i = 1:n
    keybuf{i} = sprintf("%d,%d,%d", keys(i,1), keys(i,2), keys(i,3));
  endfor
  [~, ~, idx] = unique(keybuf);
  Q = zeros(max(idx), size(P, 2));
  for i = 1:max(idx)
    sel = (idx == i);
    Q(i, :) = mean(P(sel, :), 1);
  endfor
endfunction

function [inl, out] = pc_statistical_outlier(P, k_std)
  n = rows(P);
  if n < 2, inl = 1:n; out = []; return; end
  kk = min(8, n - 1);
  D = pc_sq_distances(P);
  [~, order] = sort(D, 2);
  knn = order(:, 2:kk+1);
  md = zeros(n, 1);
  for i = 1:n
    md(i) = mean(sqrt(D(i, knn(i, :))));
  endfor
  mu = mean(md); sigma = std(md);
  threshold = mu + k_std * sigma;
  inl = find(md <= threshold);
  out = find(md > threshold);
endfunction

function D = pc_sq_distances(P)
  n = rows(P);
  G = P * P';
  sq = diag(G);
  D = sq(:, ones(1, n)) + sq(ones(1, n), :) - 2 * G;
  D(D < 0) = 0;
endfunction

function [plane, inliers] = pc_ransac_plane(P, iters, dist_thr)
  n = rows(P);
  plane = zeros(4, 1); inliers = [];
  if n < 3, return; end
  best_n = 0; best_plane = zeros(4, 1);
  rng(0);
  for it = 1:iters
    ids = randperm(n, 3);
    p1 = P(ids(1), :); p2 = P(ids(2), :); p3 = P(ids(3), :);
    normal = cross(p2 - p1, p3 - p1);
    nn = norm(normal);
    if nn < 1e-12, continue; end
    normal = normal / nn;
    d = -normal * p1';
    dist = abs(P * normal' + d);
    inl = find(dist < dist_thr);
    if rows(inl) > best_n
      best_n = rows(inl);
      best_plane = [normal; d];
    endif
  endfor
  plane = best_plane;
  if best_n > 0
    normal = best_plane(1:3); d = best_plane(4);
    dist = abs(P * normal' + d);
    inliers = find(dist < dist_thr);
  endif
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
