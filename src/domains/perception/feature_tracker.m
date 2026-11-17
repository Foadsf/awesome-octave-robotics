% feature_tracker.m — Corner detection + patch descriptor matching.
%
%   corners = feature_tracker(I, "detect", opts)           % detect corners
%   matches = feature_tracker(I1, I2, "match", opts)       % detect + match
%
%   I, I1, I2 : grayscale images (2D double matrices, any scale)
%   opts : struct with optional fields
%     .method   : "harris" (default) | "shi_tomasi"
%     .k        : Harris detector free parameter (default 0.04)
%     .thresh   : corner response threshold (default 1e-4)
%     .nms      : non-max suppression radius in px (default 3)
%     .patch    : half-width of descriptor patch (default 5 -> 11x11)
%     .metric   : "ncc" (default) | "ssd"
%     .ratio    : Lowe's ratio test threshold (default 0.8)
%     .max_corners: cap on detected corners (default 500)
%
%   "detect" returns corners (Mx2 [x y] column convention) sorted by response.
%   "match" returns matches (Kx2 [idx1 idx2] into each image's corner list).
%
%   Pure Octave native code (no image toolbox).

function varargout = feature_tracker(varargin)
  % parse: feature_tracker(I, mode, opts) or feature_tracker(I1, I2, mode, opts)
  if nargin < 3, error("feature_tracker: need at least I, mode, opts"); end
  I1 = varargin{1};
  I2 = [];
  if nargin == 3
    mode = varargin{2}; opts = varargin{3};
  else
    I2 = varargin{2}; mode = varargin{3}; opts = varargin{4};
  endif
  if ~isstruct(opts), opts = struct(); end
  method = pc_pick(opts, "method", "harris");
  k = pc_pick(opts, "k", 0.04);
  thresh = pc_pick(opts, "thresh", 1e-4);
  nms_r = pc_pick(opts, "nms", 3);
  patch = pc_pick(opts, "patch", 5);
  metric = pc_pick(opts, "metric", "ncc");
  ratio = pc_pick(opts, "ratio", 0.8);
  max_corners = pc_pick(opts, "max_corners", 500);

  if strcmpi(mode, "detect")
    corners = detect_corners(I1, method, k, thresh, nms_r, max_corners);
    varargout{1} = corners;
  elseif strcmpi(mode, "match")
    if isempty(I2), error("feature_tracker: 'match' needs I2"); end
    c1 = detect_corners(I1, method, k, thresh, nms_r, max_corners);
    c2 = detect_corners(I2, method, k, thresh, nms_r, max_corners);
    matches = match_patches(I1, c1, I2, c2, patch, metric, ratio);
    varargout{1} = matches;
    varargout{2} = c1;
    varargout{3} = c2;
  else
    error("feature_tracker: unknown mode '%s'", mode);
  endif
endfunction

function C = detect_corners(I, method, k, thresh, nms_r, max_corners)
  % image gradients (central differences)
  [Ix, Iy] = image_gradients(I);
  Ix2 = Ix.^2; Iy2 = Iy.^2; Ixy = Ix .* Iy;
  % gaussian smoothing of second-moment entries (3x3 separable)
  g = ([1 2 1] / 4)';
  G = g * g.';
  Ix2 = conv2(Ix2, G, "same");
  Iy2 = conv2(Iy2, G, "same");
  Ixy = conv2(Ixy, G, "same");
  if strcmpi(method, "shi_tomasi")
    R = min(eig2x2(Ix2, Iy2, Ixy), [], 2);
  else
    R = Ix2 .* Iy2 - Ixy.^2 - k * (Ix2 + Iy2).^2;
  endif
  % threshold
  cand = find(R > thresh);
  if isempty(cand), C = zeros(0, 2); return; end
  [rows_c, cols_c] = ind2sub(size(R), cand);
  % corners are [x y] = [col row]
  pts = [cols_c, rows_c];
  resp = R(cand);
  % sort by descending response
  [~, ord] = sort(resp, "descend");
  pts = pts(ord, :); resp = resp(ord);
  % non-max suppression
  keep = nms_suppress(pts, resp, nms_r, max_corners);
  C = pts(keep, :);
endfunction

function [Ix, Iy] = image_gradients(I)
  [ny, nx] = size(I);
  Ix = zeros(ny, nx); Iy = zeros(ny, nx);
  Ix(:, 2:nx-1) = (I(:, 3:nx) - I(:, 1:nx-2)) / 2;
  Iy(2:ny-1, :) = (I(3:ny, :) - I(1:ny-2, :)) / 2;
  % edges: forward/backward difference
  Ix(:, 1) = I(:, 2) - I(:, 1); Ix(:, nx) = I(:, nx) - I(:, nx-1);
  Iy(1, :) = I(2, :) - I(1, :); Iy(ny, :) = I(ny, :) - I(ny-1, :);
endfunction

function m = eig2x2(a, b, c)
  % elementwise smaller eigenvalue of [[a, c],[c, b]] for 2D arrays
  m = (a + b) ./ 2 - sqrt(((a - b) ./ 2).^2 + c.^2);
endfunction

function keep = nms_suppress(pts, resp, radius, max_corners)
  n = rows(pts);
  keep = ones(n, 1);
  suppressed = false(n, 1);
  for i = 1:n
    if suppressed(i), continue; end
    for j = i+1:n
      if suppressed(j), continue; end
      if norm(pts(i,:) - pts(j,:)) <= radius
        if resp(j) > resp(i)
          suppressed(i) = true; break;
        else
          suppressed(j) = true;
        endif
      endif
    endfor
  endfor
  keep = find(~suppressed);
  if rows(keep) > max_corners
    keep = keep(1:max_corners);
  endif
endfunction

function matches = match_patches(I1, c1, I2, c2, patch, metric, ratio)
  if isempty(c1) || isempty(c2), matches = zeros(0, 2); return; end
  d1 = extract_patches(I1, c1, patch);
  d2 = extract_patches(I2, c2, patch);
  matches = zeros(0, 2);
  for i = 1:rows(d1)
    if metric == "ssd"
      d = sum((d2 - d1(i, :)).^2, 2);
    else
      d = ncc_similarity(d2, d1(i, :));
    endif
    [~, ord] = sort(d);   % ascending; SSD best=ord(1), NCC best=ord(end)
    if metric == "ssd"
      % lower is better; ratio test on best vs second
      best = ord(1); second = ord(2);
      if rows(ord) >= 2 && d(best) < ratio * d(second)
        matches = [matches; i, best];
      endif
    else
      % higher is better; ratio test on best vs second
      best = ord(end); second = ord(end - 1);
      if rows(ord) >= 2 && d(best) > ratio * d(second)
        matches = [matches; i, best];
      endif
    endif
  endfor
endfunction

function P = extract_patches(I, corners, patch)
  [ny, nx] = size(I);
  n = rows(corners);
  dim = (2 * patch + 1)^2;
  P = zeros(n, dim);
  half = patch;
  for i = 1:n
    cx = round(corners(i, 1)); cy = round(corners(i, 2));
    % clamp so the full patch stays inside the image
    cx = min(max(cx, half + 1), nx - half);
    cy = min(max(cy, half + 1), ny - half);
    patchm = I(cy - half : cy + half, cx - half : cx + half);
    P(i, :) = patchm(:)';
  endfor
endfunction

function s = ncc_similarity(D, q)
  % normalized cross-correlation of each row of D against q
  qm = q - mean(q); qs = sqrt(sum(qm.^2));
  Dm = D - mean(D, 2);
  Ds = sqrt(sum(Dm.^2, 2));
  denom = Ds * qs;
  denom(denom < 1e-12) = 1e-12;
  s = (Dm * qm') ./ denom;
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
