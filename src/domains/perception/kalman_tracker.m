% kalman_tracker.m — Constant-velocity multi-object Kalman tracker.
%
%   tracks = kalman_tracker(detections, dt, opts)
%
%   detections : Kx2 matrix of [x y] detections at this time step (or [] if none)
%   dt         : time step (s)
%   opts       : struct with optional fields
%     .q          : process noise intensity (default 0.1)
%   .r          : measurement noise std (default 0.5)
%   .gate       : Euclidean gating distance (default 2.5)
%   .max_age    : frames a track survives without match (default 5)
%   .min_hits   : frames a track must be matched before confirmed (default 3)
%
%   Returns tracks: struct array, each with .id, .x (4x1 [x y vx vy]), .P,
%   .age, .hits, .confirmed.
%
%   Pure Octave native code.

function tracks = kalman_tracker(detections, dt, opts)
  if ~isstruct(opts), opts = struct(); end
  q = pc_pick(opts, "q", 0.1);
  r = pc_pick(opts, "r", 0.5);
  gate = pc_pick(opts, "gate", 2.5);
  max_age = pc_pick(opts, "max_age", 5);
  min_hits = pc_pick(opts, "min_hits", 3);

  persistent tracks_ next_id;
  if isempty(tracks_) || isempty(next_id)
    tracks_ = []; next_id = 1;
  endif

  % constant-velocity model: state [x y vx vy]
  F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
  H = [1 0 0 0; 0 1 0 0];
  Q = q * [dt^4/4 0 dt^3/2 0;
           0 dt^4/4 0 dt^3/2;
           dt^3/2 0 dt^2 0;
           0 dt^3/2 0 dt^2];
  Rm = r^2 * eye(2);

  nt = numel(tracks_);
  % --- predict all existing tracks ---
  for i = 1:nt
    t = tracks_(i);
    t.x = F * t.x;
    t.P = F * t.P * F' + Q;
    t.age = t.age + 1;
    tracks_(i) = t;
  endfor

  assigned_dets = [];
  % --- associate detections to tracks via gated nearest neighbor ---
  if ~isempty(detections) && nt > 0
    nd = rows(detections);
    cost = inf(nt, nd);
    for i = 1:nt
      z_pred = H * tracks_(i).x;
      S = H * tracks_(i).P * H' + Rm;
      Sinv = inv(S);
      for j = 1:nd
        dz = detections(j, :)' - z_pred;
        m = sqrt(dz' * Sinv * dz);   % Mahalanobis distance
        if m < gate
          cost(i, j) = m;
        endif
      endfor
    endfor
    % greedy assignment: smallest cost first
    assigned_tracks = false(nt, 1); assigned_dets = false(nd, 1);
    [c_order, ~] = sort(cost(:));
    for k = 1:numel(c_order)
      [ti, di] = ind2sub([nt, nd], c_order(k));
      if assigned_tracks(ti) || assigned_dets(di), continue; end
      if cost(ti, di) == inf, continue; end
      % Kalman update
      t = tracks_(ti);
      z = detections(di, :)';
      y = z - H * t.x;
      S = H * t.P * H' + Rm;
      K = t.P * H' * inv(S);
      t.x = t.x + K * y;
      t.P = (eye(4) - K * H) * t.P;
      t.age = 0; t.hits = t.hits + 1;
      if t.hits >= min_hits, t.confirmed = true; end
      tracks_(ti) = t;
      assigned_tracks(ti) = true; assigned_dets(di) = true;
    endfor
  endif

  % --- create new tracks for unmatched detections ---
  if ~isempty(detections)
    nd = rows(detections);
    for j = 1:nd
      if ~isempty(assigned_dets) && assigned_dets(j), continue; end
      x0 = [detections(j, 1); detections(j, 2); 0; 0];
      P0 = diag([1, 1, 10, 10]);
      t = struct("id", next_id, "x", x0, "P", P0, "age", 0, ...
                 "hits", 1, "confirmed", false);
      next_id = next_id + 1;
      if isempty(tracks_)
        tracks_ = t;
      else
        tracks_ = [tracks_; t];
      endif
    endfor
  endif

  % --- prune dead tracks ---
  nt = numel(tracks_);
  keep = true(nt, 1);
  for i = 1:nt
    if tracks_(i).age > max_age, keep(i) = false; end
  endfor
  if any(~keep)
    tracks_ = tracks_(keep);
  endif

  tracks = tracks_;
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
