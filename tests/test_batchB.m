% tests/test_batchB.m — Unit tests for Batch B domains
% (05 perception, 06 mapping/SLAM, 07 motion planning, 08 motion control).
%
% Run: octave-cli --no-gui --quiet --eval "run('setup_paths.m'); run('tests/test_batchB.m')"
% Exit code 0 iff every check passes.

global failures;
failures = 0;

function check(cond, msg)
  global failures;
  if cond, printf("  PASS: %s\n", msg);
  else, printf("  FAIL: %s\n", msg); failures = failures + 1; end
endfunction

repo_root = fileparts(fileparts(mfilename("fullpath")));  % tests/ -> repo root
run(fullfile(repo_root, "setup_paths.m"));

printf("=== 05 perception ===\n");
% complementary filter tracks a slow roll manoeuvre (body rates ~= Euler rates,
% yaw = 0). Deterministic signal so the check is reproducible.
dt = 0.01; t = (0:dt:2.0)'; N = numel(t);
true_roll = 0.20 * sin(t); true_pitch = 0.10 * sin(1.5 * t);
gyro = [0.20*cos(t), 0.15*cos(1.5*t), zeros(N,1)];   % exact derivative, no noise
g = 9.81;
accel = [-g*sin(true_pitch), g*sin(true_roll).*cos(true_pitch), ...
         g*cos(true_roll).*cos(true_pitch)];
[ang, R] = complementary_filter(gyro, accel, dt, 0.98, [0 0 0]);
check(size(ang) == [N 3] && all(isfinite(ang(:))), "complementary_filter angles Nx3 finite");
check(size(R) == [3 3 N] && all(isfinite(R(:))), "complementary_filter R is 3x3xN finite");
check(sqrt(mean((ang(:,1)-true_roll).^2)) < 1e-3, "complementary_filter roll tracks truth");
check(sqrt(mean((ang(:,2)-true_pitch).^2)) < 1e-3, "complementary_filter pitch tracks truth");
% raw gyro integration drifts; the filter must be strictly better
ang_int = cumsum(gyro(:,1:2)) * dt;   % no accel correction
check(sqrt(mean((ang(:,1)-true_roll).^2)) < sqrt(mean((ang_int(:,1)-true_roll).^2)), ...
      "filter beats uncorrected gyro integration on roll");

% feature tracker: detect corners on a synthetic image with a bright square corner
I = zeros(60); I(25:35, 25:35) = 1;
c1 = feature_tracker(I, "detect", struct());
check(rows(c1) > 0 && all(isfinite(c1(:))), "feature_tracker detects corners on synthetic image");
% two images sharing one bright square feature -> at least one match
I1 = zeros(60); I1(25:35, 25:35) = 1;
I2 = zeros(60); I2(25:35, 25:35) = 1; I2(45:52, 45:52) = 0.5;
[m, ~, ~] = feature_tracker(I1, I2, "match", struct());
check(rows(m) >= 1, "feature_tracker matches shared feature between images");

printf("=== 06 mapping / SLAM ===\n");
% ICP recovers a known rigid transform applied to a synthetic point cloud
rng(0);
P = rand(200, 2) * 4;                         % source cloud
th_true = 0.4; t_true = [1.5; -0.8];
Rt = [cos(th_true), -sin(th_true); sin(th_true), cos(th_true)];
Q = P * Rt' + t_true';                        % target cloud (row convention)
[Ticp, cost, iters] = icp(P, Q, 100, 1e-6);
Rrec = Ticp(1:2,1:2); trec = Ticp(1:2,3);
errR = norm(Rrec - Rt, 'fro'); errT = norm(trec - t_true);
check(errR < 0.05 && errT < 0.05, sprintf("icp recovers known transform (dR=%.3f dT=%.3f)", errR, errT));
% ICP cost should be small after convergence
check(cost < 1e-2, sprintf("icp converged (cost=%.4f)", cost));

% occupancy grid from a single beam hitting an obstacle at (1.5, 0)
bearings = 0; scans = [1.5]; poses = [0 0 0];
[grid, origin] = occupancy_grid(scans, bearings, poses, 0.1, 40);
check(size(grid) == [40 40] && all(grid(:) >= 0) && all(grid(:) <= 1), ...
      "occupancy_grid returns 40x40 probabilities in [0,1]");
ci0 = round((0 - origin(2))/0.1) + 1;         % cell for world y=0
cj_occ = round((1.5 - origin(1))/0.1) + 1;    % obstacle cell (x=1.5)
cj_free = round((0.6 - origin(1))/0.1) + 1;   % free cell along the ray (x=0.6)
check(grid(ci0, cj_occ) > 0.5, "occupancy_grid marks hit cell occupied");
check(grid(ci0, cj_free) < 0.5, "occupancy_grid marks along-ray cell free");

printf("=== 07 motion planning ===\n");
obstacles = [4, 2, 0.7; 6, 4, 0.6];  % [cx cy r]
bounds = [-1, 9, -1, 9];
[start, goal] = deal([0, 0], [8, 8]);
[path, ~] = rrt(start, goal, obstacles, bounds, 4000, 0.4, 0.15);
check(~isempty(path) && size(path,1) >= 2, "rrt finds a path around obstacles");
check(norm(path(1,:) - start) < 1e-9 && norm(path(end,:) - goal) < 0.5, ...
      "rrt path starts at start and reaches goal");
% every segment collision-free
collision = false;
for k = 1:size(path,1)-1
  for i = 1:size(obstacles,1)
    d = path(k+1,:) - path(k,:); L2 = d*d';
    c = obstacles(i,1:2); r = obstacles(i,3);
    tt = max(0, min(1, ((c - path(k,:))*d')/L2));
    closest = path(k,:) + tt * d;
    if norm(closest - c) <= r, collision = true; end
  end
end
check(~collision, "rrt path is collision-free");
% goal deep inside an obstacle -> no path
[path2, ~] = rrt([0,0], [4,2], [4,2,0.7], bounds, 2000, 0.4, 0.15);
check(isempty(path2), "rrt returns [] when goal is inside an obstacle");

printf("=== 08 motion control ===\n");
[t, X, U, K] = lqr_track(1.0, 0.02, 6.0, 0.5, 0.2);
check(all(isfinite(K(:))) && all(isfinite(U(:))) && all(isfinite(X(:))), ...
      "lqr_track finite gain, control, states");
check(abs(X(end,2)) < 0.05 && abs(X(end,3)) < 0.05, ...
      sprintf("lqr_track converges (final y=%.3f theta=%.3f)", X(end,2), X(end,3)));
check(max(abs(X(:,2))) < 0.6, "lqr_track bounded lateral response");

printf("=== summary ===\n");
if failures == 0
  printf("ALL Batch B domain checks PASS\n");
  exit(0);
else
  printf("%d Batch B domain check(s) FAILED\n", failures);
  exit(1);
endif
