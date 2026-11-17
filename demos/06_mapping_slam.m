% demos/06_mapping_slam.m — Mapping & SLAM: occupancy grid from range scans,
% plus ICP scan matching between the first and last scan.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/06_mapping_slam.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

obstacles = [4, 2, 0.5; 6, 5, 0.5; 2, 6, 0.5; 7, 1, 0.4];  % [cx cy r]
max_range = 8.0;
bearings = (-pi:0.15:pi)'; M = numel(bearings);
dt = 0.01;
% robot poses: straight line along x
xs = 0:0.75:6; K = numel(xs);
poses = [xs', zeros(K,1), zeros(K,1)];   % [x y theta]

scans = zeros(K, M);
for k = 1:K
  p = poses(k, 1:2)'; th = poses(k, 3);
  for b = 1:M
    ba = th + bearings(b); d = [cos(ba); sin(ba)];  % unit ray direction (2x1)
    best = max_range;
    for i = 1:size(obstacles,1)
      c = obstacles(i,1:2)'; r = obstacles(i,3);   % circle centre (2x1), radius
      f = p - c;                                   % (2x1)
      aa = d' * d; bb = 2 * (d' * f); cc = f' * f - r^2;  % scalars
      disc = bb^2 - 4 * aa * cc;
      if disc >= 0
        s1 = (-bb + sqrt(disc)) / (2 * aa); s2 = (-bb - sqrt(disc)) / (2 * aa);
        for s = [s1, s2]
          if s > 0.1 && s < best, best = s; end
        end
      end
    end
    scans(k,b) = best;
  end
end

% occupancy grid
[grid, origin] = occupancy_grid(scans, bearings, poses, 0.1, 120);
occ = grid > 0.6;
[ci, cj] = find(occ);
wx = origin(1) + (cj - 0.5)*0.1;
wy = origin(2) + (ci - 0.5)*0.1;

P = plot_helpers();
hf = P.newfig("06 mapping / SLAM — occupancy grid + ICP");
P.plot_points(hf, [wx, wy], ".", "MarkerSize", 2, "Color", [0.6 0.6 0.6]);
thc = linspace(0, 2*pi, 60)';
for i = 1:size(obstacles,1)
  plot(obstacles(i,1) + obstacles(i,3)*cos(thc), ...
       obstacles(i,2) + obstacles(i,3)*sin(thc), "k-", "LineWidth", 1.5);
  plot(obstacles(i,1), obstacles(i,2), "k.", "MarkerSize", 6);
end
plot(poses(:,1), poses(:,2), "k.-", "LineWidth", 1);
xlabel("x (m)"); ylabel("y (m)");
legend("occupied cells", "obstacles", "robot path", "location", "best");

% ICP: align first scan cloud onto last scan cloud (relative motion)
to_cloud = @(s) [s .* cos(bearings), s .* sin(bearings)];
P0 = to_cloud(scans(1,:)); Pk = to_cloud(scans(K,:));
[T, cost, iters] = icp(P0, Pk, 100, 1e-5);
% true relative pose from pose1 to poseK (pure translation along x)
dth = poses(K,3) - poses(1,3);
Ttrue = [cos(dth), -sin(dth), poses(K,1)-poses(1,1);
         sin(dth),  cos(dth), poses(K,2)-poses(1,2);
         0, 0, 1];
terr = norm(T(1:2,3) - Ttrue(1:2,3));
rerr = abs(atan2(T(2,1),T(1,1)) - dth);
fprintf("  ICP: iters=%d  cost=%.4f  trans_err=%.3f m  rot_err=%.3f rad\n", ...
    iters, cost, terr, rerr);

P.set_limits(hf, -1, 8, -1, 8);
out = fullfile(repo_root, "output", "figures", "06_mapping_slam.png");
P.savefig(hf, out);
printf("06_mapping_slam: wrote %s\n", out);
