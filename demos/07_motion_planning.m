% demos/07_motion_planning.m — Motion planning: RRT among circular obstacles.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/07_motion_planning.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

start = [0, 0]; goal = [8, 6];
obstacles = [3, 2, 0.9; 5, 4, 1.0; 6, 1.5, 0.8; 2, 5, 0.7];  % [cx cy r]
bounds = [-1, 9, -1, 8];

[path, tree] = rrt(start, goal, obstacles, bounds, 3000, 0.4, 0.15);

P = plot_helpers();
hf = P.newfig("07 motion planning — RRT among obstacles");
if ~isempty(path) && size(path,1) > 1
  P.plot_trajectory(hf, path(:,1), path(:,2), "-r", "LineWidth", 2);
  plot(start(1), start(2), "go", "MarkerFaceColor", "g", "MarkerSize", 8);
  plot(goal(1), goal(2), "bx", "MarkerSize", 10, "LineWidth", 2);
else
  warning("RRT: no path found");
  plot(start(1), start(2), "go", "MarkerFaceColor", "g", "MarkerSize", 8);
  plot(goal(1), goal(2), "bx", "MarkerSize", 10, "LineWidth", 2);
end
thc = linspace(0, 2*pi, 60)';
for i = 1:size(obstacles,1)
  plot(obstacles(i,1)+obstacles(i,3)*cos(thc), obstacles(i,2)+obstacles(i,3)*sin(thc), "k-", "LineWidth", 1.5);
end
xlabel("x (m)"); ylabel("y (m)");
legend("RRT path", "start", "goal", "obstacles", "location", "best");

  if ~isempty(path) && size(path,1) > 1, status = "ok"; else, status = "FAILED"; end
  if isempty(path), npts = 0; else, npts = size(path,1); end
  fprintf("  RRT: %s  path points=%d\n", status, npts);

P.set_limits(hf, -1, 9, -1, 8);
out = fullfile(repo_root, "output", "figures", "07_motion_planning.png");
P.savefig(hf, out);
printf("07_motion_planning: wrote %s\n", out);
