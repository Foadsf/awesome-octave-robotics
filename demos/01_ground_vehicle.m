% demos/01_ground_vehicle.m — Ground vehicle: differential-drive path following
% with pure pursuit + unicycle kinematics (RK4 integration).
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/01_ground_vehicle.m')"
% (self-contained: adds paths via setup_paths, writes output/figures/01_ground_vehicle.png)

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

%% waypoint path (a loop)
path = [ 0   0;
         4   0;
         8   1;
        10   4;
         9   8;
         5   9;
         1   8;
        -1   4;
        -1   0];        % Nx2 waypoints
path = [path; path(1,:)];  % close loop -> 10x2

%% simulation
x = [0; 0; 0];           % [px; py; theta]
v = 1.0;                 % m/s
Ld = 1.5;                % lookahead (m)
L  = 0.8;                % wheelbase (m)
dt = 0.05;               % s
nsteps = 700;
traj = zeros(nsteps, 3);
for k = 1:nsteps
  traj(k, :) = x';
  [~, omega, ~, ~, ~] = pure_pursuit_steer(x, path, Ld, L, v);
  [x, ~] = unicycle_step(x, v, omega, dt);
end

%% plot
P = plot_helpers();
hf = P.newfig("01 ground vehicle — pure pursuit path following");
P.plot_trajectory(hf, traj(:,1), traj(:,2), "-b", "LineWidth", 1.5);
plot(path(:,1), path(:,2), "k--", "LineWidth", 1.2);   % reference path
thE = traj(end, 3);
Tend = [cos(thE), -sin(thE), traj(end,1);
        sin(thE),  cos(thE), traj(end,2);
        0,         0,        1];
P.plot_frame2d(hf, Tend, 1.0);
P.set_limits(hf, -3, 11, -2, 11);
out = fullfile(repo_root, "output", "figures", "01_ground_vehicle.png");
P.savefig(hf, out);
printf("01_ground_vehicle: wrote %s (final pose [%.2f %.2f %.2f])\n", ...
    out, traj(end,1), traj(end,2), traj(end,3));
