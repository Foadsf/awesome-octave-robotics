% demos/03_legged_locomotion.m — Legged locomotion: capture-point footstep
% placement (biped), trot gait scheduling (quadruped), and a tiny policy net.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/03_legged_locomotion.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

%% Biped capture-point footstep placement
x = 0.05; xd = 0.5; step_length = 0.35; n_steps = 6; goal = 1.2;
[footsteps, cps] = biped_footstep_planner(x, xd, step_length, n_steps, goal);
printf("03_legged_locomotion: footsteps = %s\n", ...
    mat2str(footsteps', 4));
printf("03_legged_locomotion: capture points = %s\n", mat2str(cps', 4));
capture_error = abs(goal - cps(end));
printf("03_legged_locomotion: final capture error = %.3f m\n", capture_error);

%% Quadruped trot gait
T = 1.0; dt = 0.02; v_body = [0.2, 0.0]; yaw_rate = 0.2;
[contacts, foot_pos] = quadruped_trot(T, dt, v_body, yaw_rate);
stance_pct = 100 * mean(contacts(:));
printf("03_legged_locomotion: trot stance duty = %.1f%%\n", stance_pct);
% verify diagonal pairs alternate (FL & RR together, FR & RL together)
checkA = all(contacts(:,1) == contacts(:,4));
checkB = all(contacts(:,2) == contacts(:,3));
printf("03_legged_locomotion: diagonal pairing FL==RR: %d, FR==RL: %d\n", checkA, checkB);

%% Tiny policy forward pass
obs = [0.15; 0.0; 0.1; 0.02; -0.01; 1; 1; 0; 1];  % [vx vy wz roll pitch contacts]
[action, info] = legged_rl_policy(obs);
printf("03_legged_locomotion: policy action = %s\n", mat2str(action', 4));

%% Plot: biped capture points & footsteps + quadruped foot trajectories
P = plot_helpers();
hf = P.newfig("03 legged locomotion — footsteps + trot");
% capture points as points over walking direction (x-axis), zero y
P.plot_points(hf, [cps, zeros(size(cps))], "b.", "MarkerSize", 6);
plot([0, goal], [0, 0], "k--", "LineWidth", 1);
P.plot_trajectory(hf, footsteps, zeros(size(footsteps)), "ro", "LineWidth", 0, "MarkerSize", 5);
% quadruped FL foot swing trajectory (leg 1)
P.plot_trajectory(hf, foot_pos(:,1,1), foot_pos(:,1,3), "-g", "LineWidth", 1.5);
P.set_limits(hf, -0.2, 1.6, -0.6, 0.6);
out = fullfile(repo_root, "output", "figures", "03_legged_locomotion.png");
P.savefig(hf, out);
printf("03_legged_locomotion: wrote %s\n", out);
