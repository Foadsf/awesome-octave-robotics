% demos/09_uav.m — UAV: quadrotor control allocation (mixer) and motor speeds.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/09_uav.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

% Command a roll manoeuvre while hovering (total thrust = weight)
weight = 4.0;   % N
wrench_hover = [weight; 0; 0; 0];
wrench_roll  = [weight; 2.0; 0; 0];  % + roll torque
[speed_h, thrust_h] = mixer(wrench_hover);
[speed_r, thrust_r] = mixer(wrench_roll);

P = plot_helpers();
hf = P.newfig("09 UAV — quadrotor mixer: motor thrust");
x = 1:4;
P.plot_points(hf, [x', thrust_h], "-bo", "LineWidth", 1.5, "MarkerSize", 6);
P.plot_points(hf, [x', thrust_r], "rx--", "LineWidth", 1, "MarkerSize", 6);
xlabel("motor"); ylabel("thrust (N)");
legend("hover", "roll command", "location", "best");

fprintf("  hover   thrust=[%.3f %.3f %.3f %.3f] speed=[%.3f %.3f %.3f %.3f]\n", ...
    thrust_h(1),thrust_h(2),thrust_h(3),thrust_h(4), speed_h(1),speed_h(2),speed_h(3),speed_h(4));
fprintf("  roll    thrust=[%.3f %.3f %.3f %.3f] (right motors push more)\n", ...
    thrust_r(1),thrust_r(2),thrust_r(3),thrust_r(4));

out = fullfile(repo_root, "output", "figures", "09_uav.png");
P.savefig(hf, out);
printf("09_uav: wrote %s\n", out);
