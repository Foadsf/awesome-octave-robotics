% demos/08_motion_control.m — Motion control: LQR tracking of a unicycle along
% a straight reference (lateral + heading regulation).
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/08_motion_control.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

[t, X, U, K] = lqr_track(1.0, 0.02, 8.0, 0.5, 0.2);

P = plot_helpers();
hf = P.newfig("08 motion control — LQR straight-line tracking");
P.plot_trajectory(hf, X(:,1), X(:,2), "-b", "LineWidth", 1.5);
plot(0, 0, "go", "MarkerFaceColor", "g", "MarkerSize", 6);
plot(X(end,1), X(end,2), "rx", "MarkerSize", 8);
xlabel("x (m)"); ylabel("y (m) — should converge to 0");
legend("tracked path", "start (y=0.5)", "end", "location", "best");

fprintf("  LQR: K=[%.3f %.3f]  final y=%.3f  final theta=%.3f\n", K(1), K(2), X(end,2), X(end,3));

hf2 = P.newfig("08 motion control — lateral error");
P.plot_points(hf2, [t, X(:,2)], "-b-", "LineWidth", 1.5);
P.plot_points(hf2, [t, X(:,3)], "r--", "LineWidth", 1.5);
xlabel("t (s)"); ylabel("error");
legend("lateral error y (m)", "heading error theta (rad)", "location", "best");
fprintf("  max |y|=%.3f m  max |theta|=%.3f rad\n", max(abs(X(:,2))), max(abs(X(:,3))));

out = fullfile(repo_root, "output", "figures", "08_motion_control.png");
P.savefig(hf, out);
out2 = fullfile(repo_root, "output", "figures", "08_motion_control_error.png");
P.savefig(hf2, out2);
printf("08_motion_control: wrote %s\n", out);
