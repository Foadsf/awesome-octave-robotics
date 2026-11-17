% demos/02_manipulation.m — Manipulation: analytic inverse kinematics of a
% 2-link planar arm reaching a target, with forward-kinematics verification.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/02_manipulation.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

l1 = 1.0; l2 = 0.8;          % link lengths (m)
p_target = [1.2; 0.5];       % target end-effector position (m)

% analytic IK: two elbow solutions
[q, reachable] = ik_2link(p_target, l1, l2);

P = plot_helpers();
hf = P.newfig("02 manipulation — 2-link arm IK");

% target point
P.plot_points(hf, p_target', "r*", "MarkerSize", 12, "MarkerFaceColor", "r");

colors = ["m", "c"];
for i = 1:2
  qi = q(:, i);
  [p_ee, T1, T2] = fk_2link(qi, l1, l2);
  % link geometry: base -> joint1 (end of link1) -> end effector
  j1 = [l1 * cos(qi(1)); l1 * sin(qi(1))];
  plot([0 j1(1) p_ee(1)], [0 j1(2) p_ee(2)], ...
       "-", "Color", colors(i), "LineWidth", 3);
  plot(j1(1), j1(2), "o", "Color", colors(i), "MarkerSize", 7);
  % link frames
  P.plot_frame2d(hf, T1, 0.4);
  P.plot_frame2d(hf, T1 * T2, 0.4);
  fprintf("  solution %d: q1=%.3f q2=%.3f  ee=[%.3f %.3f]  err=%.2e\n", ...
      i, qi(1), qi(2), p_ee(1), p_ee(2), norm(p_ee - p_target));
end
fprintf("  reachable=%d  target=[%.2f %.2f]\n", reachable, p_target(1), p_target(2));

P.set_limits(hf, -1, 2.2, -1, 1.6);
out = fullfile(repo_root, "output", "figures", "02_manipulation.png");
P.savefig(hf, out);
printf("02_manipulation: wrote %s\n", out);
