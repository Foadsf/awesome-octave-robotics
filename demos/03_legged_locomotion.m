% demos/03_legged_locomotion.m — Legged locomotion: Linear Inverted Pendulum
% Model (LIPM) walking over a sequence of footsteps; tracks CoM, capture point
% and ZMP with a simple stability margin.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/03_legged_locomotion.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

z = 0.80;      % CoM height (m)
g = 9.81;
dt = 0.01;     % integration (s)
T = 0.5;       % step duration (s)
w = sqrt(g / z);            % LIPM frequency
x0 = 0.0; xd0 = 0.6;        % initial CoM position & velocity
nsteps = 7;
% Capture-point stepping: place each footstep at the current capture point
% x_c = x + xd/w, which is the stabilizing foot-placement rule for the LIPM.
zmp_seq = zeros(nsteps, 1);
x = x0; xd = xd0;
for k = 1:nsteps
  c = x + xd / w;     % next footstep at capture point
  zmp_seq(k) = c;
  % exact one-step LIPM update with ZMP held at c
  wh = w * T; ch = cosh(wh); sh = sinh(wh);
  x_new = c + (x - c) * ch + (xd / w) * sh;
  xd_new = w * (x - c) * sh + xd * ch;
  x = x_new; xd = xd_new;
end

[com, zmp, capture, stable] = lipm(z, g, dt, T, zmp_seq, x0, xd0);
steps = (1:numel(zmp))';

P = plot_helpers();
hf = P.newfig("03 legged locomotion — LIPM walking");
P.plot_points(hf, [steps, com(:,1)], "-bo-", "LineWidth", 2, "MarkerSize", 5);
P.plot_points(hf, [steps, capture], "r--x", "LineWidth", 1.5);
P.plot_points(hf, [steps, zmp], "ks-", "LineWidth", 1.5, "MarkerSize", 7);
xlabel("step"); ylabel("x (m)");
legend("CoM position", "capture point", "ZMP / footstep", "location", "best");

nstable = sum(stable);
fprintf("  steps=%d  stable=%d  final CoM x=%.3f  capture=%.3f\n", ...
    numel(zmp), nstable, com(end,1), capture(end));

P.set_limits(hf, 0.5, numel(zmp)+0.5, -0.4, 2.1);
out = fullfile(repo_root, "output", "figures", "03_legged_locomotion.png");
P.savefig(hf, out);
printf("03_legged_locomotion: wrote %s\n", out);
