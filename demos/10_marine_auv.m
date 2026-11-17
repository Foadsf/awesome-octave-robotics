% demos/10_marine_auv.m — Marine AUV: LOS guidance track following.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/10_marine_auv.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

% desired track: two straight segments (a dogleg)
path = [0, 0; 4, 0; 4, 3; 7, 3];
Delta = 0.8;   % look-ahead distance (m)
v = 0.6; dt = 0.05; T = 25;
n = round(T/dt);
pos = [0.2; -0.4]; chi = 0.1;   % start off-track with heading error
X = zeros(n, 2); ECT = zeros(n, 1);
for k = 1:n
  [chi_d, e_ct] = los_guidance(pos, path, Delta);
  % turn rate proportional to wrapped course error (simple heading autopilot)
  werr = chi_d - chi; werr = rem(werr + pi, 2*pi) - pi;
  chi_dot = 1.2 * werr;
  chi = rem(chi + chi_dot * dt + pi, 2*pi) - pi;
  pos = pos + v * [cos(chi); sin(chi)] * dt;
  X(k, :) = pos'; ECT(k) = e_ct;
end

P = plot_helpers();
hf = P.newfig("10 marine AUV — LOS track following");
P.plot_points(hf, path, "-k", "LineWidth", 2);
P.plot_trajectory(hf, X(:,1), X(:,2), "-b", "LineWidth", 1.5);
plot(X(1,1), X(1,2), "go", "MarkerFaceColor", "g", "MarkerSize", 6);
plot(X(end,1), X(end,2), "rx", "MarkerSize", 8);
xlabel("x (m)"); ylabel("y (m)");
legend("desired track", "vessel track", "start", "end", "location", "best");

hf2 = P.newfig("10 marine AUV — cross-track error");
P.plot_points(hf2, [(0:n-1)'*dt, ECT], "-b-", "LineWidth", 1.5);
xlabel("t (s)"); ylabel("cross-track error (m)");

fprintf("  final cross-track error = %.3f m  max = %.3f m\n", ...
    abs(ECT(end)), max(abs(ECT)));

out = fullfile(repo_root, "output", "figures", "10_marine_auv.png");
P.savefig(hf, out);
out2 = fullfile(repo_root, "output", "figures", "10_marine_auv_ect.png");
P.savefig(hf2, out2);
printf("10_marine_auv: wrote %s\n", out);
