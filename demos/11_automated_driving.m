% demos/11_automated_driving.m — Automated driving: Stanley lane-keeping on a
% curved reference path.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/11_automated_driving.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

% curved reference lane: y = 0.6 sin(x/3)
xs = linspace(0, 40, 400)';
path = [xs, 0.6*sin(xs/3)];
v = 2.5; dt = 0.03; T = 12; L = 2.8;   % speed, step, time, wheelbase
n = round(T/dt);
% start offset from the lane
x = 0.5; y = 0.6*sin(0.5/3) + 0.8; yaw = 0.1;
X = zeros(n, 2); CT = zeros(n, 1); DE = zeros(n, 1);
for k = 1:n
  [delta, cte, e_psi] = stanley(x, y, yaw, v, path, 1.2, 0.5);
  % bicycle model: xdot=v cos(yaw), ydot=v sin(yaw), yawdot = v/L tan(delta)
  x = x + v*cos(yaw)*dt;
  y = y + v*sin(yaw)*dt;
  yaw = rem(yaw + (v/L)*tan(delta)*dt + pi, 2*pi) - pi;
  X(k, :) = [x, y]; CT(k) = cte; DE(k) = delta;
end

P = plot_helpers();
hf = P.newfig("11 automated driving — Stanley lane keeping");
P.plot_points(hf, path, "-k", "LineWidth", 1.5);
P.plot_trajectory(hf, X(:,1), X(:,2), "-b", "LineWidth", 1.5);
plot(X(1,1), X(1,2), "go", "MarkerFaceColor", "g", "MarkerSize", 6);
plot(X(end,1), X(end,2), "rx", "MarkerSize", 8);
xlabel("x (m)"); ylabel("y (m) — lane centreline");
legend("reference lane", "vehicle track", "start", "end", "location", "best");

hf2 = P.newfig("11 automated driving — cross-track error");
P.plot_points(hf2, [(0:n-1)'*dt, CT], "-b-", "LineWidth", 1.5);
xlabel("t (s)"); ylabel("cross-track error (m)");
fprintf("  final cte = %.3f m  max |cte| = %.3f m\n", CT(end), max(abs(CT)));

out = fullfile(repo_root, "output", "figures", "11_automated_driving.png");
P.savefig(hf, out);
out2 = fullfile(repo_root, "output", "figures", "11_automated_driving_cte.png");
P.savefig(hf2, out2);
printf("11_automated_driving: wrote %s\n", out);
