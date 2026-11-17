% demos/05_perception.m — Perception: complementary-filter attitude estimation
% from gyro + accelerometer, compared against the truth.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/05_perception.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

dt = 0.01; T = 6.0;
t = (0:dt:T)'; N = numel(t);
% truth: smooth attitude manoeuvre
true_roll  = 0.25 * sin(t);
true_pitch = 0.15 * sin(1.5 * t);
true_yaw   = zeros(size(t));   % no yaw: body rates == Euler rates
truth = [true_roll, true_pitch, true_yaw];
% gyro = derivative of truth + noise (body rates ~= Euler rates, yaw = 0)
gyro = [0.25*cos(t), 0.225*cos(1.5*t), zeros(N,1)] + 0.02 * randn(N, 3);
% accel = gravity in body frame, sign convention matching complementary_filter:
%   roll_acc  = atan2(ay, az)  -> roll
%   pitch_acc = atan2(-ax, sqrt(ay^2+az^2)) -> pitch
g = 9.81;
accel = [-g*sin(true_pitch), g*sin(true_roll).*cos(true_pitch), ...
         g*cos(true_roll).*cos(true_pitch)] + 0.05 * randn(N, 3);

[ang, ~] = complementary_filter(gyro, accel, dt, 0.98, [0 0 0]);
err_roll = ang(:,1) - true_roll; err_pitch = ang(:,2) - true_pitch;

P = plot_helpers();
hf = P.newfig("05 perception — complementary filter attitude");
P.plot_points(hf, [t, true_roll], "-b-", "LineWidth", 1.5);
P.plot_points(hf, [t, ang(:,1)], "r--x", "LineWidth", 1);
xlabel("t (s)"); ylabel("roll (rad)");
legend("truth", "estimated", "location", "best");

fprintf("  roll  RMSE=%.4f rad   pitch RMSE=%.4f rad\n", ...
    sqrt(mean(err_roll.^2)), sqrt(mean(err_pitch.^2)));

% second sub-plot: pitch
hf2 = P.newfig("05 perception — pitch");
P.plot_points(hf2, [t, true_pitch], "-b-", "LineWidth", 1.5);
P.plot_points(hf2, [t, ang(:,2)], "r--x", "LineWidth", 1);
xlabel("t (s)"); ylabel("pitch (rad)");
legend("truth", "estimated", "location", "best");

out = fullfile(repo_root, "output", "figures", "05_perception.png");
P.savefig(hf, out);
printf("05_perception: wrote %s\n", out);
out2 = fullfile(repo_root, "output", "figures", "05_perception_pitch.png");
P.savefig(hf2, out2);
