% demos/04_robot_modeling.m — Robot modeling: DC-motor+gearbox step response,
% manipulator-on-conveyor pick, and URDF forward kinematics.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/04_robot_modeling.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

%% DC motor + gearbox step response
params = struct("R", 1.0, "L", 0.001, "ke", 0.01, "kt", 0.01, ...
                "J", 0.01, "b", 0.001, "N", 10, "tau_load", 0.0);
[t, omega, i, out] = actuator_dc_step_response(12.0, 0.5, 0.001, params);
printf("04_robot_modeling: motor speed final=%.3f rad/s  output=%.3f rad/s  i_final=%.3f A\n", ...
    omega(end), out(end), i(end));

%% Manipulator on conveyor belt pick
[part_traj, ee_traj, metrics] = manipulator_conveyor_sim(4.0, 0.01, 0.15);
printf("04_robot_modeling: conveyor pick=%d pick_time=%.2f s  mean_err=%.3f m\n", ...
    metrics.picked, metrics.pick_time, metrics.mean_tracking_error);

%% URDF forward kinematics (planar 3DOF arm)
urdf = fullfile(repo_root, "assets", "urdf", "planar_3dof_arm.urdf");
robot = urdf_parse(urdf);
q = [0.5; 0.4; 0.3];
[T, ee] = urdf_fk(robot, q);
printf("04_robot_modeling: URDF links=%d joints=%d ee=[%.3f %.3f %.3f]\n", ...
    robot.num_links, robot.num_joints, ee(1), ee(2), ee(3));

%% Plot: motor speed + EE tracking error + arm configuration
P = plot_helpers();
hf = P.newfig("04 robot modeling — actuator + conveyor + URDF FK");
% top: motor speed (output shaft)
subplot(2,2,1);
plot(t, out, "-b", "LineWidth", 1.5); grid on;
xlabel("t (s)"); ylabel("output speed (rad/s)");
title("DC motor + gearbox step response");
% middle: conveyor tracking error
subplot(2,2,2);
ee_err = sqrt(sum((part_traj(:,1:2) - ee_traj).^2, 2));
plot(part_traj(:,1), ee_err, "-r", "LineWidth", 1.5); grid on;
xlabel("part x (m)"); ylabel("EE error (m)");
title("Conveyor pick tracking error");
% bottom: arm configuration (link endpoints via FK chain)
subplot(2,2,3);
c1 = cos(q(1)); s1 = sin(q(1));
c12 = cos(q(1)+q(2)); s12 = sin(q(1)+q(2));
c123 = cos(q(1)+q(2)+q(3)); s123 = sin(q(1)+q(2)+q(3));
link_pts = [0, 0;
            c1, s1;
            c1+c12, s1+s12;
            c1+c12+c123, s1+s12+s123];
plot(link_pts(:,1), link_pts(:,2), "-bo-", "LineWidth", 2, "MarkerSize", 6);
axis equal; grid on;
xlabel("x (m)"); ylabel("y (m)");
title("Planar 3DOF arm configuration");
P.set_limits(hf, -0.5, 3.5, -0.5, 3.5);
out = fullfile(repo_root, "output", "figures", "04_robot_modeling.png");
P.savefig(hf, out);
printf("04_robot_modeling: wrote %s\n", out);
