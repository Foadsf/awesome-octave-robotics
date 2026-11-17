% demos/04_robot_modeling.m — Robot modeling: parse a URDF (planar 3-DOF arm)
% and compute forward kinematics from joint positions.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/04_robot_modeling.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

urdf = fullfile(repo_root, "assets", "urdf", "planar_3dof_arm.urdf");
robot = urdf_parse(urdf);
q = [0.7; 0.5; 0.3];   % joint angles (rad)

[T, ee] = urdf_fk(robot, q);

% link frame origins (xy) and headings
n = robot.num_links;
orig = zeros(n, 2);

P = plot_helpers();
hf = P.newfig("04 robot modeling — URDF forward kinematics");
P.plot_trajectory(hf, orig(:,1), orig(:,2), "-bo-", "LineWidth", 2, "MarkerSize", 6);
P.plot_points(hf, ee(1:2)', "r*", "MarkerSize", 14, "MarkerFaceColor", "r");
for i = 1:n
  th = atan2(T{i}(2,1), T{i}(1,1));
  T2 = [cos(th), -sin(th), orig(i,1); sin(th), cos(th), orig(i,2); 0 0 1];
  P.plot_frame2d(hf, T2, 0.45);
end
fprintf("  links=%d joints=%d  ee=[%.3f %.3f %.3f]  q=[%.2f %.2f %.2f]\n", ...
    robot.num_links, robot.num_joints, ee(1), ee(2), ee(3), q(1), q(2), q(3));

P.set_limits(hf, -1, 3.4, -1, 1.4);
out = fullfile(repo_root, "output", "figures", "04_robot_modeling.png");
P.savefig(hf, out);
printf("04_robot_modeling: wrote %s\n", out);
