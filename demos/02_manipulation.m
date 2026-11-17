% demos/02_manipulation.m — Manipulation: rigid-body-tree FK, numerical IK,
% collision checking, joint-space trajectory gen, and impedance control.
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/02_manipulation.m')"
% (self-contained: adds paths via setup_paths, writes output/figures/02_manipulation.png)

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

%% 3-DOF planar arm DH table [theta d a alpha] (all revolute)
dh = [ 0, 0, 1.0, 0;
       0, 0, 1.0, 0;
       0, 0, 0.6, 0];

%% Forward kinematics via rigid body tree
q0 = [0.4; 0.3; 0.2];
[T, transforms] = rigid_body_tree(dh, q0);
ee = T(1:3, 4);
printf("02_manipulation: FK ee = [%.3f %.3f %.3f]\n", ee(1), ee(2), ee(3));

%% Numerical IK to a target
p_des = [1.2; 0.5; 0];
T_des = [eye(3), p_des(:); 0 0 0 1];
[q_ik, success, n_iter] = ik_solver_ndof(dh, q0, T_des, 100, 1e-4);
[T_ik, ~] = rigid_body_tree(dh, q_ik);
ee_ik = T_ik(1:3, 4);
printf("02_manipulation: IK success=%d iter=%d ee_err=%.2e\n", ...
    success, n_iter, norm(ee_ik - p_des));

%% Collision check (self + one obstacle sphere near link 2)
link_obs = struct("radius", 0.08 * ones(3,1));
obstacles = [0.9, 0.2, 0.0, 0.10];   % [x y z r]
[coll, pairs] = collision_check_manip(transforms, link_obs, obstacles);
printf("02_manipulation: collision=%d pairs=%d\n", coll, rows(pairs));

%% Joint-space trajectory (via points) from q0 to q_ik
via = [0.7 0.5; 0.4 0.7; 0.3 0.2];   % 3 x 2 via points (3 joints x 2 via points)
via3 = via;                          % already 3 joints
[qt, qdt, qddt] = traj_gen_manip(q0, q_ik, 1.5, 0.02, via3);
printf("02_manipulation: trajectory samples=%d q_end=[%.3f %.3f %.3f]\n", ...
    size(qt,1), qt(end,1), qt(end,2), qt(end,3));

%% Impedance control at the final configuration
M = eye(3); B = 2*eye(3); Kp = 50*eye(3);
grav = [0; 0; 0];
tau = impedance_control(q_ik, qdt(end,:)', q_ik, zeros(3,1), M, B, Kp, grav);
printf("02_manipulation: final torque norm=%.3f\n", norm(tau));

%% Plot: EE path (initial -> target -> achieved) + joint trajectories
P = plot_helpers();
hf = P.newfig("02 manipulation — IK + trajectory + impedance");
% waypoints for plotting EE path (each row = one joint configuration)
waypts = [q0'; via3'; q_ik'];
ee_path = zeros(rows(waypts), 3);
for i = 1:rows(waypts)
  [Ti, ~] = rigid_body_tree(dh, waypts(i,:)');
  ee_path(i,:) = Ti(1:3,4)';
endfor
P.plot_trajectory(hf, ee_path(:,1), ee_path(:,2), "-b", "LineWidth", 1.5);
plot(ee_path(1,1), ee_path(1,2), "go", "MarkerFaceColor", "g", "MarkerSize", 6);
plot(ee_path(end,1), ee_path(end,2), "rx", "MarkerFaceColor", "r", "MarkerSize", 8);
plot(p_des(1), p_des(2), "m*", "MarkerSize", 12, "MarkerFaceColor", "m");
P.set_limits(hf, -0.5, 2.5, -1.0, 2.0);
out = fullfile(repo_root, "output", "figures", "02_manipulation.png");
P.savefig(hf, out);
printf("02_manipulation: wrote %s\n", out);
