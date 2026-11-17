% tests/test_batchA.m — Unit tests for Batch A domains (01 ground_vehicles,
% 02 manipulation, 03 legged_locomotion, 04 robot_modeling).
%
% Run: octave-cli --no-gui --quiet --eval "run('setup_paths.m'); run('tests/test_batchA.m')"
% Exit code 0 iff every check passes.

global failures;
failures = 0;

function check(cond, msg)
  global failures;
  if cond, printf("  PASS: %s\n", msg);
  else, printf("  FAIL: %s\n", msg); failures = failures + 1; end
endfunction

repo_root = fileparts(fileparts(mfilename("fullpath")));  % tests/ -> repo root
run(fullfile(repo_root, "setup_paths.m"));

printf("=== 01 ground_vehicles ===\n");
% pure pursuit returns finite, sensible outputs
x = [0; 0; 0]; path = [0 0; 3 0; 3 3];   % 3x2 waypoints
[delta, omega, p_ld, idx, alpha] = pure_pursuit_steer(x, path, 1.5, 0.8, 1.0);
check(all(isfinite([delta; omega; p_ld; alpha])), "pure_pursuit finite outputs");
check(idx >= 1 && idx <= size(path,1), "pure_pursuit valid waypoint idx");
check(abs(alpha) <= pi + 1e-9, "pure_pursuit alpha wrapped");
% unicycle: straight line at v=1, omega=0, dt=1 -> x=[1;0;0]
[x1, ~] = unicycle_step([0;0;0], 1.0, 0.0, 1.0);
check(norm(x1 - [1;0;0]) < 1e-6, "unicycle_step straight line x=[1;0;0]");
% unicycle: pure rotation v=0, omega=pi/2, dt=1 -> heading +pi/2, pos~0
[x2, ~] = unicycle_step([0;0;0], 0.0, pi/2, 1.0);
check(abs(x2(3) - pi/2) < 1e-6 && norm(x2(1:2)) < 1e-3, "unicycle_step pure rotation");

printf("=== 02 manipulation ===\n");
l1 = 1.0; l2 = 0.8;
p_target = [1.0; 0.4];
[q, reachable] = ik_2link(p_target, l1, l2);
check(reachable, "ik_2link reachable target");
for i = 1:2
  [p_ee, ~, ~] = fk_2link(q(:, i), l1, l2);
  check(norm(p_ee - p_target) < 1e-9, sprintf("ik->fk roundtrip elbow %d (err=%.2e)", i, norm(p_ee-p_target)));
end
% unreachable target
[qunr, runr] = ik_2link([5.0; 0], l1, l2);
check(~runr && all(isnan(qunr(:))), "ik_2link unreachable -> NaN, not reachable");
% jacobian shape
J = jacobian_2link([0.3; 0.4], l1, l2);
check(size(J) == [2 2], "jacobian_2link is 2x2");
check(all(isfinite(J(:))), "jacobian_2link finite");

printf("=== 03 legged_locomotion ===\n");
z = 0.80; g = 9.81; dt = 0.01; T = 0.5;
w = sqrt(g / z); x0 = 0.0; xd0 = 0.6; nsteps = 7;
zmp_seq = zeros(nsteps, 1); x = x0; xd = xd0;
for k = 1:nsteps
  zmp_seq(k) = x + xd / w;     % capture-point foot placement
  wh = w * T; ch = cosh(wh); sh = sinh(wh);
  x_new = zmp_seq(k) + (x - zmp_seq(k)) * ch + (xd / w) * sh;
  xd_new = w * (x - zmp_seq(k)) * sh + xd * ch;
  x = x_new; xd = xd_new;
end
[com, zmp_out, capture, stable] = lipm(z, g, dt, T, zmp_seq, x0, xd0);
check(all(stable), "lipm capture-stepped walking all stable");
check(norm(zmp_out - zmp_seq) < 1e-12, "lipm returns ZMP sequence");
% zmp_cop: with zero CoM accel, ZMP == CoM position
c = zmp_cop([1.0; 2.0], [0; 0], z, g);
check(norm(c - [1.0; 2.0]) < 1e-12, "zmp_cop zero accel -> c == x");

printf("=== 04 robot_modeling ===\n");
urdf = fullfile(repo_root, "assets", "urdf", "planar_3dof_arm.urdf");
robot = urdf_parse(urdf);
check(robot.num_links == 4, sprintf("urdf 4 links (got %d)", robot.num_links));
check(robot.num_joints == 3, sprintf("urdf 3 joints (got %d)", robot.num_joints));
check(strcmp(robot.joints(1).type, "revolute"), "joint1 revolute");
check(strcmp(robot.links(1).name, "base_link"), "root link base_link");
q = [0.5; 0.4; 0.3];
[T, ee] = urdf_fk(robot, q);
check(all(isfinite(ee(:))), "urdf_fk ee finite");
check(numel(T) == 4, "urdf_fk returns 4 link transforms");
% ee (link3 frame) sits at joint3 = sum of two unit links at q1, q1+q2
xexp = cos(q(1)) + cos(q(1) + q(2));
yexp = sin(q(1)) + sin(q(1) + q(2));
check(norm([ee(1); ee(2)] - [xexp; yexp]) < 1e-6, "urdf_fk ee matches planar FK");

printf("=== 01 ground_vehicles (new) ===\n");
% ackermann_step: straight drive forward, heading unchanged
[xn, info] = ackermann_step([0;0;0], 1.0, 0.0, 1.0, 1.0);
check(abs(xn(1)-1.0) < 1e-6 && abs(xn(2)) < 1e-6 && abs(xn(3)) < 1e-6, ...
    "ackermann_step straight drive x=[1;0;0]");
% ackermann_step: turning changes heading
[xn2, ~] = ackermann_step([0;0;0], 1.0, pi/4, 1.0, 1.0);
check(abs(xn2(3)) > 1e-3, "ackermann_step turning changes heading");
% omni wheel: inverse then forward recovers velocity
[Jom, Jompinv] = omni_wheel_kinematics("3wheel");
w = omni_wheel_kinematics("forward", Jom, 1.0, 0.0, 0.0, 0.05);
vel_rec = omni_wheel_kinematics("inverse", Jompinv, w(:));
check(norm(vel_rec - [1;0;0]) < 1e-6, "omni wheel forward/inverse roundtrip");
% warehouse fleet: runs and completes at least one task
[trajw, metw] = warehouse_fleet_sim(4, 10, 6, 80);
check(metw.tasks_completed >= 1, "warehouse_fleet completes a task");
check(metw.deadlocks == 0, "warehouse_fleet deadlocks == 0");
% swarm: runs, finite, bounded cohesion
[traj_swarm, mets] = robot_swarm_sim(12, 200, 0.05, "flock");
check(all(isfinite(traj_swarm(:))), "swarm trajectories finite");
check(mets.final_cohesion < 20, "swarm cohesion bounded");
% offroad nav: costmap has free space, path valid-or-empty
[pathn, costmap, metn] = offroad_ugv_nav(20, [1 1], [20 20], 3);
check(metn.blocked_fraction < 1, "offroad costmap has free space");
check(isempty(pathn) || size(pathn,1) >= 2, "offroad path valid or empty");

printf("=== 02 manipulation (new) ===\n");
dh3 = [0 0 1.0 0; 0 0 1.0 0; 0 0 0.6 0];
[Tbt, trs] = rigid_body_tree(dh3, [0.3; 0.2; 0.1]);
check(all(isfinite(Tbt(:))), "rigid_body_tree finite transform");
check(numel(trs) == 3, "rigid_body_tree returns 3 link transforms");
% IK: target = FK of a known reachable config (orientation consistent)
q_true = [0.3; 0.2; 0.1];
[Tdes, ~] = rigid_body_tree(dh3, q_true);
[Tik, success, nit] = ik_solver_ndof(dh3, zeros(3,1), Tdes, 200, 1e-3);
[Tchk, ~] = rigid_body_tree(dh3, Tik);
check(norm(Tchk(1:3,4) - Tdes(1:3,4)) < 5e-2, ...
    sprintf("ik_solver_ndof reaches target (err=%.2e)", norm(Tchk(1:3,4)-Tdes(1:3,4))));
[coll, pairs] = collision_check_manip(trs, struct("radius",0.05*ones(3,1)));
check(~coll, "collision_check_manip no self-collision (outstretched)");
[qt, qdt, qddt] = traj_gen_manip([0;0], [1;1], 1.0, 0.02);
check(norm(qt(1,:) - [0 0]) < 1e-9 && norm(qt(end,:) - [1 1]) < 1e-9, ...
    "traj_gen_manip endpoints match");
check(all(isfinite(qt(:))) && all(isfinite(qdt(:))), "traj_gen_manip finite");
tau = impedance_control([0.1;0.2], [0;0], [0.2;0.2], [0;0], eye(2), eye(2), eye(2), [0;0]);
check(all(isfinite(tau)), "impedance_control finite torque");
st = "IDLE";
evs = ["start","reached","grasped","lifted","transferred","lowered","released"];
for e = 1:length(evs)
  [st, ~, ~] = pick_and_place_fsm(evs(e), st);
end
check(strcmp(st, "IDLE"), "pick_and_place_fsm cycle returns to IDLE");
Tm = [cos(0.3) -sin(0.3) 0 0.5; sin(0.3) cos(0.3) 0 0.2; 0 0 1 0; 0 0 0 1];
dq = dual_quat_manip("from_transform", Tm);
Tm2 = dual_quat_manip("to_transform", dq);
check(norm(Tm - Tm2) < 1e-9, "dual_quat_manip transform roundtrip");
dq2 = dual_quat_manip("sclerp", dq, dq, 0.5);
Tm3 = dual_quat_manip("to_transform", dq2);
check(norm(Tm - Tm3) < 1e-6, "dual_quat_manip sclerp identity");

printf("=== 03 legged_locomotion (new) ===\n");
[fs2, cp2] = biped_footstep_planner(0.0, 0.4, 0.3, 4, 1.0);
check(all(isfinite(fs2)) && all(isfinite(cp2)), "biped_footstep_planner finite");
check(max(abs(diff([0; fs2]))) <= 0.3 + 1e-6, "biped footsteps respect step length");
[ct, fp] = quadruped_trot(0.5, 0.01, [0.1;0], 0.1);
check(size(ct) == [round(0.5/0.01)+1, 4], "quadruped_trot contact matrix shape");
check(all(ct(:,1) == ct(:,4)) && all(ct(:,2) == ct(:,3)), ...
    "quadruped_trot diagonal pairs pair up");
[act, info2] = legged_rl_policy([0.1;0;0.1;0;0;1;1;1;1]);
check(all(isfinite(act)), "legged_rl_policy finite action");

printf("=== 04 robot_modeling (new) ===\n");
p = struct("R",1.0,"L",0.001,"ke",0.01,"kt",0.01,"J",0.01,"b",0.001,"N",10);
[om_next, i_next, info] = actuator_dc_gearbox(0.0, 0.0, 12.0, 0.01, p);
check(info.output_speed > 0, "actuator_dc_gearbox spins up under voltage");
check(isfinite(info.output_torque), "actuator_dc_gearbox finite output torque");
[ptraj, eetraj, metc] = manipulator_conveyor_sim(2.0, 0.01, 0.1);
check(isfinite(metc.mean_tracking_error), "manipulator_conveyor finite error");

printf("=== summary ===\n");
if failures == 0
  printf("ALL Batch A domain checks PASS\n");
  exit(0);
else
  printf("%d Batch A domain check(s) FAILED\n", failures);
  exit(1);
endif
