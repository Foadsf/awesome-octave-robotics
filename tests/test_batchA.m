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

printf("=== summary ===\n");
if failures == 0
  printf("ALL Batch A domain checks PASS\n");
  exit(0);
else
  printf("%d Batch A domain check(s) FAILED\n", failures);
  exit(1);
endif
