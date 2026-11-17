% tests/test_batchC.m — Unit tests for Batch C domains
% (09 UAV, 10 marine AUV, 11 automated driving) and tools
% (12 simulators, 13 ROS middleware, 14 hardware connectivity).
%
% Run: octave-cli --no-gui --quiet --eval "run('setup_paths.m'); run('tests/test_batchC.m')"
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

printf("=== 09 uav ===\n");
wrench = [4; 0; 0; 0];
[speed, thrust] = mixer(wrench);
check(size(thrust) == [4 1] && all(isfinite(thrust)), "mixer returns 4 finite thrusts");
check(norm(thrust - 1) < 1e-9, "mixer hover: each thrust = weight/4 = 1");
% small roll torque: right-side motors push more (thrust(4) > thrust(2))
[s2, t2] = mixer([4; 0.1; 0; 0]);
check(t2(4) > t2(2), "mixer roll: thrust(4) > thrust(2)");
% saturation: huge wrench clamps every motor at max_thrust
[s3, t3] = mixer([100; 0; 0; 0], [], 10.0);
check(all(t3 == 10.0), "mixer saturates at max_thrust");
check(all(s3 == sqrt(10)), "mixer speed = sqrt(thrust)");

printf("=== 10 marine_auv ===\n");
path = [0, 0; 10, 0];
[chi_d_on, e_ct_on] = los_guidance([5, 0], path, 1.0);
check(isfinite(chi_d_on) && isfinite(e_ct_on), "los_guidance finite outputs");
check(abs(e_ct_on) < 0.05, "los_guidance on-path cross-track error ~= 0");
check(abs(chi_d_on) < 0.05, "los_guidance on-path heading -> chi_d ~= 0");
[~, e_ct_off] = los_guidance([5, 0.5], path, 1.0);
check(abs(e_ct_off - 0.5) < 0.1, sprintf("los_guidance off-path cross-track (got %.3f)", e_ct_off));

printf("=== 11 automated_driving ===\n");
path = [0, 0; 10, 0];
[delta, cte, e_psi] = stanley(5, 0.2, 0, 5, path, 1.0, 0.5);
check(isfinite(delta), "stanley finite steering");
check(abs(cte + 0.2) < 0.05, sprintf("stanley cross-track error (got %.3f)", cte));
check(abs(delta) < 0.1 && sign(delta) == -1, "stanley steers toward path for +y error");
[delta0, cte0, e_psi0] = stanley(5, 0.0, 0, 5, path, 1.0, 0.5);
check(abs(delta0) < 0.05 && abs(cte0) < 0.05, "stanley zero error on straight path");

printf("=== 12 simulators ===\n");
k = 2.0; c = 0.4; dt = 0.02; T = 4.0;
f = @(x, u) [x(1) + x(2)*dt; x(2) + (-k*x(1) - c*x(2))*dt];
[t, X, U] = sim_discrete(f, [1.0; 0.0], T, dt);
n = round(T/dt);
check(size(X) == [n 2] && size(t) == [n 1] && all(isfinite(X(:))), ...
      "sim_discrete logs n finite states and time");
check(abs(X(end,1)) < abs(X(1,1)), "sim_discrete oscillator state decays");

printf("=== 13 ros_middleware ===\n");
msg = struct(); msg.linear_x = 0.5; msg.linear_y = -0.2; msg.angular_z = 1.0;
fields = {"linear_x", "linear_y", "angular_z"};
buf = ros_serialise(msg, fields);
msg2 = ros_deserialise(buf, fields);
check(numel(buf) == 32, "ros_serialise 32-byte frame");
check(isequal(msg.linear_x, msg2.linear_x) && ...
      isequal(msg.linear_y, msg2.linear_y) && ...
      isequal(msg.angular_z, msg2.angular_z), ...
      "ros round-trip field equality");

printf("=== 14 hardware_connectivity ===\n");
[pkt, len] = serial_packet(0x01, uint8([10, 20, 30, 40]));
[cmd, payload, ok] = serial_parse(pkt);
check(ok && cmd == 1 && isequal(payload, uint8([10; 20; 30; 40])), ...
      "serial packet build + parse roundtrip");
pkt_bad = pkt; pkt_bad(6) = bitxor(pkt_bad(6), 0xFF);
[cmd_b, payload_b, ok_b] = serial_parse(pkt_bad);
check(~ok_b, "serial parse rejects corrupted packet");

printf("=== summary ===\n");
if failures == 0
  printf("ALL Batch C domain/tool checks PASS\n");
  exit(0);
else
  printf("%d Batch C domain/tool check(s) FAILED\n", failures);
  exit(1);
endif
