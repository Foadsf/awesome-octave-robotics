% test_core.m — Unit tests for src/core/ (SE(2)/SE(3)/SO(3), quats, dual quats,
% RK4, Riccati LQR, QP, headless PNG export).
%
% Run: octave-cli --no-gui --quiet --eval "run('setup_paths.m'); run('tests/test_core.m')"
% Exit code 0 iff every check passes.

global failures;
failures = 0;

function check(cond, msg)
  global failures;
  if cond, printf("  PASS: %s\n", msg);
  else, printf("  FAIL: %s\n", msg); failures = failures + 1; end
endfunction

printf("=== Core spatial algebra ===\n");
S = so3();
% SO(3) exp/log roundtrip
for w = {[0.3;0.2;0.1]; [2.5;0.1;-0.7]; [0;0;3.0]}
  R = S.exp(w{1}); wr = S.log(R);
  check(norm(wr - w{1}) < 1e-9, "so3 exp/log roundtrip");
end
% SO(3) composition / inverse: R * R' = I
R1 = S.exp([0.2;0.1;0.3]); R2 = S.exp([-0.1;0.4;0.2]);
Ierr = norm(S.compose(R1, S.inv(R1)) - eye(3));
check(Ierr < 1e-12, sprintf("so3 inverse R*R'=I (err=%.2e)", Ierr));
% SO(3) associativity: (R1 R2) R3 = R1 (R2 R3)
R3 = S.exp([0.3;-0.2;0.1]);
assoc = norm(S.compose(S.compose(R1,R2),R3) - S.compose(R1,S.compose(R2,R3)));
check(assoc < 1e-12, sprintf("so3 associativity (err=%.2e)", assoc));
check(S.isRotation(eye(3)) && S.isRotation(R1), "so3 isRotation");
check(~S.isRotation(zeros(3)), "so3 isRotation rejects zeros");

printf("=== SE(2) / SE(3) ===\n");
G2 = se2();
T = G2.tform(1.0, 2.0, 0.5);
Ti = G2.inv(T);
check(norm(G2.compose(T,Ti) - eye(3)) < 1e-12, "se2 inverse");
p = [3.0; -1.0];
p2 = G2.transform(T, p);
check(norm(G2.transform(Ti, p2) - p) < 1e-9, "se2 transform roundtrip");

G3 = se3();
xi = [0.1;0.2;0.3; 0.4;0.1;0.2];
T = G3.exp(xi); xir = G3.log(T);
check(norm(xir - xi) < 1e-9, "se3 exp/log roundtrip");
Ti = G3.inv(T);
check(norm(G3.compose(T,Ti) - eye(4)) < 1e-12, "se3 inverse");
p3 = [1;2;3];
p3t = G3.transform(T, p3);
check(norm(G3.transform(Ti, p3t) - p3) < 1e-9, "se3 transform roundtrip");
Ad = G3.adjoint(T);
check(size(Ad) == [6 6], "se3 adjoint is 6x6");

printf("=== Quaternions ===\n");
Q = quat_ops();
q1 = Q.from_axis_angle([1;0;0], 0.5);
q2 = Q.from_axis_angle([0;1;0], 0.3);
R1q = Q.rotmat(q1); R2q = Q.rotmat(q2);
check(norm(Q.rotmat(Q.mul(q1,q2)) - R1q*R2q) < 1e-10, "quat mul == rotmat composition");
check(norm(Q.slerp(q1,q2,0) - q1) < 1e-9, "slerp t=0");
check(norm(Q.slerp(q1,q2,1) - q2) < 1e-9, "slerp t=1");
check(norm(Q.mul(q1, Q.conj(q1)) - Q.identity()) < 1e-12, "quat q*conj(q)=1");

printf("=== Dual quaternions ===\n");
D = dual_quat();
Tref = G3.tform(eye(3), [0.4; -0.2; 0.7]);
dq = D.from_se3(Tref);
Tback = D.to_se3(dq);
check(norm(Tback(1:3,4) - [0.4;-0.2;0.7]) < 1e-9, "dual_quat from_se3/to_se3 translation");
check(norm(Tback(1:3,1:3) - eye(3)) < 1e-9, "dual_quat from_se3/to_se3 rotation");
dq_id = D.identity();
T0 = D.to_se3(D.sclerp(dq_id, dq, 0.0));
check(norm(T0(1:3,4)) < 1e-9, "dual_quat sclerp t=0 translation");
T1 = D.to_se3(D.sclerp(dq_id, dq, 1.0));
check(norm(T1(1:3,4) - [0.4;-0.2;0.7]) < 1e-9, "dual_quat sclerp t=1 translation");

printf("=== RK4 ===\n");
f = @(t,y) [y(2); -y(1)];
[t, y] = ode_rk4(f, [0, 20], [1; 0], 4000);
E = sum(y.^2, 2);
check(max(abs(E - 1)) < 1e-6, sprintf("RK4 energy conservation (max err=%.2e)", max(abs(E-1))));
check(abs(y(end,1) - cos(20)) < 1e-3, sprintf("RK4 final x ~ cos(20)=%.3f got %.3f", cos(20), y(end,1)));
[t2, y2] = semi_implicit_euler(f, [0, 20], [1; 0], 4000);
E2 = sum(y2.^2, 2);
check(max(abs(E2 - 1)) < 5e-2, sprintf("symplectic Euler bounded energy (max err=%.2e)", max(abs(E2-1))));

printf("=== Riccati LQR ===\n");
Adm = [1.1 0.1; 0 0.9]; Bdm = [0; 1]; Qdm = eye(2); Rdm = 1;
[Pd, Kd, infod] = dare_lqr(Adm, Bdm, Qdm, Rdm);
cldeig = eig(Adm - Bdm*Kd);
check(all(abs(cldeig) < 1), sprintf("DARE closed-loop Schur stable (max|eig|=%.3f)", max(abs(cldeig))));
check(infod.converged, "DARE converged");
Ac = [0 1; -1 -0.5]; Bc = [0; 1]; Qc = eye(2); Rc = 1;
[Pc, Kc, infoc] = care_lqr(Ac, Bc, Qc, Rc);
cleig = eig(Ac - Bc*Kc);
check(all(real(cleig) < 0), sprintf("CARE closed-loop Hurwitz stable (max Re=%.3f)", max(real(cleig))));
check(infoc.converged, "CARE converged");

printf("=== QP ===\n");
H = 2*eye(2); f = [-6; 2]; lb=[0;-2]; ub=[2;0];
[x, obj, info] = qp_solve(H, f, [], [], [], [], lb, ub);
check(info.status == "optimal", "QP box optimal");
check(norm(x - [2;-1]) < 1e-3, sprintf("QP box solution [%.4f %.4f] ~ [2 -1]", x(1), x(2)));
[x2,~,info2] = qp_solve(2*eye(2), [0;0], [-1 -1], [-1]);
check(norm(x2 - [0.5;0.5]) < 1e-3, sprintf("QP ineq [%.4f %.4f] ~ [0.5 0.5]", x2(1), x2(2)));

printf("=== Headless PNG export ===\n");
P = plot_helpers();
hf = P.newfig("core test");
P.plot_frame2d(hf, G2.tform(0,0,0), 0.5);
P.plot_trajectory(hf, [0 1 2], [0 1 0]);
this_file = mfilename("fullpath");
this_dir  = fileparts(this_file);          % .../tests
repo_root = fileparts(this_dir);           % .../awesome-octave-robotics
filepath  = fullfile(repo_root, "output", "figures", "test_core_smoke.png");
P.savefig(hf, filepath);
check(exist(filepath,'file') ~= 0, sprintf("PNG written: %s", filepath));

printf("=== Result ===\n");
if failures == 0
  printf("ALL CORE TESTS PASSED\n");
  exit(0);
else
  printf("%d CORE TEST(S) FAILED\n", failures);
  exit(1);
end
