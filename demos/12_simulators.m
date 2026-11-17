% demos/12_simulators.m — Simulators: fixed-step discrete simulation of a
% damped harmonic oscillator (sim_discrete core loop).
%
% Run: octave-cli --no-gui --quiet --eval "run('demos/12_simulators.m')"

repo_root = fileparts(fileparts(mfilename("fullpath")));
run(fullfile(repo_root, "setup_paths.m"));

% oscillator: x=[position; velocity], xdot = [vel; -k*pos - c*vel]
k = 2.0; c = 0.4; dt = 0.02; T = 8.0;
f = @(x, u) [x(1) + x(2)*dt; x(2) + (-k*x(1) - c*x(2))*dt];
[t, X, ~] = sim_discrete(f, [1.0; 0.0], T, dt);

P = plot_helpers();
hf = P.newfig("12 simulator — damped harmonic oscillator");
P.plot_trajectory(hf, t, X(:,1), "-b", "LineWidth", 1.5);
xlabel("t (s)"); ylabel("position (m)");
fprintf("  amplitude at t=4s = %.3f  at t=8s = %.3f (decaying)\n", ...
    abs(X(round(4/dt)+1,1)), abs(X(end,1)));

out = fullfile(repo_root, "output", "figures", "12_simulators.png");
P.savefig(hf, out);
printf("12_simulators: wrote %s\n", out);
