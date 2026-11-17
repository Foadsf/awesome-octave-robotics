% robot_swarm_sim.m — Reynolds flocking + Olfati-Saber consensus formation.
%
%   [trajectories, metrics] = robot_swarm_sim(n, T, dt, mode)
%
%   Simulates n holonomic agents with single-integrator dynamics
%       p_dot = u
%   under two modes:
%     "flock"   : Reynolds flocking (separation + alignment + cohesion)
%     "form"    : consensus formation keeping around a moving reference
%   Returns trajectories (T+1) x (2n) flattened [x1 y1 x2 y2 ...] and metrics.
%
%   Pure Octave native code.

function [trajectories, metrics] = robot_swarm_sim(n, T, dt, mode)
  if nargin < 4, mode = "flock"; end
  if nargin < 3, dt = 0.05; end
  if nargin < 2, T = 400; end
  if nargin < 1, n = 20; end

  rng(7);
  p = zeros(n, 2);
  for i = 1:n
    ang = rand*2*pi; rad = sqrt(rand)*5;
    p(i,:) = [rad*cos(ang), rad*sin(ang)];
  endfor
  v = zeros(n, 2);

  ref_cx = 0; ref_cy = 0; ref_r = 6; ref_omega = 0.02;
  offsets = zeros(n, 2);
  for i = 1:n
    th = (i-1)*2*pi/n;
    offsets(i,:) = [ref_r*cos(th), ref_r*sin(th)];
  endfor

  k_sep = 1.5; k_ali = 0.6; k_coh = 0.5;
  sep_dist = 1.5;
  nbr_dist = 4.0;
  v_max = 2.0;

  trajectories = zeros(T+1, 2*n);
  trajectories(1,:) = reshape(p, 1, []);

  metrics.collisions = 0;
  metrics.formation_error = 0;
  metrics.cohesion = 0;
  min_dist = inf;

  for t = 1:T
    if strcmp(mode, "form")
      ref = [ref_cx + ref_r*cos(ref_omega*t),
             ref_cy + ref_r*sin(ref_omega*t)];
    endif

    u = zeros(n, 2);
    for i = 1:n
      pi_ = p(i,:); vi = v(i,:);
      sep = [0 0]; ali = [0 0]; coh = [0 0];
      nsep = 0; nnbr = 0;
      for j = 1:n
        if j == i, continue; end
        d = p(j,:) - pi_;
        dist = norm(d);
        if dist < sep_dist
          sep = sep - d / max(dist, 1e-6);
          nsep = nsep + 1;
        endif
        if dist < nbr_dist
          ali = ali + v(j,:);
          coh = coh + p(j,:);
          nnbr = nnbr + 1;
        endif
      endfor
      if nsep > 0, sep = sep / nsep; end
      if nnbr > 0
        ali = ali / nnbr - vi;
        coh = coh / nnbr - pi_;
      endif

      if strcmp(mode, "flock")
        u(i,:) = k_sep*sep + k_ali*ali + k_coh*coh;
      else
        form_err = (ref + offsets(i,:)) - pi_;
        u(i,:) = k_coh*form_err - k_ali*vi;
        if nsep > 0, u(i,:) = u(i,:) + k_sep*sep; end
      endif

      nu = norm(u(i,:));
      if nu > v_max, u(i,:) = u(i,:) * (v_max / nu); end
    endfor

    v = v + dt * u;
    for i = 1:n
      nv = norm(v(i,:));
      if nv > v_max, v(i,:) = v(i,:) * (v_max / nv); end
    endfor
    p = p + dt * v;

    for i = 1:n
      for j = i+1:n
        d = norm(p(i,:) - p(j,:));
        if d < min_dist, min_dist = d; end
        if d < 0.5, metrics.collisions = metrics.collisions + 1; end
      endfor
    endfor
    if strcmp(mode, "form")
      ferr = 0;
      for i = 1:n
        ferr = ferr + norm((ref + offsets(i,:)) - p(i,:));
      endfor
      metrics.formation_error = metrics.formation_error + ferr / n;
    endif
    metrics.cohesion = metrics.cohesion + mean(vecnorm(p - mean(p,1)));
    trajectories(t+1,:) = reshape(p, 1, []);
  endfor

  metrics.min_distance = min_dist;
  metrics.final_cohesion = mean(vecnorm(p - mean(p,1)));
endfunction
