% scripts/generate_category_icons.m — Clean-room 200×200 schematic category icons.
%
% Generates 14 original, unambiguous FLOSS PNG badges for the robotics category
% taxonomy, replacing any third-party/icons originating under restrictive licenses.
% Every icon is drawn natively in GNU Octave from geometric primitives — no
% external assets, no proprietary toolboxes.
%
% Run: octave-cli --no-gui --quiet --eval "run('scripts/generate_category_icons.m');"

function generate_category_icons()
  graphics_toolkit("gnuplot");  % headless-safe, file-based toolkit (matches repo demos)
  % Clean-room colour palette (BSD-3-Clause original).
  C.dark   = [0.12 0.23 0.37];   % deep navy — primary lines
C.accent = [0.20 0.55 0.85];   % clear blue — active elements
C.warn   = [0.88 0.45 0.14];   % orange — highlights / axes X
C.ok     = [0.22 0.66 0.30];   % green — axes Y / success
C.blueZ  = [0.10 0.45 0.75];   % blue — axes Z
C.grey   = [0.62 0.66 0.72];   % muted — structure / grid

repo_root = fileparts(fileparts(mfilename("fullpath")));  % scripts/ -> repo root
imgdir    = fullfile(repo_root, "images");

% Canonical 14 categories (order matches README coverage table).
categories = { "ground", "manip", "legged", "model", "percep", "slam", "path", "control", "uav", "marine", "auto", "sim", "ros", "connect" };

% Figure canvas (square). Printed at -r50 from a 400×400 px figure ≈ 200×200 px.
canvas_w = 400; canvas_h = 400;
dpi      = 50;   % 400 px canvas ≈ target 200×200 output

for i = 1:numel(categories)
  name = categories{i};
  fprintf("  drawing %s.png ... ", name);
  ax = new_canvas(canvas_w, canvas_h);
  fig = get(ax, "Parent");
  draw_icon(ax, name, C);
  print(fig, fullfile(imgdir, sprintf("%s.png", name)), "-dpng", sprintf("-r%d", dpi));
  close(fig);
  fprintf("done\n");
endfor

fprintf("All %d category icons written to %s\n", numel(categories), imgdir);
endfunction

% ---------------------------------------------------------------------------
% Figure scaffold: invisible headless figure, white background, unit canvas.
% Returns the axes (fig is ax.Parent).
% ---------------------------------------------------------------------------
function ax = new_canvas(w, h)
  fig = figure("visible", "off");
  try; graphics_toolkit(fig, "gnuplot"); catch; end
  set(fig, 'Position', [100 100 w h]);
  ax = axes('Parent', fig, 'Position', [0 0 1 1], 'Visible', 'off', ...
       'XLim', [0 1], 'YLim', [0 1], 'DataAspectRatio', [1 1 1], ...
       'Box', 'off', 'CLipping', 'off');
endfunction

% ---------------------------------------------------------------------------
% Icon dispatch
% ---------------------------------------------------------------------------
function draw_icon(ax, name, C)
  switch name
    case "ground",     draw_ground(ax, C);
    case "manip",      draw_manip(ax, C);
    case "legged",     draw_legged(ax, C);
    case "model",      draw_model(ax, C);
    case "percep",     draw_percep(ax, C);
    case "slam",       draw_slam(ax, C);
    case "path",       draw_path(ax, C);
    case "control",    draw_control(ax, C);
    case "uav",        draw_uav(ax, C);
    case "marine",     draw_marine(ax, C);
    case "auto",       draw_auto(ax, C);
    case "sim",        draw_sim(ax, C);
    case "ros",        draw_ros(ax, C);
    case "connect",    draw_connect(ax, C);
  endswitch
endfunction

% ---------------------------------------------------------------------------
% 01 ground — differential-drive mobile base: chassis + 2 wheels + axle
% ---------------------------------------------------------------------------
function draw_ground(ax, C)
  c = [0.5 0.5]; rw = 0.17; rw2 = rw*0.95;
  rectangle('Parent', ax, 'Position', [0.22 0.34 0.56 0.18], ...
       'FaceColor', C.dark, 'EdgeColor', C.dark, 'LineWidth', 3);
  line(ax, [0.22 0.78], [0.5 0.5], 'Color', C.grey, 'LineWidth', 2);
  wheel(ax, [0.30 0.5], rw, C.dark);
  wheel(ax, [0.70 0.5], rw, C.dark);
  wheel(ax, [0.30 0.5], rw2, [1 1 1]);
  wheel(ax, [0.70 0.5], rw2, [1 1 1]);
  line(ax, [0.5 0.5], [0.26 0.20], 'Color', C.accent, 'LineWidth', 2);
endfunction

% 02 manip — 3-link planar arm from base, end-effector dot
function draw_manip(ax, C)
  base = [0.22 0.28];
  l1 = [0.42 0.055]; l2 = [0.42 0.055]; l3 = [0.30 0.045];
  p1 = base + l1; p2 = p1 + l2; p3 = p2 + l3;
  line(ax, [base(1) p1(1)], [base(2) p1(2)], 'Color', C.dark, 'LineWidth', 6);
  line(ax, [p1(1) p2(1)], [p1(2) p2(2)], 'Color', C.dark, 'LineWidth', 5);
  line(ax, [p2(1) p3(1)], [p2(2) p3(2)], 'Color', C.accent, 'LineWidth', 4);
  joint(ax, base, C.dark); joint(ax, p1, C.dark); joint(ax, p2, C.accent);
  plot(ax, p3(1), p3(2), '.', 'Color', C.ok, 'MarkerSize', 14, 'MarkerFaceColor', C.ok);
  rectangle('Parent', ax, 'Position', [base(1)-0.03 base(2)-0.025 0.06 0.045], ...
       'FaceColor', C.dark, 'EdgeColor', C.dark);
endfunction

% 03 legged — biped stick figure: head, torso, 2 legs, 2 arms
function draw_legged(ax, C)
  hip = [0.5 0.62]; head = [0.5 0.84];
  line(ax, [0.5 0.5], [0.62 0.74], 'Color', C.dark, 'LineWidth', 5);
  circle(ax, head, 0.055, C.dark, [1 1 1]);
  line(ax, [0.5 0.36], [0.62 0.30], 'Color', C.dark, 'LineWidth', 5);
  line(ax, [0.5 0.64], [0.62 0.30], 'Color', C.dark, 'LineWidth', 5);
  line(ax, [0.33 0.39], [0.30 0.30], 'Color', C.accent, 'LineWidth', 4);
  line(ax, [0.61 0.67], [0.30 0.30], 'Color', C.accent, 'LineWidth', 4);
  line(ax, [0.5 0.34], [0.74 0.70], 'Color', C.dark, 'LineWidth', 4);
  line(ax, [0.5 0.66], [0.74 0.70], 'Color', C.dark, 'LineWidth', 4);
endfunction

% 04 model — URDF coordinate frame axes (X red, Y green, Z blue) from origin
function draw_model(ax, C)
  O = [0.5 0.5]; L = 0.30;
  arrow(ax, O, O + [L 0], C.warn, 5);
  text(ax, O(1)+L+0.02, O(2), "X", 'Color', C.warn, 'FontSize', 20, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
  arrow(ax, O, O + [0 L], C.ok, 5);
  text(ax, O(1), O(2)+L+0.02, "Y", 'Color', C.ok, 'FontSize', 20, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
  arrow(ax, O, O + [L*0.42 L*0.42], C.blueZ, 5);
  text(ax, O(1)+L*0.42+0.02, O(2)+L*0.42+0.02, "Z", 'Color', C.blueZ, 'FontSize', 20, 'FontWeight', 'bold');
  rectangle('Parent', ax, 'Position', [0.30 0.30 0.40 0.40], 'FaceColor', 'none', ...
       'EdgeColor', C.grey, 'LineWidth', 2);
endfunction

% 05 percep — camera frustum + point cloud scatter
function draw_percep(ax, C)
  cam = [0.5 0.20];
  arrow(ax, cam, [0.32 0.68], C.dark, 3);
  arrow(ax, cam, [0.68 0.68], C.dark, 3);
  line(ax, [0.32 0.68], [0.68 0.68], 'Color', C.dark, 'LineWidth', 2);
  rng("default");
  pts = 0.25 + 0.45*rand(28, 2);
  plot(ax, pts(:,1), pts(:,2), '.', 'Color', C.accent, 'MarkerSize', 5, 'MarkerFaceColor', C.accent);
  plot(ax, cam(1), cam(2), 'o', 'Color', C.dark, 'MarkerSize', 7, 'MarkerFaceColor', C.dark);
endfunction

% 06 slam — occupancy grid cells + connected pose graph
function draw_slam(ax, C)
  occ = [0.22 0.22; 0.34 0.22; 0.22 0.34; 0.58 0.22; 0.70 0.34; 0.46 0.58];
  for k = 1:size(occ,1)
    rectangle('Parent', ax, 'Position', [occ(k,:) 0.10 0.10], 'FaceColor', C.dark, 'EdgeColor', C.dark);
  endfor
  free = [0.46 0.22; 0.58 0.34; 0.34 0.46];
  for k = 1:size(free,1)
    rectangle('Parent', ax, 'Position', [free(k,:) 0.10 0.10], 'FaceColor', [0.92 0.94 0.96], 'EdgeColor', C.grey, 'LineWidth', 1);
  endfor
  poses = [0.34 0.34; 0.50 0.30; 0.62 0.46];
  for k = 1:size(poses,1)
    circle(ax, poses(k,:), 0.022, C.ok, C.ok);
  endfor
  line(ax, [poses(1,1) poses(2,1)], [poses(1,2) poses(2,2)], 'Color', C.ok, 'LineWidth', 2, 'LineStyle', '--');
  line(ax, [poses(2,1) poses(3,1)], [poses(2,2) poses(3,2)], 'Color', C.ok, 'LineWidth', 2, 'LineStyle', '--');
endfunction

% 07 path — RRT/A* branching tree from start to goal
function draw_path(ax, C)
  start = [0.18 0.5]; goal = [0.82 0.5];
  branches = {
    [0.18 0.50; 0.34 0.58; 0.48 0.52],
    [0.18 0.50; 0.30 0.42; 0.44 0.46],
    [0.18 0.50; 0.34 0.58; 0.52 0.66; 0.66 0.60]
  };
  for b = 1:numel(branches)
    pts = branches{b};
    line(ax, pts(:,1), pts(:,2), 'Color', C.grey, 'LineWidth', 2);
    for k = 1:size(pts,1)
      plot(ax, pts(k,1), pts(k,2), '.', 'Color', C.grey, 'MarkerSize', 3);
    endfor
  endfor
  line(ax, [0.18 0.44 0.66 0.82], [0.50 0.46 0.50 0.50], 'Color', C.accent, 'LineWidth', 3);
  circle(ax, start, 0.028, C.ok, C.ok);
  circle(ax, goal, 0.028, C.warn, C.warn);
  text(ax, start(1)-0.03, start(2), "S", 'Color', C.ok, 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'right');
  text(ax, goal(1)+0.03, goal(2), "G", 'Color', C.warn, 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'left');
endfunction

% 08 control — step response with MPC horizon steps
function draw_control(ax, C)
  t = linspace(0, 1, 60);
  y = 1 - exp(-5*t);
  line(ax, 0.22 + 0.52*t, 0.28 + 0.50*y, 'Color', C.accent, 'LineWidth', 3);
  line(ax, [0.22 0.74], [0.78 0.78], 'Color', C.dark, 'LineWidth', 2, 'LineStyle', ':');
  hx = [0.30 0.40 0.50 0.60]; hy = [0.42 0.56 0.68 0.74];
  for k = 1:numel(hx)
    rectangle('Parent', ax, 'Position', [hx(k)-0.012 hy(k)-0.012 0.024 0.024], ...
         'FaceColor', C.warn, 'EdgeColor', C.warn);
  endfor
  circle(ax, [0.22 0.28], 0.020, C.ok, C.ok);
endfunction

% 09 uav — quadrotor X-frame with 4 rotors
function draw_uav(ax, C)
  O = [0.5 0.5]; L = 0.26;
  line(ax, [O(1)-L O(1)+L], [O(2)-L O(2)+L], 'Color', C.dark, 'LineWidth', 5);
  line(ax, [O(1)-L O(1)+L], [O(2)+L O(2)-L], 'Color', C.dark, 'LineWidth', 5);
  rectangle('Parent', ax, 'Position', [O(1)-0.035 O(2)-0.035 0.07 0.07], ...
       'FaceColor', C.accent, 'EdgeColor', C.accent);
  rotor(ax, [O(1)-L*0.72 O(2)-L*0.72], C.dark);
  rotor(ax, [O(1)+L*0.72 O(2)+L*0.72], C.dark);
  rotor(ax, [O(1)-L*0.72 O(2)+L*0.72], C.dark);
  rotor(ax, [O(1)+L*0.72 O(2)-L*0.72], C.dark);
endfunction

% 10 marine — torpedo AUV hull + fins + propeller
function draw_marine(ax, C)
  rectangle('Parent', ax, 'Position', [0.26 0.38 0.48 0.24], 'FaceColor', C.dark, ...
       'EdgeColor', C.dark, 'LineWidth', 3, 'Curvature', [1 1]);
  patch(ax, [0.26 0.26 0.34], [0.38 0.62 0.50], C.dark, 'EdgeColor', C.dark);
  patch(ax, [0.74 0.74 0.78], [0.34 0.66 0.50], C.accent, 'EdgeColor', C.accent);
  patch(ax, [0.74 0.74 0.70], [0.34 0.66 0.50], C.accent, 'EdgeColor', C.accent);
  line(ax, [0.74 0.80], [0.50 0.50], 'Color', C.warn, 'LineWidth', 3);
  line(ax, [0.77 0.77], [0.47 0.53], 'Color', C.warn, 'LineWidth', 3);
endfunction

% 11 auto — lane lines + ego car
function draw_auto(ax, C)
  for x = 0.22:0.10:0.62
    line(ax, [x x+0.06], [0.30 0.30], 'Color', [0.95 0.95 0.20], 'LineWidth', 3);
  endfor
  line(ax, [0.22 0.22], [0.22 0.78], 'Color', [0.85 0.85 0.85], 'LineWidth', 2);
  line(ax, [0.78 0.78], [0.22 0.78], 'Color', [0.85 0.85 0.85], 'LineWidth', 2);
  rectangle('Parent', ax, 'Position', [0.40 0.38 0.20 0.14], 'FaceColor', C.accent, ...
       'EdgeColor', C.accent, 'LineWidth', 3, 'Curvature', [0.3 0.3]);
  circle(ax, [0.44 0.39], 0.022, C.dark, C.dark);
  circle(ax, [0.56 0.39], 0.022, C.dark, C.dark);
  rectangle('Parent', ax, 'Position', [0.46 0.52 0.08 0.05], 'FaceColor', C.ok, 'EdgeColor', C.ok);
endfunction

% 12 sim — 3D grid world (wireframe cubes receding)
function draw_sim(ax, C)
  for k = 1:6
    s = 0.16 + (k-1)*0.07;
    x = 0.5 - s/2; y = 0.5 - s/2 + (k-1)*0.035;
    rectangle('Parent', ax, 'Position', [x y s s], 'FaceColor', 'none', ...
         'EdgeColor', C.grey, 'LineWidth', 1.5);
  endfor
  circle(ax, [0.5 0.42], 0.025, C.accent, C.accent);
  patch(ax, [0.5 0.52 0.50], [0.82 0.82 0.86], C.warn, 'EdgeColor', C.warn);
endfunction

% 13 ros — pub/sub node graph: publisher → topic → subscriber
function draw_ros(ax, C)
  node(ax, [0.22 0.50], "pub", C.dark);
  node(ax, [0.50 0.50], "topic", C.accent);
  node(ax, [0.78 0.50], "sub", C.dark);
  arrow(ax, [0.30 0.50], [0.42 0.50], C.dark, 3);
  arrow(ax, [0.58 0.50], [0.70 0.50], C.dark, 3);
endfunction

% 14 connect — hardware bus UART pinout (bus line + 3 devices + TX/RX)
function draw_connect(ax, C)
  line(ax, [0.18 0.82], [0.50 0.50], 'Color', C.dark, 'LineWidth', 6);
  devx = [0.30 0.50 0.70];
  for k = 1:numel(devx)
    rectangle('Parent', ax, 'Position', [devx(k)-0.05 0.30 0.10 0.34], ...
         'FaceColor', C.accent, 'EdgeColor', C.accent);
    line(ax, [devx(k) devx(k)], [0.30 0.50], 'Color', C.dark, 'LineWidth', 2);
  endfor
  text(ax, 0.30, 0.66, "TX", 'Color', C.ok, 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
  text(ax, 0.70, 0.66, "RX", 'Color', C.warn, 'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
endfunction

% ---------------------------------------------------------------------------
% Primitive helpers
% ---------------------------------------------------------------------------
function wheel(ax, c, r, fc)
  th = linspace(0, 2*pi, 40)';
  patch(ax, c(1) + r*cos(th), c(2) + r*sin(th), fc, 'EdgeColor', fc);
endfunction

function rotor(ax, c, fc)
  th = linspace(0, 2*pi, 32)';
  patch(ax, c(1) + 0.05*cos(th), c(2) + 0.05*sin(th), fc, 'EdgeColor', fc);
endfunction

function circle(ax, c, r, ec, fc)
  th = linspace(0, 2*pi, 40)';
  patch(ax, c(1) + r*cos(th), c(2) + r*sin(th), fc, 'EdgeColor', ec, 'LineWidth', 1);
endfunction

function joint(ax, p, ccol)
  circle(ax, p, 0.028, ccol, ccol);
endfunction

function arrow(ax, p1, p2, ccol, lw)
  line(ax, [p1(1) p2(1)], [p1(2) p2(2)], 'Color', ccol, 'LineWidth', lw);
  v = p2 - p1; v = v / norm(v);
  perp = [-v(2) v(1)];
  headlen = 0.045;
  tip = p2;
  left = tip - headlen*v + 0.018*perp;
  rght = tip - headlen*v - 0.018*perp;
  line(ax, [tip(1) left(1)], [tip(2) left(2)], 'Color', ccol, 'LineWidth', lw);
  line(ax, [tip(1) rght(1)], [tip(2) rght(2)], 'Color', ccol, 'LineWidth', lw);
endfunction

function node(ax, c, label, ccol)
  rectangle('Parent', ax, 'Position', [c(1)-0.05 c(2)-0.03 0.10 0.06], ...
       'FaceColor', ccol, 'EdgeColor', ccol, 'LineWidth', 2);
  text(ax, c(1), c(2), label, 'Color', [1 1 1], 'FontSize', 13, 'FontWeight', 'bold', ...
       'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
endfunction
