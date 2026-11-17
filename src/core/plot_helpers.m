% plot_helpers.m — Headless figure & plotting helpers (struct of function handles).
%
%   P = plot_helpers();            % create helper object
%   hf = P.newfig(title);          % create an invisible figure
%   P.savefig(hf, filepath);       % print to PNG and close figure
%   P.closefig(hf);                % close figure without saving
%   P.plot_frame2d(hf, T, scale);  % draw SE(2) coordinate frame
%   P.plot_frame3d(hf, T, scale);  % draw SE(3) coordinate frame
%   P.plot_trajectory(hf, x, y, ...); % draw 2D path
%   P.plot_points(hf, pts, ...);   % draw 2D/3D points
%   P.plot_obstacles(hf, obs);     % draw circular obstacles obs = [cx cy r; ...]
%   P.plot_polygon(hf, V, ...);    % draw polygon from vertices V (2xN)
%   P.set_limits(hf, ...);         % axis equal/grid/limits

function s = plot_helpers()
  % make all new figures invisible by default (headless-safe)
  try; set(0, "defaultfigurevisible", "off"); end
  s.newfig   = @ph_newfig;
  s.savefig  = @ph_savefig;
  s.closefig = @ph_closefig;
  s.plot_frame2d = @ph_plot_frame2d;
  s.plot_frame3d = @ph_plot_frame3d;
  s.plot_trajectory = @ph_plot_trajectory;
  s.plot_points = @ph_plot_points;
  s.plot_obstacles = @ph_plot_obstacles;
  s.plot_polygon = @ph_plot_polygon;
  s.set_limits = @ph_set_limits;
endfunction

function hf = ph_newfig(title)
  hf = figure("visible", "off");
  % headless-safe: force gnuplot toolkit (fltk/qt need a visible display)
  try; graphics_toolkit(hf, "gnuplot"); catch; end
  if nargin > 1, title(title); end
  grid on; hold on;
  xlabel("x"); ylabel("y");
endfunction

function ph_savefig(hf, filepath)
  figure(hf);
  print(hf, filepath, "-dpng", "-r120");
  close(hf);
endfunction

function ph_closefig(hf)
  close(hf);
endfunction

function ph_plot_frame2d(hf, T, scale)
  if nargin < 3, scale = 0.5; end
  T = reshape(T, 3, 3);
  R = T(1:2, 1:2); p = T(1:2, 3);
  figure(hf); hold on;
  % x-axis red, y-axis green
  plot([p(1), p(1)+scale*R(1,1)], [p(2), p(2)+scale*R(2,1)], "r-", "LineWidth", 2);
  plot([p(1), p(1)+scale*R(1,2)], [p(2), p(2)+scale*R(2,2)], "g-", "LineWidth", 2);
  plot(p(1), p(2), "k.", "MarkerSize", 6);
endfunction

function ph_plot_frame3d(hf, T, scale)
  if nargin < 3, scale = 0.5; end
  T = reshape(T, 4, 4);
  R = T(1:3, 1:3); p = T(1:3, 4);
  figure(hf); hold on;
  plot3([p(1), p(1)+scale*R(1,1)], [p(2), p(2)+scale*R(2,1)], [p(3), p(3)+scale*R(3,1)], "r-", "LineWidth", 2);
  plot3([p(1), p(1)+scale*R(1,2)], [p(2), p(2)+scale*R(2,2)], [p(3), p(3)+scale*R(3,2)], "g-", "LineWidth", 2);
  plot3([p(1), p(1)+scale*R(1,3)], [p(2), p(2)+scale*R(2,3)], [p(3), p(3)+scale*R(3,3)], "b-", "LineWidth", 2);
  plot3(p(1), p(2), p(3), "k.", "MarkerSize", 6);
  grid on;
endfunction

function ph_plot_trajectory(hf, x, y, varargin)
  figure(hf); hold on;
  if nargin == 3
    plot(x, y, varargin{:});
  else
    plot(x, y, "-b", "LineWidth", 1.5);
  end
  % mark start and end
  plot(x(1), y(1), "go", "MarkerFaceColor", "g", "MarkerSize", 6);
  plot(x(end), y(end), "rx", "MarkerFaceColor", "r", "MarkerSize", 8);
endfunction

function ph_plot_points(hf, pts, varargin)
  pts = reshape(pts, [], 2);  % expect 2xN or Nx2 -> convert to Nx2
  % if pts is 2xN, transpose
  if size(pts,1) == 2 && size(pts,2) ~= 1
    pts = pts';
  end
  figure(hf); hold on;
  plot(pts(:,1), pts(:,2), varargin{:});
endfunction

function ph_plot_obstacles(hf, obs, varargin)
  % obs: [cx cy r; ...] circular obstacles
  figure(hf); hold on;
  if isempty(obs), return; end
  for i = 1:size(obs,1)
    th = linspace(0, 2*pi, 60)';
    cx = obs(i,1); cy = obs(i,2); r = obs(i,3);
    x = cx + r * cos(th); y = cy + r * sin(th);
    fill(x, y, [0.7 0.7 0.7], "EdgeColor", "k", "FaceAlpha", 0.4);
  end
endfunction

function ph_plot_polygon(hf, V, varargin)
  % V: 2xN vertices (closed or not)
  figure(hf); hold on;
  V = reshape(V, 2, []);
  V = [V, V(:,1)];  % close
  fill(V(1,:), V(2,:), [0.8 0.8 0.8], "EdgeColor", "k");
endfunction

function ph_set_limits(hf, xmin, xmax, ymin, ymax)
  figure(hf);
  axis([xmin xmax ymin ymax]);
  axis equal;
  grid on;
endfunction
