% rl_obstacle_avoidance.m — Tabular Q-learning obstacle-avoidance agent.
%
%   [Q, curve] = rl_obstacle_avoidance(grid, start, goal, opts)
%
%   grid   : 2D occupancy grid (1 = obstacle, 0 = free)
%   start, goal: [row, col]
%   opts   : struct (optional)
%     .episodes : (default 2000)
%     .alpha    : learning rate (default 0.1)
%     .gamma    : discount (default 0.95)
%     .epsilon  : exploration (default 0.1)
%
%   Returns Q-table (nS x nA) and learning curve (reward per episode).
%   Actions: 0=up,1=down,2=left,3=right.
%
%   Pure Octave native code.

function [Qmat, curve] = rl_obstacle_avoidance(grid, start, goal, opts)
  if nargin < 4 || ~isstruct(opts), opts = struct(); end
  n_ep = pc_pick(opts, "episodes", 2000);
  alpha = pc_pick(opts, "alpha", 0.1);
  gamma = pc_pick(opts, "gamma", 0.95);
  epsilon = pc_pick(opts, "epsilon", 0.1);
  [grid_rows, grid_cols] = size(grid);
  nS = grid_rows * grid_cols; nA = 4;
  Qmat = zeros(nS, nA);
  curve = zeros(n_ep, 1);
  actions = [-1 0; 1 0; 0 -1; 0 1];
  s0 = sub2ind([grid_rows, grid_cols], start(1), start(2));
  sg = sub2ind([grid_rows, grid_cols], goal(1), goal(2));
  rng(0);
  for ep = 1:n_ep
    s = s0; ep_reward = 0;
    for step = 1:200
      if rand() < epsilon
        a = randi(nA);
      else
        [~, a] = max(Qmat(s, :));
      endif
      r = floor((s - 1) / grid_cols) + 1;
      c = mod(s - 1, grid_cols) + 1;
      nr = r + actions(a, 1); nc = c + actions(a, 2);
      reward = -1; done = false;
      if nr < 1 || nr > grid_rows || nc < 1 || nc > grid_cols || grid(nr, nc) == 1
        reward = -10; nr = r; nc = c;
      elseif nr == goal(1) && nc == goal(2)
        reward = 100; done = true;
      endif
      s2 = sub2ind([grid_rows, grid_cols], nr, nc);
      Qmat(s, a) = Qmat(s, a) + alpha * (reward + gamma * max(Qmat(s2, :)) - Qmat(s, a));
      ep_reward = ep_reward + reward;
      s = s2;
      if done, break; end
    endfor
    curve(ep) = ep_reward;
  endfor
endfunction

function v = pc_pick(s, field, def)
  if isfield(s, field) && ~isempty(s.(field))
    v = s.(field);
  else
    v = def;
  endif
endfunction
