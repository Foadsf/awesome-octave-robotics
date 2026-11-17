% legged_rl_policy.m — Tiny deterministic locomotion policy (forward pass).
%
%   [action, info] = legged_rl_policy(obs, W1, b1, W2, b2)
%
%   A 2-layer MLP policy (tanh hidden, linear output) mapping an observation
%   vector to a continuous action (e.g. desired foot placement offsets or
%   joint targets). No training here — the weights are the "policy"; this is
%   the inference routine used to deploy a policy headlessly.
%
%   obs    : (n_obs x 1) observation [vx vy yaw_rate roll pitch foot_contact...]
%   W1,b1  : (n_hid x n_obs), (n_hid x 1) hidden layer
%   W2,b2  : (n_act x n_hid), (n_act x 1) output layer
%
%   Returns action (n_act x 1) and info with preactivation values.
%
%   Pure Octave native code.

function [action, info] = legged_rl_policy(obs, W1, b1, W2, b2)
  obs = reshape(obs, [], 1);
  if nargin < 2, W1 = randn(16, length(obs)) * 0.5; end
  if nargin < 3, b1 = zeros(16, 1); end
  if nargin < 4, W2 = zeros(4, rows(W1)); end
  if nargin < 5, b2 = zeros(rows(W2), 1); end

  % forward pass
  z1 = W1 * obs + b1;
  h  = tanh(z1);
  z2 = W2 * h + b2;
  action = z2;   % linear output (could clamp)

  info.hidden = h;
  info.pre_act = z1;
  info.action = action;
endfunction

% --- default observation -> action for a quadruped trot policy ---
function action = legged_rl_policy_default(obs)
  % obs: [vx vy wz roll pitch leg1_contact..leg4_contact]
  % action: [desired step length, step width, body height] ... keep 4-dim
  W1 = [ 0.8  0.0  0.4  0.3 -0.3   1 0 0 0;   % example fixed policy weights
        -0.2  0.6  0.0 -0.1  0.4   0 1 0 0;
         0.1 -0.3  0.5  0.2 -0.2   0 0 1 0;
         0.0  0.0  0.0  0.0  0.0   0 0 0 1];
  b1 = zeros(4, 1);
  W2 = [1 0 0 0; 0 1 0 0; 0 0 0.5 0; 0 0 0 0.5];
  b2 = zeros(4, 1);
  [action, ~] = legged_rl_policy(obs, W1, b1, W2, b2);
endfunction
