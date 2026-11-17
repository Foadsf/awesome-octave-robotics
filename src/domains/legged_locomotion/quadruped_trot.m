% quadruped_trot.m — Trot gait scheduler + body-to-foot kinematics.
%
%   [contacts, foot_positions] = quadruped_trot(T, dt, v_body, yaw_rate)
%
%   Diagonal trot gait: legs (FL, FR, RL, RR). Two diagonal pairs alternate
%   stance/swing with a 50% duty cycle and a phase offset. Returns the contact
%   signal (1 = stance, 0 = swing) per leg per timestep and the desired foot
%   positions relative to the body (hip frame), including body translation
%   from v_body and rotation from yaw_rate during swing.
%
%   T        : total duration (s)
%   dt       : sample time (s)
%   v_body   : [vx vy] body velocity (m/s)
%   yaw_rate : yaw rate (rad/s)
%
%   Returns:
%     contacts      : (K x 4) binary contact matrix
%     foot_positions: (K x 4 x 3) desired foot positions [x y z] per leg
%
%   Pure Octave native code.

function [contacts, foot_positions] = quadruped_trot(T, dt, v_body, yaw_rate)
  if nargin < 4, yaw_rate = 0.0; end
  if nargin < 3, v_body = [0.1, 0]; end
  if nargin < 2, dt = 0.01; end
  if nargin < 1, T = 2.0; end

  % default hip / foot home positions (body frame): [FL FR RL RR]
  home = [ 0.25, -0.25,  0.25, -0.25;   % x (front +)
          -0.10,  0.10, -0.10,  0.10;   % y (left +)
           0.00,  0.00,  0.00,  0.00];  % z
  foot_z = -0.25;                        % stance foot height
  swing_z = 0.05;                        % swing foot clearance
  step_len = 0.10;                       % swing stride length

  K = round(T/dt) + 1;
  contacts = zeros(K, 4);
  foot_positions = zeros(K, 4, 3);
  period = 0.5;                          % trot period (s)
  duty = 0.5;

  for k = 1:K
    t = (k-1)*dt;
    % gait phase for each diagonal pair
    phase = mod(t, period) / period;
    % Pair A: FL & RR ; Pair B: FR & RL
    stance_A = (phase < duty);
    stance_B = (phase >= duty);
    contacts(k,1) = stance_A;   % FL
    contacts(k,2) = stance_B;   % FR
    contacts(k,3) = stance_B;   % RL
    contacts(k,4) = stance_A;   % RR

    % body displacement over time (integrate velocity)
    dx = v_body(1) * t;
    dy = v_body(2) * t;
    dyaw = yaw_rate * t;

    for leg = 1:4
      if contacts(k, leg)
        % stance: foot planted, moves backward relative to body as body advances
        foot_positions(k,leg,:) = [home(1,leg) - dx, home(2,leg) - dy, foot_z];
      else
        % swing: lift and move forward to next placement
        % swing progress within swing half-period
        p = mod(phase - duty + 1, 1);   % 0..1 over swing
        % smooth swing: raised arc in z, forward in x
        z = foot_z + swing_z * sin(p * pi);
        sx = home(1,leg) - dx + step_len * sin(p * pi) * (1 - 2*p);  % swing forward then back
        sy = home(2,leg) - dy;
        % apply yaw rotation to x/y offset
        c = cos(dyaw); s = sin(dyaw);
        xr = c * sx - s * sy;
        yr = s * sx + c * sy;
        foot_positions(k,leg,:) = [xr, yr, z];
      endif
    endfor
  endfor
endfunction
