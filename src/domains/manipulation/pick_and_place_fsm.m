% pick_and_place_fsm.m — Pick-and-place finite state machine for a manipulator.
%
%   [state, cmd, info] = pick_and_place_fsm(event, state, data)
%
%   States:  IDLE -> APPROACH -> GRASP -> LIFT -> TRANSFER -> LOWER -> RELEASE -> IDLE
%   Events:  "start", "reached", "grasped", "lifted", "transferred", "lowered",
%            "released", "fault"
%
%   Returns the next state, a command struct (target pose, gripper open/close),
%   and an info struct with step count and fault flag.
%
%   Pure Octave native code.

function [state, cmd, info] = pick_and_place_fsm(event, state, data)
  persistent step_count
  if isempty(step_count), step_count = 0; end

  if nargin < 3, data = struct(); end
  if nargin < 2, state = "IDLE"; end
  if nargin < 1, event = "tick"; end

  if isfield(data, "target")
    cmd.target = data.target;   % [x y z roll pitch yaw]
  else
    cmd.target = zeros(6, 1);
  endif
  cmd.gripper = "open";
  info.fault = false;
  info.step = step_count;

  switch state
    case "IDLE"
      if strcmp(event, "start")
        state = "APPROACH"; cmd.gripper = "open";
        info.msg = "approach pick";
      endif
    case "APPROACH"
      cmd.gripper = "open";
      if strcmp(event, "reached")
        state = "GRASP";
        info.msg = "grasp";
      endif
    case "GRASP"
      cmd.gripper = "close";
      if strcmp(event, "grasped")
        state = "LIFT";
        info.msg = "lift";
      endif
    case "LIFT"
      cmd.gripper = "close";
      if strcmp(event, "lifted")
        state = "TRANSFER";
        info.msg = "transfer";
      endif
    case "TRANSFER"
      cmd.gripper = "close";
      if strcmp(event, "transferred")
        state = "LOWER";
        info.msg = "lower";
      endif
    case "LOWER"
      cmd.gripper = "close";
      if strcmp(event, "lowered")
        state = "RELEASE";
        info.msg = "release";
      endif
    case "RELEASE"
      cmd.gripper = "open";
      if strcmp(event, "released")
        state = "IDLE";
        info.msg = "idle";
        step_count = step_count + 1;
      endif
    case "FAULT"
      cmd.gripper = "open";
      if strcmp(event, "reset")
        state = "IDLE"; info.fault = false;
      endif
    otherwise
      state = "IDLE";
  endswitch

  if strcmp(event, "fault")
    state = "FAULT"; info.fault = true; info.msg = "fault";
  endif

  info.state = state;
  info.cmd = cmd;
endfunction

% --- run a full pick-and-place cycle and report ---
function report = pick_and_place_run(n_cycles)
  events = ["start", "reached", "grasped", "lifted", "transferred", ...
            "lowered", "released"];
  state = "IDLE"; report = struct();
  for c = 1:n_cycles
    for e = 1:length(events)
      [state, cmd, info] = pick_and_place_fsm(events(e), state);
    endfor
  endfor
  report.cycles = n_cycles;
  report.final_state = state;
  report.completed = strcmp(state, "IDLE");
endfunction
