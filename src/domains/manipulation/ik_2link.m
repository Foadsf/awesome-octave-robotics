% ik_2link.m — Analytic inverse kinematics for a 2-link planar arm.
%
%   [q, reachable] = ik_2link(p, l1, l2)
%
%   p     : target end-effector position [x; y] (m)
%   l1, l2: link lengths (m)
%
%   Returns q = [q1a q1b; q2a q2b], two elbow solutions (columns), and a
%   reachable flag. Closed-form law of cosines:
%     D = (x^2+y^2-l1^2-l2^2)/(2 l1 l2),  q2 = atan2(±sqrt(1-D^2), D),
%     q1 = atan2(y,x) - atan2(l2 sin(q2), l1 + l2 cos(q2)).
%   Unreachable targets (|D|>1) return q=NaN and reachable=false.

function [q, reachable] = ik_2link(p, l1, l2)
  p = reshape(p, 2, 1);
  x = p(1); y = p(2);
  D = (x^2 + y^2 - l1^2 - l2^2) / (2 * l1 * l2);
  reachable = (abs(D) <= 1 + 1e-9);
  if ~reachable
    q = NaN(2, 2);
    return;
  end
  s2 = sqrt(max(0, 1 - D^2));
  q2a = atan2(s2, D);   % elbow up
  q2b = atan2(-s2, D);  % elbow down
  q1a = atan2(y, x) - atan2(l2 * sin(q2a), l1 + l2 * cos(q2a));
  q1b = atan2(y, x) - atan2(l2 * sin(q2b), l1 + l2 * cos(q2b));
  q = [q1a, q1b; q2a, q2b];
endfunction
