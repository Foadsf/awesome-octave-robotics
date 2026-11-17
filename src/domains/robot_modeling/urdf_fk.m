% urdf_fk.m — Forward kinematics from a parsed URDF (planar/general chain).
%
%   [T, ee] = urdf_fk(robot, q)
%
%   robot : struct from urdf_parse()
%   q     : joint positions vector, ordered like robot.joints (rad for revolute,
%           m for prismatic)
%
%   Returns T(i) = 4x4 homogeneous transform of link i (base frame -> link frame)
%   and ee = end-effector (last child link) position [x;y;z].
%
%   Each link transform = parent_transform * Trans(joint.origin) *
%   Rot(joint.axis, q_joint). Uses Rodrigues' rotation formula (real numeric
%   SE(3)), consistent with src/core/se3.m.

function [T, ee] = urdf_fk(robot, q)
  nJ = robot.num_joints;
  if nargin < 2, q = zeros(nJ, 1); end
  q = reshape(q, [], 1);
  % base link = link that is never a joint child
  ischild = false(robot.num_links, 1);
  linkname = {robot.links.name};
  for j = 1:nJ
    c = robot.joints(j).child;
    k = find(strcmp(linkname, c));
    if ~isempty(k), ischild(k) = true; end
  end
  root = find(~ischild, 1);
  if isempty(root), root = 1; end
  T = cell(robot.num_links, 1);
  computed = false(robot.num_links, 1);
  T{root} = eye(4); computed(root) = true;
  % resolve each link following joint chain
  for k = 1:robot.num_links
    T{k} = resolve_link(robot, k, q, linkname, T, computed);
  end
  % ee = last joint's child link position
  ee = zeros(3, 1);
  if nJ > 0
    child = robot.joints(nJ).child;
    k = find(strcmp(linkname, child));
    if ~isempty(k), ee = T{k}(1:3, 4); end
  end
endfunction

function Tk = resolve_link(robot, k, q, linkname, T, computed)
  if computed(k), Tk = T{k}; return; end
  % find the joint that produces link k
  j = find(strcmp({robot.joints.child}, robot.links(k).name));
  if isempty(j)
    Tk = eye(4); computed(k) = true; T{k} = Tk; return;
  end
  j = j(1);
  pname = robot.joints(j).parent;
  pk = find(strcmp(linkname, pname));
  Tp = eye(4);
  if ~isempty(pk)
    Tp = resolve_link(robot, pk, q, linkname, T, computed);
  end
  % joint transform = Trans(origin) * Rot(axis, q)   (or Trans(axis*q) prismatic)
  o = robot.joints(j).origin.xyz;
  a = robot.joints(j).origin.rpy;   % fixed offset rotation (roll-pitch-yaw)
  % fixed offset: Trans(o) * Rot_rpy(a)
  Toff = transl(o) * rot_rpy(a(1), a(2), a(3));
  if strcmpi(robot.joints(j).type, 'prismatic')
    Jm = transl(robot.joints(j).axis.xyz * q(j));
  else
    R = rot_axis(robot.joints(j).axis.xyz, q(j));
    Jm = [R, zeros(3, 1); 0 0 0 1];   % homogeneous rotation
  end
  Tk = Tp * Toff * Jm;
  computed(k) = true; T{k} = Tk;
endfunction

function R = rot_axis(axis, theta)
  axis = reshape(axis, 3, 1); axis = axis / norm(axis);
  ux = axis(1); uy = axis(2); uz = axis(3);
  c = cos(theta); s = sin(theta); C = 1 - c;
  R = [c + ux^2*C,     ux*uy*C - uz*s, ux*uz*C + uy*s;
       uy*ux*C + uz*s, c + uy^2*C,     uy*uz*C - ux*s;
       uz*ux*C - uy*s, uz*uy*C + ux*s, c + uz^2*C];
endfunction

function T = transl(v)
  v = reshape(v, 3, 1);
  T = [1 0 0 v(1); 0 1 0 v(2); 0 0 1 v(3); 0 0 0 1];
endfunction

function T = rot_rpy(r, p, y)
  % ZYX extrinsic roll-pitch-yaw, returns 4x4 homogeneous transform
  Rz = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0 0 1];
  Ry = [cos(p), 0, sin(p); 0 1 0; -sin(p), 0, cos(p)];
  Rx = [1 0 0; 0 cos(r), -sin(r); 0 sin(r), cos(r)];
  R = Rz * Ry * Rx;
  T = [R, zeros(3, 1); 0 0 0 1];
endfunction
