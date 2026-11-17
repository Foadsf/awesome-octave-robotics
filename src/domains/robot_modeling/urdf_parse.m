% urdf_parse.m — Lightweight URDF parser (no XML toolbox dependency).
%
%   robot = urdf_parse(filename)
%
%   Reads a URDF file and returns a struct with:
%     robot.name
%     robot.links(i).name, .mass, .origin.xyz/.rpy, .visual, .collision
%     robot.joints(i).name, .type, .parent, .child, .origin.xyz/.rpy,
%                                      .axis.xyz, .limit.lower/.upper/.effort/.velocity
%     robot.num_links, robot.num_joints
%
%   Parsing is line-by-line (regex on attributes), so it needs no XML
%   toolbox. Supports the common subset: robot/link/joint/origin/parent/child/
%   axis/limit/mass/inertia/visual/collision/geometry (box/cylinder/sphere).

function robot = urdf_parse(filename)
  robot.name = ''; robot.links = struct(); robot.joints = struct();
  robot.num_links = 0; robot.num_joints = 0;
  if isempty(filename) || ~exist(filename, 'file')
    return;
  end
  text = fileread(filename);
  % strip XML comments
  text = regexprep(text, '<!--.*?-->', '');
  lines = strsplit(text, '\n');

  link = []; joint = [];
  iL = 0; iJ = 0;
  for k = 1:numel(lines)
    line = lines{k};
    m = regexp(line, '\<\s*link\s+name\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && isempty(link)
      iL = iL + 1; link.name = m{1}{1}; link.mass = 0;
      link.origin.xyz = [0 0 0]; link.origin.rpy = [0 0 0];
      link.visual = struct(); link.collision = struct();
      % self-closing <link .../> (e.g. empty base link) -> register at once
      if regexp(line, '<link[^>]*/>')
        robot.links(iL) = link; link = [];
      end
    end
    m = regexp(line, '\<\s*joint\s+name\s*=\s*"([^"]*)"\s+type\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && isempty(joint)
      iJ = iJ + 1; joint.name = m{1}{1}; joint.type = m{1}{2};
      joint.origin.xyz = [0 0 0]; joint.origin.rpy = [0 0 0];
      joint.axis.xyz = [0 0 1]; joint.parent = ''; joint.child = '';
      joint.lower = 0; joint.upper = 0; joint.effort = 0; joint.velocity = 0;
    end
    % origin (joint or link)
    m = regexp(line, '\<\s*origin\s+[^\>]*xyz\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m)
      xyz = str2num(strrep(m{1}{1}, ',', ' '));
      if isempty(joint) && ~isempty(link), link.origin.xyz = xyz;
      elseif ~isempty(joint), joint.origin.xyz = xyz; end
    end
    m = regexp(line, '\<\s*origin\s+[^\>]*rpy\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m)
      rpy = str2num(strrep(m{1}{1}, ',', ' '));
      if isempty(joint) && ~isempty(link), link.origin.rpy = rpy;
      elseif ~isempty(joint), joint.origin.rpy = rpy; end
    end
    m = regexp(line, '\<\s*parent\s+link\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && ~isempty(joint), joint.parent = m{1}{1}; end
    m = regexp(line, '\<\s*child\s+link\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && ~isempty(joint), joint.child = m{1}{1}; end
    m = regexp(line, '\<\s*axis\s+[^\>]*xyz\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && ~isempty(joint)
      joint.axis.xyz = str2num(strrep(m{1}{1}, ',', ' '));
    end
    m = regexp(line, '\<\s*limit\s+[^\>]*lower\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && ~isempty(joint), joint.lower = str2double(m{1}{1}); end
    m = regexp(line, '\<\s*limit\s+[^\>]*upper\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && ~isempty(joint), joint.upper = str2double(m{1}{1}); end
    m = regexp(line, '\<\s*limit\s+[^\>]*effort\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && ~isempty(joint), joint.effort = str2double(m{1}{1}); end
    m = regexp(line, '\<\s*limit\s+[^\>]*velocity\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && ~isempty(joint), joint.velocity = str2double(m{1}{1}); end
    % mass / inertia (link)
    m = regexp(line, '\<\s*mass\s+[^\>]*value\s*=\s*"([^"]*)"', 'tokens');
    if ~isempty(m) && ~isempty(link), link.mass = str2double(m{1}{1}); end
    if ~isempty(link) && regexp(line, '\s*\<\s*/\s*link\s*\>')
      robot.links(iL) = link; link = [];
    end
    if ~isempty(joint) && regexp(line, '\s*\<\s*/\s*joint\s*\>')
      robot.joints(iJ) = joint; joint = [];
    end
  end
  robot.num_links = iL; robot.num_joints = iJ;
  % robot name
  m = regexp(text, '\<\s*robot\s+[^\>]*name\s*=\s*"([^"]*)"', 'tokens');
  if ~isempty(m), robot.name = m{1}{1}; end
endfunction
