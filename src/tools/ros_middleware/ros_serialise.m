% ros_serialise.m — Pack a ROS-like numeric message into a byte vector.
%
%   buf = ros_serialise(msg, fields)
%
%   msg    : struct with numeric fields
%   fields : cell array of field names in transmission order
%
%   Layout: magic(4 bytes = "ROSB") | nfields(4, uint32 LE) |
%   for each field: float64 little-endian (8 bytes). Mirrors ROS CDR-style
%   primitive serialisation without any ROS dependency.

function buf = ros_serialise(msg, fields)
  if nargin < 2, fields = fieldnames(msg); end
  nf = numel(fields);
  buf = zeros(8 + 8*nf, 1, "uint8");
  buf(1:4) = uint8([82, 79, 83, 66]);       % "ROSB"
  buf(5:8) = typecast(uint32(nf), "uint8"); % little-endian count
  off = 9;
  for i = 1:nf
    v = double(msg.(fields{i}));
    buf(off:off+7) = reshape(typecast(v, "uint8"), 1, 8);  % float64 LE bytes
    off = off + 8;
  end
endfunction
