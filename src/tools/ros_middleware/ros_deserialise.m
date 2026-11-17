% ros_deserialise.m — Unpack a byte vector produced by ros_serialise into a struct.
%
%   msg = ros_deserialise(buf, fields)
%
%   buf    : byte vector (uint8) as produced by ros_serialise
%   fields : cell array of field names in transmission order
%
%   Inverse of ros_serialise. Returns a struct with the recovered float64 fields.

function msg = ros_deserialise(buf, fields)
  buf = reshape(buf, [], 1);
  if nargin < 2, fields = {}; end
  nf = numel(fields);
  msg = struct();
  off = 9;
  for i = 1:nf
    v = typecast(reshape(buf(off:off+7), 8, 1), "double");
    msg.(fields{i}) = double(v);
    off = off + 8;
  end
endfunction
