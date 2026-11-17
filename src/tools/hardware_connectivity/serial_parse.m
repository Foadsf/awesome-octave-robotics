% serial_parse.m — Parse and verify a framed serial packet.
%
%   [cmd, payload, ok] = serial_parse(pkt)
%
%   pkt    : received byte vector (uint8)
%   cmd    : command byte (empty if invalid)
%   payload: payload bytes (empty if invalid)
%   ok     : true if header, length and CRC-8 all check out
%
%   Verifies the 0xAA header, that the declared length matches the frame, and
%   the CRC-8 over len+cmd+payload. Real serial-link framing validation.

function [cmd, payload, ok] = serial_parse(pkt)
  pkt = reshape(pkt, [], 1); cmd = []; payload = []; ok = false;
  if numel(pkt) < 4, return; end
  if pkt(1) ~= 0xAA, return; end
  declen = pkt(2);
  if numel(pkt) < 4 + declen, return; end
  c = crc8(pkt(2 : 2 + declen + 1));   % len + cmd + payload
  if c ~= pkt(4 + declen), return; end
  cmd = pkt(3);
  payload = pkt(4 : 3 + declen);
  ok = true;
endfunction
