% serial_packet.m — Build a framed serial packet (hardware-connectivity pattern).
%
%   [pkt, len] = serial_packet(cmd, payload)
%
%   cmd     : command byte (0..255)
%   payload : byte vector (uint8) payload
%
%   Frame: [0xAA header][len uint8][cmd uint8][payload][crc8]. CRC-8 (poly 0x07)
%   over len+cmd+payload. Real pattern used on embedded serial links.

function [pkt, len] = serial_packet(cmd, payload)
  payload = reshape(payload, [], 1);
  len = numel(payload);
  header = [uint8(0xAA); uint8(len); uint8(cmd)];
  crc = crc8([header(2:3); payload]);  % crc over len, cmd, payload
  pkt = [header; payload; crc];
endfunction
