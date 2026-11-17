% crc8.m — CRC-8, polynomial x^8 + x^2 + x + 1 (0x07), init 0x00.
%
%   c = crc8(data)   % data: uint8 vector

function c = crc8(data)
  data = reshape(data, [], 1);
  c = uint8(0);
  for i = 1:numel(data)
    c = bitxor(c, data(i));
    for b = 1:8
      if bitand(c, 0x80), c = bitxor(bitshift(c, 1), uint8(0x07));
      else, c = bitshift(c, 1); end
    end
  end
endfunction
