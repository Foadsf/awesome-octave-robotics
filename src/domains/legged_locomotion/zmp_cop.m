% zmp_cop.m — Zero Moment Point / Center of Pressure from CoM dynamics.
%
%   c = zmp_cop(x, xdd, z, g)
%
%   For a CoM at constant height z, the ZMP (CoP) is:
%     c = x - (z / g) * x_ddot
%
%   x   : CoM position (m)
%   xdd : CoM acceleration (m/s^2)
%   z   : CoM height (m)
%   g   : gravity (m/s^2)

function c = zmp_cop(x, xdd, z, g)
  c = reshape(x, [], 1) - (z / g) * reshape(xdd, [], 1);
endfunction
