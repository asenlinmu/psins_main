function c = unichar(u)
% Return unicode by hex string or dec number.
%   ----- ylabel('$\varphi$', 'Interpreter', 'latex');
%
% Prototype: c = unichar(u)
% Input: u - hex string or dec number
% Output: c - unicode char
%
% See also  labeldef, xygo.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/02/2026
    if ischar(u)
        u = hex2dec(u);
    end
    c = char(u);