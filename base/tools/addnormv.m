function res = addnormv(vects)
% Add row-norm column to the input data.
%
% Prototype: res = addnormv(vects)
% Input: vects - data source input
% Output: res - =[vect2, normv(vects)]
%
% See also  normv.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/04/2026
    res = [vects, normv(vects)];