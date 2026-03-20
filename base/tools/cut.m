function [data, idx] = cut(data0, t1, t2, co)
% Short for datacut.
%
% Prototype: [data, idx] = cut(data0, t1, t2)
% Inputs: data0 - input data, whose last column should be time index
%         t1, t2 - start & end time tags
%         co - closed or open interval
% Outputs: data, idx - output data & index in data0
%
% See also  datacut, datacuts.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/12/2025
    if nargin<4, co='['; end
    [data, idx] = datacut(data0, t1, t2, co);