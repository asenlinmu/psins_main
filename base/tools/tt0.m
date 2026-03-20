function [data,dt] = tt0(data, t0)
% Adjust first time tag to t0.
%
% Prototype: [data,dt] = tt0(data, t0)
% Inputs: data - data input
%         t0 - first time tag, default 0
% Output; data - data output
%         dt - time to shift
%
% See also  adddt, getat, sortt, tshift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/12/2025
    if nargin<2, t0=0; end
    dt = t0-data(1,end);
    data(:,end) = data(:,end)+dt;