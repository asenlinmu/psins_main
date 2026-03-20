function data = resett(data, dt, t0)
% Time tag reset for equal interval.
%
% Prototype: data = resett(data, dt, t0)
% Inputs: data - data before reset, with last comumn to be time tag
%         dt - time interval
%         t0 - begin time
%
% See also  ttest appendt.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/10/2025
    if nargin<3, t0=data(1,end); end
    data(:,end) = t0+(0:size(data,1)-1)'*dt;
    