function avp = avpinv(avp)
% Reverse avp(t), with vn & eb negative.
%
% Prototype: avp = avpinv(avp)
% Input: avp - navigation results, avp = [att,vn,pos,t]
% Output: avp - navigation results, avp = [att,vn,pos,t]
%
% See also  insinstant, attpure, inspurervs, attrvs.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/09/2023
    avp = flipud(avp);
    [m,n] = size(avp);
    if n>19, idx = [4:6,10:12,19];  % vn,eb,dT
    elseif n>15, idx = [4:6,10:12]; 
    elseif n>9, idx = 4:6;
    end
    avp(:,idx) = -avp(:,idx);