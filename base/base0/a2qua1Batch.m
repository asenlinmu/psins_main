function qnb = a2qua1Batch(att)
% Convert Euler angles to attitude quaternion (batch processing).
% NOTE: the input Euler angle sequence is pitch->yaw->roll (in F-U-R), 
% which is always used by launch vehicle.
%
% Prototype: qnb = a2qua1Batch(att)
% Input: att - att=[pitch; roll; yaw] in radians
% Output: qnb - attitude quaternion
%
% See also  q2att1, a2qua, a2quaBatch, attrf, axxx2a.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/09/2014

    att2 = att/2;
    s = sin(att2); c = cos(att2);
    sp = s(:,1); sr = s(:,2); sy = s(:,3); 
    cp = c(:,1); cr = c(:,2); cy = c(:,3); 
    qnb = [ cp.*cr.*cy + sp.*sr.*sy, ...
            cp.*sr.*cy - sp.*cr.*sy, ...
            sp.*sr.*cy + cp.*cr.*sy, ...
            sp.*cr.*cy - cp.*sr.*sy ];
    if size(att,2)>3, qnb(:,5) = att(:,end); end

% syms sp cp sr cr sy cy
% q = qmul([cp;0;0;sp],[cy;0;sy;0],[cr;sr;0;0])
