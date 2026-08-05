function fn = fb2fn(fb, att)
% Trans sepecific force fb in b-frame to fn in n-frame.
%
% Prototype: fn = fb2fn(fb, att)
% Inputs: fb - sepecific force in b-frame
%         att - attitude
% Outputs: fn - sepecific force in b-frame
%
% See also  vb2vn, fb2atti, insupdate.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/04/2026
    [m,n] = size(fb);
    if n==1
        fn = a2mat(att(1:3))*fb(1:3);
    else
        if n>6, fb=[fb(:,4:6)/diff(fb(2:3,end)),fb(:,end)]; end  % fb=imu;
        att = att2c(att(:,[1:3,end]));
        att = interp1n(att, fb(:,end));
        Cnb = a2matBatch(att);
        fn = [m3xv3(Cnb,fb(:,1:3)), fb(:,end)];
    end
    