function imu = imuaddavwn(imu, awn, vwn)
% Add angular & velocity white noise.
%
% Prototype: imu = addstep(imu, awn, vwn)
% Inputs: imu - IMU data
%         awn - angular white noise in arcsec
%         vwn - velocity white noise in mm
% Output: imu - IMU data output with noise
%
% See also  imuadderr, imuseterr.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 24/04/2026
global glv
    n = length(imu);
    awn = awn*glv.sec;
    if length(awn)==1, awn=[awn;awn;awn]; end
    for k=1:3
        g = cumsum(imu(:,k))+awn(k)*randn(n,1);
        imu(:,k) = diffs(g);
    end
    if exist('wvn','var')
        vwn = vwn*0.001;
        if length(vwn)==1, vwn=[vwn;vwn;vwn]; end
        for k=4:6
            g = cumsum(imu(:,k))+vwn(k-3)*randn(n,1);
            imu(:,k) = diffs(g);
        end
    end

    