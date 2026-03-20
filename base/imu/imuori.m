function oristr = imuori(imu, maxline)
% Static SIMU orientation determination.
%
% Prototype: oristr = imuori(imu, maxline)
% Inputs: imu - SIMU array = [gx, gy, gz, ax, ay, az, t]
%         maxline - max record to calculate
% Output: oristr - orientation string
%               'U': Upper; 'D': Down;
%               'E': East; 'W': West; 'N': North; 'S': South.
%
% See also  imurfu, imuaxis, imuidx.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/01/2026
    if nargin<2, maxline=99; end
    oristr = char([]);
    for k=1:size(imu,1)
        if k>maxline, break; end
        [~,idx] = max(abs(imu(k,4:6)));  str='XXX';
        if idx==1
            w=vnormlz(imu(k,[2,3]));  wy=w(1); wz=w(2);
            if imu(k,4)>0
                if wy>0.707, str='UNW';  elseif wy<-0.707, str='USE';  elseif wz>0.707, str='UEN';  elseif wz<-0.707, str='UWS'; end
            elseif imu(k,4)<0
                if wy>0.707, str='DNE';  elseif wy<-0.707, str='DSW';  elseif wz>0.707, str='DWN';  elseif wz<-0.707, str='DES'; end
            end
        elseif idx==2
            w=vnormlz(imu(k,[1,3]));  wx=w(1); wz=w(2);
            if imu(k,5)>0
                if wx>0.707, str='NUE';  elseif wx<-0.707, str='SUW';  elseif wz>0.707, str='WUN';  elseif wz<-0.707, str='EUS'; end
            elseif imu(k,5)<0
                if wx>0.707, str='NDW';  elseif wx<-0.707, str='SDE';  elseif wz>0.707, str='EDN';  elseif wz<-0.707, str='WDS'; end
            end
        elseif idx==3
            w=vnormlz(imu(k,[1,2]));  wx=w(1); wy=w(2);
            if imu(k,6)>0
                if wx>0.707, str='NWU';  elseif wx<-0.707, str='SEU';  elseif wy>0.707, str='ENU';  elseif wy<-0.707, str='WSU'; end
            elseif imu(k,6)<0
                if wx>0.707, str='NED';  elseif wx<-0.707, str='SWD';  elseif wy>0.707, str='WND';  elseif wy<-0.707, str='ESD'; end
            end
        end
        oristr(k,:) = [sprintf('%2d-',mod(k,100)), str];
    end
