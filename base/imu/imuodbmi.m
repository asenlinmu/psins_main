function [imuod, t2, t0, t1] = imuodbmi(imuod, endStatic, isfig)
% Add imuod back mirror image for reverse navigation
%
% Prototype: [imuod, t2, t0, t1] = imuodbmi(imuod, endStatic, isfig)
% Inputs: imuod - imuod struct array [gyro,acc,od,t]
%         endStatic - [t(end)-endStatic,t(end)] to be in static base.
%         isfig - figure flag
% Outputs: imuod - imuod output array with forward+backward data
%          t2,t0,t2 - end/start/middle time
% 
% See also  imuodplot.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/12/2025
    if nargin<3, isfig=0; end
    if nargin<2, endStatic=[]; end
    if isempty(endStatic), endStatic=5; end
    t0 = imuod(1,end);  t1 = imuod(end,end);
    ts = diff(imuod(1:2,end));
    imuodBack = [flipud(imuod(:,1:end-1)), imuod(:,end)+imuod(end,end)-imuod(1,end)+ts];  % back imuod
    if endStatic>1
        gyroStatic = datacut(imuod(:,[1:3,end]), t1-endStatic,t1);
        eb = mean(gyroStatic);
        for k=1:3, imuodBack(:,k)=-imuodBack(:,k)+2*eb(k)*ts; end  % -(x-b)+b=-x+2b
    else
        imuodBack(:,1:3) = -imuodBack(:,1:3);
    end
    if size(imuod,2)>7  % if exist od data column
        dis = max(abs(imuod(end,7:end-1)-imuod(1,7:end-1)));
        if dis<1, imuodBack(:,7:end-1)=-imuodBack(:,7:end-1); end % if is distance increment
    end
    imuod = [imuod; imuodBack];  t2 = imuod(end,end);
    if isfig==1
        imuodplot(imuod);
    end
