function imu = imudz(imu, dzg, dza)
% SIMU dead zone.
%
% Prototype: imu = imuqz(imu, qzg, qza)
% Inputs: imu - raw SIMU data
%         dzg - gyro dead zone, in deg/h
%         dza- acc dead zone, in ug
% Outputs: imu - output SIMU data with dead zone error
%
% See also  imuqz, imuadderr.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/12/2025
global glv
    if nargin<3, dza=1; end  % default ug
    if nargin<2, dzg=0.01; end    % default 0.01*glv.dph
    if length(dzg)==1; dzg=repmat(dzg,3,1); end
    if length(dza)==1; dza=repmat(dza,3,1); end
    if size(dzg,2)==1, dzg=[-dzg,dzg]; end
    if size(dza,2)==1, dza=[-dza,dza]; end
    dzg = dzg*glv.dph;  dza = dza*glv.ug;
    ts = diff(imu(1:2,end));
    for k=1:3  % gyro
        if dzg(k,1)==dzg(k,2), continue; end
        idx = imu(:,k)<dzg(k,2)*ts & imu(:,k)>dzg(k,1)*ts;
        imu(idx,k) = 0;
    end
    for k=1:3  % acc
        if dza(k,1)==dza(k,2), continue; end
        idx = imu(:,k+3)<dza(k,2)*ts & imu(:,k+3)>dza(k,1)*ts;
        imu(idx,k+3) = 0;
    end
    