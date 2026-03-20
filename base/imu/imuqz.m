function [imu, qerr] = imuqz(imu, qzg, qza)
% SIMU add quantization noise.
%
% Prototype: imu = imuqz(imu, qzg, qza)
% Inputs: imu - raw SIMU data
%         qzg - gyro quantization noise, like RLG, in arcsec or rad
%         qza- acc quantization noise, like I/F convertor, in mm/s or g*s
% Outputs: imu - output SIMU data with quantization noise
%          qerr - quantization error
%
% Example
%   glvs; avp0 = avpset([0.0001;0.02;1]*glv.deg, 0, glv.pos0, 0);
%   imu = imustatic(avp0, 0.1, 180);
%   imu1 = imuqz(imu); imuplot(imu1);  % inspure(imu1, avp0, 'f');
%   alignvn(imu1, avp0(1:3), avp0(7:9));
%
% See also  quantiz, imudz, imuadderr, avarsimu.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/12/2022, 01/11/2025
global glv
    if nargin<3, qza=0.001; end  % default 1mm/s
    if nargin<2, qzg=0.5; end    % default 0.5 arcsec
    if length(qzg)==1; qzg=repmat(qzg,3,1); end
    if length(qza)==1; qza=repmat(qza,3,1); end
    if nargout==2, qerr=imu; end
    for k=1:3  % gyro
        if qzg(k)==0, continue; end
        if qzg(k)>100*glv.sec, qzg(k)=qzg(k)*glv.sec; end  % if in arcsec
        imu(:,k) = diff([0;fix(cumsum(imu(:,k))/qzg(k))])*qzg(k);
    end
    for k=1:3  % acc
        if qza(k)==0, continue; end
        if qza(k)>0.1, qza(k)=qza(k)*0.001; end  % if in mm/s
        imu(:,k+3) = diff([0;fix(cumsum(imu(:,k+3))/qza(k))])*qza(k);
    end
    if nargout==2, qerr(:,1:6)=imu(:,1:6)-qerr(:,1:6); end
    