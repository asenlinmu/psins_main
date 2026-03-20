function imu = imucpn(imu, sg, sa)
% SIMU compress noise on static base.
%
% Prototype: imu = imucpn(imu, sg, sa)
% Inputs: imu - SIMU data in
%         sg - gyro compress scale
%         sa - acc compress scale
% Outputs: imu - SIMU data out.
%
% Example:
%    imu1 = imucpn(imu, 0.5, 0.5);  imuplot(imu, imu1);
%
% See also  imudeldrift, imuadderr, delbias, imuswapn.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/11/2025
    if nargin<2, sg=0.5; end
    if nargin<3, sa=sg; end
    if length(sg)==1, sg=[sg;sg;sg]; end
    if length(sa)==1, sa=[sa;sa;sa]; end
    sga = [sg; sa];
    for k=1:6, m=mean(imu(:,k)); imu(:,k)=(imu(:,k)-m)*sga(k)+m; end