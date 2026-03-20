function imu = imuswapn(imu, swap)
% IMU swap noise in static base
%
% Prototype: imu = imuswapn(imu, swap)
% Input: imu - IMU before swap
%        swap - nx2 array for swap index
% Output: imu - IMU output after swap 
%
% See also  imucpn, imuadderr, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi'an, P.R.China
% 24/10/2025
    if nargin<2, swap = [2,1]; end
    m = mean(imu);
    for k=1:size(swap,1)
        imu(:,swap(k,:)) = [imu(:,swap(k,2))-m(swap(k,2))+m(swap(k,1)), imu(:,swap(k,1))-m(swap(k,1))+m(swap(k,2))];
    end