% 19-rotation-postion systematic calibration simulation.
% Ref:XieBo,Multiposition calibratiuon method of laser gyro SINS,JCIT,2011.
% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/01/2026
glvs
ts = 0.01; dT = 30;
att0 = [1; -91; -91]*glv.deg;  pos0 = posset(34,0,0);
% 19-position setting
paras = [
    1    0,1,0, 90, 9, dT*3, 5
    2    0,1,0, 90, 9, dT, 5
    3    0,1,0, 90, 9, dT, 5
    6    0,1,0, -270, 9, dT, 2
    7    0,0,1, 90, 9, 2, 5  %---------------
    8    1,0,0, 90, 9, dT, 5
    9    1,0,0, 90, 9, dT, 5
    10   1,0,0, 90, 9, dT, 5
    13   1,0,0, -270, 9, dT, 2
    14    0,0,1, 90, 9, 2, 5  %---------------
    15    0,0,1, 90, 9, dT, 5
    18    0,0,1, -270, 9, dT, dT
];  paras(:,5) = paras(:,5)*glv.deg;
att = attrottt(att0, paras, ts);
% att1=att; att1(:,end)=att1(:,end)/2; att3ddemo(att1(1:10:end,:),1);
% IMU simulation
imu = avp2imu(att,pos0);  % imuplot(imu);
imuplot(imu,-21);
% systematic calibration
imu1 = imuclbt(imu);  % binfile32('imusysclbt.bin', imu1);
[clbt, av] = sysclbt(imu1, pos0, [], [], 2);
clbtfile(clbt);
% clbtfile(clbt, 'clbt19.bin');
% binfile('imu19.bin', [imu1,imu1(:,end)]);

