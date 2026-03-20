% Long-time SINS pure inertial navigation simulation on static base.
% See also  test_SINS_trj, test_SINS, test_SINS_GPS_153, test_SINS_static_verify.
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/06/2011
glvs
T = 24*60*60;  % total simulation time length
[nn, ts, nts] = nnts(4, 1);
avp0 = [[0;0;0]; [0;0;0]; glv.pos0];
imuerr = imuerrset(0.00, .0, 0.0001, 0.1);
imu = imustatic(avp0, ts, T, imuerr);   % SIMU simulation
davp0 = avperrset([-0.0;0.0;0.5], [0.01;0.01;0.01]*0, [10;10;10]);
avp00 = avpadderr(avp0, davp0);
tscalepush('t/h');
avp = inspure(imu, avp00, 'V');  % pure inertial navigation
avperr = avpcmp(avp, avp0);
inserrplot(avperr);
tscalepop();

