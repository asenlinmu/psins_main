% Ballistic missile SINS error evaluasion by KF time-update covariance method
% See also  test_ballistic_missile_SINS_GPS.
% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/04/2026
glvs
ts = 0.01;
load tbinstrj_b.mat; % insplot(ap, 'ap');
imuerr = imuerrset([10;10;10],[1000;1000;1000],0.1,10, 0,0,0,0, 1000,1000,100,100, 100);
avperr = avperrset([1;1;5], 0, 1);
[sPk, tt] = inserrcov(ap, avperr, imuerr);
inserrcovplot(sPk, tt, 'dkgii', 'dpos');
inserrcovplot(sPk, tt, 'dkgyy');