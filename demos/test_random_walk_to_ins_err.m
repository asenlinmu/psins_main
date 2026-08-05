% Random Walk to INS errors Simulation
% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/07/2026
glvs
ts = 1.0;
t = 3*24*3600;
len = fix(t/ts);
att0 = pry(0,0,0);  pos0 = llh(30,110,400);
ap = appendt(repmat([att0;pos0]', len, 1), ts);
imuerr = imuerrset(0, 0, [1;1;1]*0.001, [1;1;1]*1);
[sPk, t] = inserrcov(ap, zeros(9,1), imuerr, 0);
inserrcovplot(sPk, t, 'wgx');
inserrcovplot(sPk, t, 'wgy');
inserrcovplot(sPk, t, 'wgz');
inserrcovplot(sPk, t, 'wax');
inserrcovplot(sPk, t, 'way');
inserrcovplot(sPk, t, 'waz');
