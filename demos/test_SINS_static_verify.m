% Copyright(c) 2009-2015, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/11/2015
glvs
T = 48*3600;  % total simulation time length
[nn, ts, nts] = nnts(2, 10);
phi0 = [0.1;0.1;0.5]*glv.min; dv0 = [0.0;0.0]; dp0 = 0*[1000;500]/glv.Re; en = 0.02*[0.01;0.02;0.03]*glv.dph; dn = 0*[10;20]*glv.ug;
%% 惯导解算的误差
phiE0 = phi0(1); phiN0 = phi0(2); phiU0 = phi0(3); dvE0 = dv0(1); dvN0 = dv0(2); dL0 = dp0(1); dlon0 = dp0(2);
eE = en(1); eN = en(2); eU = en(3); DE = dn(1); DN = dn(2);
avp0 = avpset([0;0;0], [0;0;0], [40.0;110;380]);  eth = earth(avp0(7:9));
imuerr = imuerrset([eE;eN;eU]/glv.dph, [DE;DN;0]/glv.ug, 0.0, 0.0);
imu = imustatic(avp0, ts, T, imuerr);   % SIMU simulation
davp0 = avperrset([phiE0;phiN0;phiU0]/glv.min, [dvE0;dvN0;0.0], [dL0;dlon0;0]*glv.Re);
avp00 = avpadderr(avp0, davp0);
ins = insinit(avp00, ts);
len = length(imu);    avp = zeros(fix(len/nn), 10);
ki = timebar(nn, len, 'Pure inertial navigation processing.');
for k=1:nn:len-nn+1
	k1 = k+nn-1;
	wvm = imu(k:k1, 1:6);  t = imu(k1,7);
	ins = insupdate(ins, wvm);  ins.vn(3) = 0;  ins.pos(3) = avp0(9);
	avp(ki,:) = [ins.avp; t]';
	ki = timebar;
end
avperr = avpcmp(avp, avp0);
inserrplot(avperr,'avpNoV');  % phi0 = vn2phil(avp(:,[4:6,end]),avp0(7:9));
%% 误差方程的传播误差
R = sqrt(eth.RNh*eth.RMh); RM=R; RN=R; RM=eth.RMh; RN=eth.RNh;
L = avp0(7); sL = sin(L); cL = cos(L); tL = tan(L); eL = sec(L);
wN = eth.wnie(2); wU = eth.wnie(3); g = -eth.gn(3);
qnb = a2qua(avp0(1:3)); en = qmulv(qnb,imuerr.eb); dn = qmulv(qnb,imuerr.db);
X0 = [phiE0;phiN0;phiU0; dvE0;dvN0; dL0;dlon0]; U = [-eE;-eN;-eU; DE;DN; 0; 0];
F = [ 0   wU -wN  0    -1/RM   0   0
     -wU  0   0   1/RN   0    -wU  0  
      wN  0   0   tL/RN  0     wN  0  
      0  -g   0   0     2*wU  0   0  
      g   0   0  -2*wU  0     0   0  
      0   0   0   0     1/RM   0   0  
      0   0   0   eL/RN  0     0   0 ];
[Fk, Bk] = c2d(F, eye(size(F)), nts); Uk = Bk*U;  % 离散化
Xk = X0; XX = zeros(len/nn,8);
for k=1:length(XX)
    Xk = Fk*Xk+Uk;
    XX(k,:) = [Xk; k*nts]';
end
t = XX(:,end);
subplot(221), hold on, plot(t, XX(:,1:2)/glv.sec, '-.');
subplot(222), hold on, plot(t, XX(:,3)/glv.min, '-.');
subplot(223), hold on, plot(t, XX(:,4:5), '-.');
subplot(224), hold on, plot(t, XX(:,6:7)*glv.Re, '-.');
%% 近似解析误差, 2025-06-01,书中表4.3.1
R = eth.RMh; sqrt(eth.RNh*eth.RMh);
wf = wU; ws = sqrt(g/R); V1 = sqrt(g*R); we = glv.wie; wet = we*t; se = sin(wet); ce = cos(wet);
wst = ws*t; ss = sin(wst); cs = cos(wst); wft = wf*t; sf = sin(wft); cf = cos(wft);
ssf = ss.*sf; scf = ss.*cf; csf = cs.*sf; ccf = cs.*cf;
phiE = phiE0*ccf+phiN0*csf-phiU0*wN/ws*scf+dvE0/V1*ssf-dvN0/V1*scf-dL0*wU/ws*ssf...
      -eE/ws*scf-eN/ws*ssf-DE/g*csf-DN/g*(1-ccf)   +eU*we/ws^2*cL*((ce-1)*cos(2*L)+(1-ccf));
phiN = -phiE0*csf+phiN0*ccf+phiU0*wN/ws*ssf+dvE0/V1*scf+dvN0/V1*ssf-dL0*wU/ws*scf...
       +eE/ws*ssf-eN/ws*scf+DE/g*(1-ccf)-DN/g*csf   +eU*we/ws^2*cL*(sL*se+csf);
phiU = phiE0*eL*(se-sL*csf)+phiN0*tL*(ccf-ce)+phiU0*(ce+wU/ws*ssf)+dvE0/V1*tL*scf+dvN0/V1*tL*ssf+dL0*eL*(se-wU/ws*sL*scf)...
       -eE*eL*((1-ce)/we-sL/ws*ssf)+eN*tL*(se/we-scf/ws)-eU/we*se+DE/g*tL*(1-ccf)-DN/g*tL*csf;
dvEt = phiE0*V1*ssf-phiN0*V1*scf+phiU0*R*wN*(csf-sL*se)+dvE0*ccf+dvN0*csf+dL0*R*wU*(ce-ccf)...
       +eE*R*(csf-sL*se)-eN*R*(ccf-cL^2-sL^2*ce)+eU*R*cL*(sL*(1-ce)-we/ws*ssf)+DE/g*V1*scf+DN/g*V1*ssf;
dvNt = phiE0*V1*scf+phiN0*V1*ssf+phiU0*R*wN*(ccf-ce)-dvE0*csf+dvN0*ccf+dL0*R*we*(sL*csf-se)...
       +eE*R*(ccf-ce)-eN*R*(sL*se-csf)+eU*R*cL*(se-we/ws*scf)-DE/g*V1*ssf+DN/g*V1*scf;
dlat = phiE0*(ce-ccf)+phiN0*(sL*se-csf)-phiU0*cL*(se-we/ws*scf)-dvE0/V1*ssf+dvN0/V1*scf+dL0*(ce+we/ws*sL*ssf)...
       -eE/we*(se-we/ws*scf)-eN/we*(sL*(1-ce)-we/ws*ssf)+eU/we*cL*(1-ce)+DE/g*csf+DN/g*(1-ccf);
dlon = phiE0*eL*(sL*se-csf)+phiN0*eL*(ccf-cL^2-sL^2*ce)-phiU0*(sL*(1-ce)-we/ws*ssf)...
       +dvE0/V1*eL*scf+dvN0/V1*eL*ssf+dL0*tL*(se-we/ws*scf)+dlon0-eE/we*eL*(sL*(1-ce)-we/ws*ssf)...
       +eN/we*(we*cL*t+sL*tL*se-we/ws*eL*scf)+eU*sL/we*(we*t-se)+DE/g*eL*(1-ccf)-DN/g*eL*csf;
subplot(221), plot(t, [phiE,phiN]/glv.sec, ':','linewidth',2);
subplot(222), plot(t, phiU/glv.min, ':','linewidth',2);
subplot(223), plot(t, [dvEt,dvNt], ':','linewidth',2);
subplot(224), plot(t, [dlat,dlon]*glv.Re, ':','linewidth',2);
myfig % error-error
subplot(221), plot(t/24/3600, [phiE-avperr(:,1),phiN-avperr(:,2),XX(:,1)-avperr(:,1),XX(:,2)-avperr(:,2)]/glv.sec); xygo('t/d','phiEN');
subplot(222), plot(t/24/3600, [phiU-avperr(:,3),XX(:,3)-avperr(:,3)]/glv.min); xygo('t/d','phiU');
subplot(223), plot(t/24/3600, [dvEt-avperr(:,4),dvNt-avperr(:,5),XX(:,4)-avperr(:,4),XX(:,5)-avperr(:,5)]); xygo('t/d','dV');
subplot(224), plot(t/24/3600, [dlat-avperr(:,7),dlon-avperr(:,8),XX(:,6)-avperr(:,7),XX(:,7)-avperr(:,8)]*glv.Re); xygo('t/d','dlat');

