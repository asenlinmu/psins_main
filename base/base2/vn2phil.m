function [phi0, en, phi] = vn2phil(vn, pos0, isfig)
% Calculating misalign angles from long-time (>20min) pure SINS velocity error.
%
% Prototype: [phi0, eN, eU] = vn2phil(vn, pos0, isfig)
% Inputs: vn - long-time pure open-loop SINS velocity error, in most case for static base
%         pos0 - init [lat; lon; hgt]
%         isfig - figure flag
% Output: phi0 - misalignment between calculating navigation frame and real
%                navigation frame at the start t0
%         en - esternward/northward/upward gyro-bias estimate
%         phi - misalignment angle at the end
%
% Example:
%   ap0 = [[0;0;0]*glv.deg;glv.pos0];
%   imu = imustatic(ap0, 0.1, 3600, imuerrset([0;0.01;0.02], 0, 0.000, .0));
%   avp = inspure(imu, [q2att(qaddphi(a2qua(ap0(1:3)),[.1;.1;3]*glv.min)); [0.1;0.2;0]; glv.pos0], 'H');
%   phi = vn2phil(avp(:,[4:6,end]), glv.pos0);
%
% See also  vn2philo, vn2phiu, vn2phistd, phiu2vn, aa2phi, vn2att.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/05/2025
global glv
    if nargin<3, isfig=1; end
    if size(vn,2)>4, vn=vn(:,[4:6,end]); end
    eth = earth(pos0);
    g = eth.g; R = (eth.RMh+eth.RNh)/2; we = glv.wie; t = vn(:,end);
    sL = sin(pos0(1)); cL = cos(pos0(1));  wN = we*cL;  wf = we*sL;  ws = sqrt(g/R);  V1 = sqrt(g*R);
    wet = we*t;  se = sin(wet);  ce = cos(wet);
    wst = ws*t;  ss = sin(wst);  cs = cos(wst);
    wft = wf*t;  sf = sin(wft);  cf = cos(wft);  ssf = ss.*sf; scf = ss.*cf; csf = cs.*sf; ccf = cs.*cf;
    %     phiE0,phiN0,          phiU0, eN,eU,                                                               VE0,VN0
    % AE = [V1*[ss.*sf, -ss.*cf], R*[wN*(cs.*sf-sL*se), -(cs.*cf-cL^2-sL^2*ce), cL*(sL*(1-ce)-we/ws*ss.*sf)], cs.*cf, cs.*sf];
    % AN = [V1*[ss.*cf,  ss.*sf], R*[wN*(cs.*cf-ce),    -(sL*se-cs.*sf),        cL*(se-we/ws*ss.*cf)],       -cs.*sf, cs.*cf];
    % X = lscov([AE;AN], [vn(:,1);vn(:,2)]);  phi0=X(1:3); eN=X(4); eU=X(5); VE0=X(6); VN0=X(7);  eL=1/cL; tL=sL*eL;
    %      phiE0;phiN0;phiU0;          dvE0;dvN0; eN;eU
    A = [ [V1*ssf -V1*scf -wN*sL*R*se  ccf csf    cL*R*eye(size(t))  sL*R*ce];
          [V1*scf  V1*ssf -wN*R*ce    -csf ccf    zeros(size(t))    -R*se] ];
    X = lscov(A, [vn(:,1);vn(:,2)]);
    % X = lscov(A(:,[1:3,6:7]), [vn(:,1);vn(:,2)]);  X = [X(1:3);0;0;X(4:5)];   % Vel0==0
    eN = X(6)*cL+X(7)*sL; eU = X(6)*sL-X(7)*cL;
    X1 = [ X(1)+eU*wN/ws^2; X(2); X(3);  X(4)+eN*R; X(5)-X(3)*wN*R; eN; eU];
    phi0=X1(1:3); VE0=X1(4); VN0=X1(5); eN=X1(6); eU=X1(7); eL=1/cL; tL=sL*eL;
    phit = [ phi0(1)*ccf+phi0(2)*csf-phi0(3)*we/ws*cL*scf-eN/ws*ssf+VE0/V1*ssf-VN0/V1*scf, ...
            -phi0(1)*csf+phi0(2)*ccf+phi0(3)*we/ws*cL*ssf-eN/ws*scf+VE0/V1*scf+VN0/V1*ssf, ...
             phi0(1)*eL*(se-sL*ccf)+phi0(2)*tL*(ccf-ce)+phi0(3)*(ce+we/ws*sL*ssf)+eN*tL*(se/we-scf/ws)-eU/we*se+VE0/V1*tL*scf+VN0/V1*tL*ssf ];
    en = [0;eN;eU];
    phi = phit(end,1:3)';
    if isfig==1
        myfig, 
        hur=0; if t(end)>10000, t=t/3600; hur=1; end
        subplot(311), plot(t, phit/glv.min); xygo('phi');  if hur==1, xlabel('\itt / \rmh'); end
        title(sprintf('\\phi_0=[%.3f, %.3f, %.3f]^\\prime;  V_{E0,N0}=[%.4f, %.4f]m/s;  \\epsilon_{N,U}=[%.4f, %.4f]\\circ/h;  \\deltaL_0=%.3fm', ...
            phi0(1)/glv.min,phi0(2)/glv.min,phi0(3)/glv.min, VE0,VN0, eN/glv.dph,eU/glv.dph, (X(7)+(eU*cL-eN*sL)*0)/we*glv.Re));
        subplot(312), plot(t, [vn(:,1:2)]); xygo('dV');  if hur==1, xlabel('\itt / \rmh'); end
        hold on, vnfit=reshape(A*X,length(A)/2,2); plot(t, vnfit, '--');
        title(['Single factor induced vel err magnitude(m/s): ', sprintf('%.3f; ',X.*[V1;V1;wN*sL*R; 1;1; cL*R;sL*R])]); %?
        subplot(313), plot(t, cumsum(vn(:,1:2)-vnfit)*diff(vn(1:2,end))); xygo('dP');  if hur==1, xlabel('\itt / \rmh'); end
    end
%% high accuracy for long-time align
    ts = diff(vn(1:2,end));  len = length(vn);
    eth = earth(pos0); R = sqrt(eth.RNh*eth.RMh); RM=R; RN=R; RM=eth.RMh; RN=eth.RNh;
    L = pos0(1); sL = sin(L); cL = cos(L); tL = sL/cL; eL = sec(L);
    wN = eth.wnie(2); wU = eth.wnie(3); g = -eth.gn(3);
    %     phi         dV           dP     eN/U
    F = [ 0   wU -wN  0    -1/RM   0   0  0 0
         -wU  0   0   1/RN   0    -wU  0  1 0 
          wN  0   0   tL/RN  0     wN  0  0 1
          0  -g   0   0     2*wU  0   0   0 0
          g   0   0  -2*wU  0     0   0   0 0
          0   0   0   0     1/RM   0   0  0 0
          0   0   0   eL/RN  0     0   0  0 0 ];  F(8:9,:) = 0;
    [Fk, Bk] = c2d(F, zeros(size(F)), ts);
    Fk0 = eye(size(Fk)); HE = zeros(len,9); HN = HE;
    for k=1:length(vn)
        Fk0 = Fk*Fk0;
        HE(k,:) = Fk0(4,:);  HN(k,:) = Fk0(5,:);
    end
    X0 = lscov([HE(:,[1:5,8:9]); HN(:,[1:5,8:9])], [vn(:,1);vn(:,2)]);
    if isfig==1
        vnt = [HE(:,[1:5,8:9])*X0, HN(:,[1:5,8:9])*X0];
        subplot(312), plot(t, [vnt(:,1:2)], '-.');
        subplot(313), plot(t, cumsum(vn(:,1:2)-vnt)*ts, '-.');
    end

    
