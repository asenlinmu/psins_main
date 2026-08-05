function [avp, lv] = avplever(avp, lever, tDelay)
% AVP lever arm monitoring or compensation.
%
% Prototype: avp = avplever(avp, lever)
% Inputs: avp - initial AVP parameter array
%         lever - lever arms [lx^b; ly^b; lz^b];
%         tDelay - time delay;
% Outputs: avp - AVP parameter array after lever arm compensation
%          lv - =[lx;ly;0] levar arms estimation under static base 
%                & by up-axis rotation
%
% Example:
%    avp1 = avplever(avp, xkpk(end,16:18)', xkpk(end,19));
%
% See also  pp2lever, inslever.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/01/2021
    ts = diff(avp(1:2,end));
    eth = earth(avp(1,7:9)');
    Mpv = [0, 1/eth.RMh, 0; 1/eth.clRNh, 0, 0; 0, 0, 1];
    Cnb_1 = a2mat(avp(1,1:3)');
    He = zeros(length(avp),4);  Hn = He;  vE = avp(:,4);  vN = avp(:,5);
    for k=2:size(avp,1)
        Cnb = a2mat(avp(k,1:3)');
        wib = m2rv(Cnb_1'*Cnb)/ts;  % ~=web
        CW = Cnb*askew(wib);
        MpvCnb = Mpv*Cnb;
        avp(k,4:9) = avp(k,4:9)+ [CW*lever; MpvCnb*lever]';
        Cnb_1 = Cnb;
        He(k,:) = [1, k*ts, CW(1,1:2)];
        Hn(k,:) = [1, k*ts, CW(2,1:2)];
    end
    avp(1,4:9) = avp(2,4:9);  He(1,:) = He(2,:); Hn(1,:) = Hn(2,:);
    if nargin==3
        avp = avpinterp1(avp, avp(:,end)+tDelay);
        avp(:,end) = avp(:,end)-tDelay;
    end
    if nargout>1  % insplot(avp);
        le = lscov(He, vE);
        ln = lscov(Hn, vN);
        lv = (le(3:4)+ln(3:4))/2;  % [le(3:4), ln(3:4), lv]
        myfig, plot(avp(:,end), [vE,vE-He(:,3:4)*lv,He(:,1:2)*le(1:2), vN,vN-Hn(:,3:4)*lv,Hn(:,1:2)*ln(1:2)], 'linewidth',1);
        legend('V_{Eraw}', 'V_{Enew}', 'V_{Elinear}', 'V_{Nraw}', 'V_{Nnew}', 'V_{Nlinear}');
        title(sprintf('Lv=%.4f, %.4f(m)',lv(1),lv(2)));  xygo('V');
    end
    % avp(:,4:5) = [vE-He(:,3:4)*lever(1:2), vN-Hn(:,3:4)*lever(1:2)];  % for test
