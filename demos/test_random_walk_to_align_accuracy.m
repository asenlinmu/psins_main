% Angular&Velocity Random Walk effect to initial alignment accuracy simulation
% See also  test_quantization_to_align_accuracy.m
% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/09/2025
glvs
[ts,pos0] = initp(1, 34);  T = 300;  N = 50;
ARW = 0.0001;  VRW = 1; % in deg/sqrt(h), ug/sqrt(Hz)
ARWorVRW=2;
if ARWorVRW==0
%% ARW&VRW to phiU
    ap0 = [zeros(3,1); pos0];  [wnie,g,gn,wN]=wnieg(pos0);
    imuerr = imuerrset(0, 0, ARW, VRW);
    res = zeros(N,4);  att=zeros(3,1);
    for k=1:N
        imu = imustatic(ap0, ts, T, imuerr);
        att = aligni0fitp(imu, pos0, 0);
        avp = inspure(imu,ap0,'O',0);  vN=avp(:,5);  t=avp(:,end);
        pp = polyfit(t, vN, 2);  dphiU=-2*pp(1)/(g*wN);
        res(k,:) = [att; dphiU]';
    end
    dy = norm([ARW*glv.dpsh/sqrt(T), 4*VRW*glv.ugpsHz/g/T^1.5]/wN)/glv.min;
    myfig, plot([res(:,3)-ap0(3),res(:,4)-ap0(3)]/glv.min,'--o');  xygo('k', 'dyaw'); hline([-dy,dy]);
    legend('i0fit', 'polyfit'); ptitle('std',[dy, std(res(:,3:4)/glv.min)]);
elseif ARWorVRW==1
%% ARW to phiU
    myfig; [wnie,g,gn,wN]=wnieg(glv.pos0); a2=[]; a1=[];
    for iter=1:N
        phiE = cumsum(randn(T,1)); vN = cumsum(g*phiE);  kk=1;
        t = 30:10:length(vN);
        for k=t
            pp = polyfit(1:k, vN(1:k), 2);
            a2(kk,:) = pp(1);  a1(kk,:) = pp(2);  kk=kk+1;
        end
        subplot(121); plot(t, ARW*glv.dpsh*a1/g/glv.sec); hold on
        subplot(122); plot(t, -ARW*glv.dpsh*2*a2/(g*wN)/glv.min); hold on
    end
    subplot(121); s = plotstd(3); grid on; xygo('T / s', 'phiE');
    plot(t, ARW*glv.dpsh/glv.sec.*sqrt(t)*1/2*3,'--','linewidth',2);  % 3sigma
    subplot(122); s = plotstd(3); grid on; xygo('T / s', 'phiU');
    plot(t, ARW*glv.dpsh/wN/glv.min./sqrt(t)*3,'--','linewidth',2);  % 3sigma
elseif ARWorVRW==2
%% VRW to phiU
    myfig; [wnie,g,gn,wN]=wnieg(glv.pos0); a2=[]; a1=[];
    for iter=1:N
        vN = cumsum(randn(T,1));  kk=1;
        t = 40:10:length(vN);
        for k=t
            pp = polyfit(1:k, vN(1:k), 2);
            a2(kk,:) = pp(1);  a1(kk,:) = pp(2);  kk=kk+1;
        end
        subplot(121); plot(t, VRW*glv.ugpsHz*a1/g/glv.sec); hold on
        subplot(122); plot(t, -VRW*glv.ugpsHz*2*a2/(g*wN)/glv.min); hold on
    end
    subplot(121); s = plotstd(3); grid on; xygo('T / s', 'phiE');
    plot(t, VRW*glv.ugpsHz/g/glv.sec./(t.^0.5)*2*3,'--','linewidth',2);  % 3sigma
    subplot(122); s = plotstd(3); grid on; xygo('T / s', 'phiU');
    boud = VRW*glv.ugpsHz/(g*wN)/glv.min./(t.^1.5)*4*3;
    plot(t, [-boud',boud'],'--','linewidth',2);  % 3sigma
end
