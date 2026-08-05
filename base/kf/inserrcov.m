function [sPk, t] = inserrcov(ap, avperr, imuerr, isfig)
% INS error covariances analysis.
% Ref. 'Yan G. 一种基于实测飞行轨迹的惯导误差分析方法研究, NPT,2024'.
%
% Examples
%    trj = trjfile('trj10ms.mat');
%    imuerr=imuerrset(0.01,50,0.001,5,  0,0,0,0,  10,20, 5,5, 5);
%    avperr=avperrset([0.3;0.3;3], 0.0, 0.0);
%    inserrcov(trj.avp, avperr, imuerr);
%
%    ts=0.01; yaw=0:10*glv.dps*ts:360*glv.deg; ap=appendt(zeros(length(yaw),6),ts); ap(:,3)=yaw; 
%    inserrcov(ap);  % insplot(ap,'ap');
%
% See also  insupdate, kfupdate, sinsgps, kfstat, inserrcovplot.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/04/2025
global glv;
    if nargin<4, isfig=0; end
    ts = diff(ap(1:2,end));
    [imu, avp0, avp] = ap2imu(ap);  % imuplot(imu), insplot(avp);
    myfig,  dxyz = pos2dxyz(avp(:,7:end));
    subplot(221), plot(avp(:,end), avp(:,1:3)/glv.deg); xygo('att'); legend('\it\theta', '\it\gamma', '\it\Psi');  title('( a )');
    subplot(223), plot(avp(:,end), avp(:,4:6)); xygo('V'); legend('\itV\rm_E', '\itV\rm_N', '\itV\rm_U');  title('( b )');
    subplot(2,2,[2,4]), plot3(dxyz(:,1), dxyz(:,2), dxyz(:,3)); grid; zlabel('\Delta\itH\rm / m'), ylabel('\Delta\itL\rm / m'), xlabel('\Delta\it\lambda\rm / m');  title('( c )');
    %%
    len = length(imu);
    N = 36; L = 6;              % 33->36, 2026-04-18
    ins = insinit(avp0, ts);
    Ft = zeros(N);  Phik0 = eye(N);  Gamma = zeros(N,L);
    Pjk = []; for j=1:N, Pjk{j} = zeros(N); end
    Qjk = []; for j=1:L, Qjk{j} = zeros(N); end
    PQk = zeros(N,N+L);  Pk = zeros(N,N+L,len);
    ki = timebar(1, len, 'Trajectory-based INS error evaluation.');
    for k=1:len
        ins = insupdate(ins, imu(k,1:6));
        Ft(1:15,1:15) = etm(ins);
        Ft(1:3,16:24) = [-ins.wib(1)*ins.Cnb, -ins.wib(2)*ins.Cnb, -ins.wib(3)*ins.Cnb];
        Ft(4:6,25:36) = [  ins.fb(1)*ins.Cnb,   ins.fb(2)*ins.Cnb,   ins.fb(3)*ins.Cnb, ins.Cnb*diag(ins.fb.^2)];    
        if k*ts>3600  % 2026-06-02 vertical-channel deal
            ins.vn(3)=avp(k,4);  ins.pos(3)=avp(k,9);  Ft(:,[6,9])=0;  Ft([6,9],:)=0;
        end
        Phikk_1 = eye(N)+Ft*ts;  Gamma(1:6,1:6) = [-ins.Cnb,zeros(3);zeros(3),ins.Cnb]; % eye(6);
        Phik0 = Phikk_1*Phik0;
        for j=1:N,  Pjk{j} = Phik0(:,j)*Phik0(:,j)';  PQk(:,j) = diag(Pjk{j});  end
        for j=1:L,  Qjk{j} = Phikk_1*Qjk{j}*Phikk_1' + Gamma(:,j)*Gamma(:,j)';  PQk(:,N+j) = diag(Qjk{j});  end
        Pk(:,:,k) = PQk;  % P,Q 引起状态误差
        ki = timebar;
    end
    %%
    if nargin<3, imuerr=[];  end
    if isempty(imuerr),  imuerr=imuerrset(0.01,50,0.001,5,  0,0,0,0,  10,20, 5,5, 5);  end
    if nargin<2, avperr=[];  end
    if isempty(avperr),  avperr=avperrset([0.3;0.3;3], 0.0, 0.0);  end
    if length(imuerr.dKga)==15, imuerr.dKga=[imuerr.dKga;imuerr.dKga(13:15)]; end
    err = [avperr; imuerr.eb; imuerr.db; imuerr.dKga; imuerr.Ka2; [imuerr.web; imuerr.wdb]*sqrt(ts)].^2;
    Pkt = zeros(len,N);  Pki = zeros(len,42,9);
    for k=1:len
        s = zeros(N,1);
        for j=1:N+L
            s = s+Pk(:,j,k)*err(j);
        end
        Pkt(k,:) = s';   % k 时刻各状态误差
        for j=1:9
            Pki(k,:,j) = Pk(j,:,k).*err';  % 姿态、速度、位置误差
        end
    end
    sPkt = sqrt(Pkt);  t = imu(:,end);
    myfig
    subplot(311), plot(t, sPkt(:,1:3)/glv.min); xygo('phi');  legend('\phi_E', '\phi_N', '\phi_U', 'Location','Best');  title('( a )');
    subplot(312), plot(t, sPkt(:,4:6)); xygo('dv');  legend('\deltaV_E', '\deltaV_N', '\deltaV_U', 'Location','Best');  title('( b )');
    subplot(313), plot(t, [sPkt(:,7:8)*glv.Re,sPkt(:,9)]); xygo('dP');  legend('\deltaL', '\delta\lambda', '\deltaH', 'Location','Best');  title('( c )');
    %
    err = [];
    for j=1:9,  err(j,:) = Pki(end,:,j);  end;  err(:,end+1) = sum(err,2);  err=sqrt(err);  % 最后时刻误差，总误差
    err(1:3,:)=err(1:3,:)/glv.min; err(7:8,:)=err(7:8,:)*glv.Re; 
    myfig;
    subplot(311), plot(err(1:3,:)','-.o','linewidth',2); xlim([1,43]); grid on; ylabel('\it\phi\rm / ( \prime )'); legend('\phi_E', '\phi_N', '\phi_U', 'Location','Best');
    xtl = {'phiE/N/U', 'dvE/N/U', 'dlat/lon/H', 'ebx/y/z', 'dbx/y/z', 'dkgx/y/zx', 'dkgx/y/zy', 'dkgx/y/zz', 'dkax/y/zx', 'dkax/y/zy', 'dkax/y/zz', 'KA2x/y/z', 'wgx/y/z', 'wax/y/z', 'Total'};
    set(gca, 'xtick', [2:3:N+L,43], 'XTicklabel', xtl); title('( a )');
    subplot(312), plot(err(4:6,:)','-.o','linewidth',2); xlim([1,43]); grid on; ylabel('\delta\itV\rm / ( m/s )'); legend('\deltaV_E', '\deltaV_N', '\deltaV_U', 'Location','Best');
    set(gca, 'xtick', [2:3:N+L,43], 'XTicklabel', xtl); title('( b )');
    subplot(313), plot(err(7:9,:)','-.o','linewidth',2); xlim([1,43]); grid on; ylabel('\delta\itP\rm / m'); legend('\deltaL', '\delta\lambda', '\deltaH', 'Location','Best');
    set(gca, 'xtick', [2:3:N+L,43], 'XTicklabel', xtl); title('( c )');
    % [xkpk, kfs, trj] = tbinseval(ap1, avperr, imuerr, 1); 
    %
    sPk = sqrt(Pki+eps/10000);
    if isfig==0, return; end

    nn = ceil(1/diff(t(1:2)));
    t1 = t(1:nn:end);
    zlb={'phiE','phiN','phiU','dvE','dvN','dvU','dlat','dlon','dH'};
    zlb1={'phiE / \prime','phiN / \prime','phiU / \prime','dvE / m/s','dvN / m/s','dvU / m/s','\delta\itL\rm / m','\delta\it\lambda\rm / m','\delta\itH\rm / m'};
    hNull = figure; % set(hNull,'Visible','off');
    for k=1:9
        sPki = sqrt(Pki(:,:,k)+eps/10000);
        if k<4, sPki = sPki/glv.min;  % 1,2,3 /glv.min
        elseif k==7 || k==8, sPki = sPki*glv.Re;  end % 7,8 *glv.Re
        z = interp2(1:42, t1, sPki(1:nn:end,:), 1:0.1:42, t1);
        h=figure(k+100); set(h, 'WindowStyle','docked','NumberTitle','off','Name',zlb{k});
        mesh(1:0.1:42, t1, z); view(59,53);
        xtl = {'phiE/N/U', 'dvE/N/U', 'dlat/lon/H', 'ebx/y/z', 'dbx/y/z', 'dkgx/y/zx', 'dkgx/y/zy', 'dkgx/y/zz', 'dkax/y/zx', 'dkax/y/zy', 'dkax/y/zz', 'KA2x/y/z', 'wgx/y/z', 'wax/y/z'};
        set(gca, 'xtick', [2:3:N+L], 'XTicklabel', xtl);  xlim([1,42]);
        ylabel('t / s'); zlabel(zlb1{k});
    end
    close(hNull);
    %  1           4          7             10         13         16           18           22           25           28           31           34          37         40       
    % 'phiE/N/U', 'dvE/N/U', 'dlat/lon/H', 'ebx/y/z', 'dbx/y/z', 'dkgx/y/zx', 'dkgx/y/zy', 'dkgx/y/zz', 'dkax/y/zx', 'dkax/y/zy', 'dkax/y/zz', 'KA2x/y/z', 'wgx/y/z', 'wax/y/z'
    % myfig(t, sPk(:,34:36,4));
