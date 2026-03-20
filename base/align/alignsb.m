function [att, attk, eb, db] = alignsb(imu, pos, yaw0, isfig)
% SINS coarse align on static base.
%
% Prototype: [att, attk, eb, db] = alignsb(imu, pos, yaw0, isfig)
% Inputs: imu - SIMU data
%         pos - initial position
%         ywo0 - initial yaw
%         isfig - figure flag
% Outputs: att, attk - attitude align results Euler angles & quaternion
%          eb, db - gyro drift & acc bias test
%
% See also  dv2atti, alignvn, aligncmps, aligni0, alignpe, alignsbtp, insupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/09/2011, 17/05/2017
global glv
    if nargin<4, isfig=1; end
    if nargin<3, yaw0=[]; end
    ts = diff(imu(1:2,end));
    wbib = mean(imu(:,1:3),1)'/ts; fbsf = mean(imu(:,4:6),1)'/ts;
    if norm(wbib)<glv.wie/10, wbib(3)=glv.wie; end
    lat = asin(wbib'*fbsf/norm(wbib)/norm(fbsf)); % latitude determing via sensor
    if nargin<2     % pos not given
        pos = lat;
    end
    if length(pos)==1
        pos = [pos; 0; 0];
    end
    eth = earth(pos);
    if norm(eth.wnie)<7.29e-06, eth.wnie=[0;1;0]; end
    [qnb, att] = dv2atti(eth.gn, eth.wnie, -fbsf, wbib);
    dH = (-eth.gn(3)-norm(fbsf))/3.086e-6;  % 27/01/2025
    if nargin<2 && isfig
        resdisp('Coarse align resusts (att,lat_estimated/arcdeg,dH_est/m)', ...
            [[att; lat]/glv.deg;dH]);
    elseif isfig
        resdisp('Coarse align resusts (att,lat_estimated,lat_real/arcdeg,dH_est/m)', ...
            [[att; lat; pos(1)]/glv.deg;dH]);
    end
    if nargout>1 && length(imu)>100  % 06/11/2021
        wvm = [cumsum(imu(:,1:6),1), imu(:,end)];
        attk = zeros(fix(length(imu)/10),4)-1; k1 = 1;
        for k=1:10:length(wvm)
            [qnb, atti] = dv2atti(eth.gn, eth.wnie, -wvm(k,4:6)', wvm(k,1:3)');
            attk(k1,:) = [atti; wvm(k,end)]; k1 = k1+1;
        end
        attk(k1:end,:) = [];
        if isfig==1
            myfig;
            subplot(211); plot(attk(:,end), attk(:,1:2)/glv.deg); xygo('pr')
            subplot(212); plot(attk(:,end), attk(:,3)/glv.deg); xygo('y');
        end
    end
% 17/05/2017
%     wb = wbib/diff(imu(1:2,end));
%     fb = fbsf/diff(imu(1:2,end));
    if ~isempty(yaw0), att(3)=yaw0;  end  % 2024-08-31
    Cnb = a2mat(att);
    wb0 = Cnb'*eth.wnie; gb0 = Cnb'*eth.gn;
    eb = wbib - wb0;  db = fbsf + gb0;
    if nargout>1 && isfig==1
        subplot(211); ptitle('eb', eb/glv.dph);
        subplot(212); ptitle('db', db/glv.ug);
    end
