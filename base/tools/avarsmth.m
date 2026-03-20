function [sigma, tau, Err, y0] = avarsmth(y0, tau0, nsmth, gyroORacc)
% Allan for data smooth.
%
% Prototype: [sigma, tau, Err, y0] = avarsmth(y0, tau0, nsmth, gyroORacc)
%
% Examples:
%    y0 = randn(1000000,1);  avarsmth(y0,0.01);
%    y0 = diff(randn(1000000,1));  avarsmth(y0,0.01);
%
%    k=2; avarsmth(imu,k); [sigma,tau]=avar(sumn(imu(:,k),1/ts)/glv.dph,1.0,0); loglog(tau,sigma,'linewidth',2);
%
% See also  avar, avarimu, avars.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/12/2025
global glv
    if size(y0,2)>1  % avarsmth(imu, kClm, nsmth);
        if nargin<4, gyroORacc=1; end
        if nargin<3, nsmth=[]; end
        kClm = tau0;  tau0 = diff(y0(1:2,end));
        f = 1/tau0/glv.dph;
        if kClm>3, gyroORacc=0; f=1/tau0/glv.mg; end
        [sigma, tau, Err, y0] = avarsmth(y0(:,kClm)*f, tau0, nsmth, gyroORacc);
        return;
    end
    if nargin<4, gyroORacc=1; end
    if nargin<3, nsmth=[]; end
    if nargin<2, tau0=0.01; end
    fs = fix(1/tau0);
    if isempty(nsmth), nsmth=fix([0.1;1;10;100]*fs); end
    smthstr{1} = 'no smooth';
    for k=1:length(nsmth)
        y0 = [y0, smooth(y0(:,1),nsmth(k))];
        smthstr{k+1} = sprintf('%.1fs smooth',nsmth(k)/fs);
    end
    [sigma, tau, Err] = avars(y0(nsmth(end):end-nsmth(end),:),tau0,gyroORacc);
    legend(smthstr);

