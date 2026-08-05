function aoa(av, aoak, vT, dpchI)
% Angle Of Attach calculate & display
%
% Prototype: aoa(av, aosk, vT, dyawI)
% Inputs: av - att & vel  = [att; vel; t] array
%         aoak - ratio
%         vT - vel threshold
%         dpchI - pitch install error
%
% See also  att2c, aos, vn2att, vn2roll.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/03/2023
global glv;
    if nargin<4, dpchI = 0; end
    if nargin<3, vT = 1; end
    if nargin<2, aoak = 0; end
    a = att2c(av(:,[1:3,end]));
    wx = diff(a([1,1:end],1))/diff(a(1:2,end));
    att1 = vn2att(av(:,[4:6,end]));
    vel = normv(av(:,4:5));
    dpch = diffyaw(att1(:,1), av(:,1));  % diffpch
    idx = vel>vT;
    myfig,
    subplot(311), plot(av(:,end), [av(:,1:3), att1(:,1)]/glv.deg); grid on; ylabel('Att / \circ');  legend('\theta', '\gamma', '\psi','\theta_{track}'); x=get(gca,'xlim');
    subplot(312), plot(av(idx,end), [wx(idx)/glv.dps,vel(idx)]); grid on; legend('\omega_x / \circ/s', 'vel / m/s'); xlim(x);
    subplot(313), plot(av(idx,end), [dpch(idx)-dpchI,-aoak*vel(idx).*wx(idx)]/glv.deg); xygo('AOA');  legend('True','Simu'); xlim(x);


