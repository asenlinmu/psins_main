function [lever, dPn, err] = pp2lever(ap, pos, isfig)
% Lever arm calculation between two position array, (one contains attitude info).
%
% Prototype: lever = pp2lever(ap, pos, isfig)
% Inputs: ap - [att, pos] array
%         pos - position array
%         isfig - figure flag
% Outputs: lever - lever parameter = [lv_x, lv_y, lv_z]^b, from ap_pos to pos
%          dPn - pos bias
%
% Examples:
%    avpL = avplever(avp, [1;2;3]);  lv = pp2lever(avp, avpL(:,7:end), 1);  % verify
%
%    lever = pp2lever(avp, gps, 1);
%
% See also  avplever, inslever, pp2vn.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/10/2021, 10/06/2026
global glv
    if size(ap,2)>7, ap=ap(:,[1:3,7:10]); end
    if size(pos,2)>7, pos=pos(:,7:10);
    elseif size(pos,2)>4, pos=pos(:,4:7);  end
    if nargin<3, isfig=0; end
    eth = earth(pos(1,1:3)');
    aperr = avpcmp(ap, [pos(:,1:3)*0, pos], 'noatt');
    aperr(:,4:6) = -[aperr(:,5)*eth.clRNh, aperr(:,4)*eth.RMh, aperr(:,6)];  % lv^n = pos - ap_pos
    n = length(aperr);  Z = reshape(aperr(:,4:6)',3*n,1);  H = zeros(3*n,6);
    for k=1:n
        H(3*(k-1)+1:3*k,:) = [a2mat(aperr(k,1:3)'), glv.I33];  % Pgps=Pimu+dPn+Cnb*lv^b => Pgps-Pimu=[Cnb,I33]*[lv^b;dPn]
    end
    X = lscov(H, Z);
    lever = X(1:3);  dPn = X(4:6);
    err = [reshape(Z-H*X,3,n)',aperr(:,end)];
    if isfig==1
        myfig;
        subplot(211), plot(aperr(:,end), aperr(:,4:6)); xygo('DP');
        subplot(212), plot(aperr(:,end), err(:,1:3)); xygo('resdual err / m');  mylegend('DPE','DPN','DPU');
        ptitle('L', lever, 'DP', dPn);
    end

    % lv = aperr(:,4:7);
    % for k=1:length(aperr)
    %     lv(k,1:3) = aperr(k,4:6)*a2mat(aperr(k,1:3)');  % lv^n = Cnb * lv^b => lv^b' = lv^n' * Cnb
    % end
    % lever = mean(lv(:,1:3))';
    % if isfig==1
    %     myfig;
    %     subplot(211), plot(aperr(:,end), [aperr(:,4:6),normv(aperr(:,4:6))]); xygo('DP');
    %     subplot(212), plot(lv(:,end), [lv(:,1:3),normv(lv(:,1:3))]); xygo('lever');
    %     title(sprintf('L_x = %.3f, L_y = %.3f, L_z = %.3f (m)', lever(1),lever(2),lever(3)));
    % end

