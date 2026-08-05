function [phi, en] = fn2phi(fn, lti, typ, N)
% Calculating misalign angles from pure SINS velocity error.
%
% Prototype: [phi, en] = fn2phi(fn, lti, typ, N)
% Inputs: vn - pure SINS velocity error, in most case for static base
%         lti - latitude
%         typ - estimate type, =4,5 for 4 or 5 states
%         N - >=10 for plot misalign angle lines
% Output: phi - misalignment between calculating navigation frame and real
%               navigation frame
%         en - gyro-bias estimate
%
% See also  vn2phi, vn2phiu.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/03/2026
global glv
    if nargin<4, N=1; end
    if nargin<3, typ=5; end
    if length(lti)>1, lti=lti(1); end  % lti = pos;
    wU = glv.wie*sin(lti);  wN = glv.wie*cos(lti);  t = fn(:,end);  I = ones(size(t));
    H = [-wU*t  I      wN*wU*t.^2/2  -t          -wN*wU*t.^3/6; 
          I     wU*t  -wN*t          -wU*t.^2/2   wN*t.^2/2 ];
    X = lscov(H(:,1:typ), [-fn(:,1); fn(:,2)]/glv.g0);
    phi = X(1:3);
    en = X(4:typ);
    if N>=10
        if N>length(fn), N=length(fn); end
        K = floor(length(fn)/N);
        phik = zeros(N-1,4); enk = zeros(N-1,typ-3);
        for n=1:N
            [phi, en] = fn2phi(fn(1:n*K,:), lti, typ);
            phik(n,:) = [phi;fn(n*K,end)];  enk(n,:)=en';
        end
        msplot(311, phik(:,end), phik(:,1:2)/glv.sec); xygo('phiEN');  mylegend('phiE','phiN');
        msplot(312, phik(:,end), phik(:,3)/glv.min); xygo('phiU');
        msplot(313, phik(:,end), enk(:,1:(typ-3))/glv.dph); xygo('en');  mylegend('enN','enU');
    end
