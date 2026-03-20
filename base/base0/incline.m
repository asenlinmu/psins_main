function incl = incline(att, isfig)
% Calculating angle of inclination from Euler algles, DCM or quaternion.
%
% Prototype: incl = incline(att)
% Inputs: att - Euler algles, DCM or quaternion
%         isfig - figure flag
% Output: incl - angle of inclination
%
% See also  m2att, q2att, q2mat.

% Copyright(c) 2009-2019, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/04/2019
    if nargin<2, isfig=0; end
    [m,n] = size(att);
    if m==4&&n==1, att=q2att(att)';
    elseif m==3&&n==3, att=m2att(att)';  % incl = acos(Cnb(3,3));
    elseif m==3&&n==1, att=att'; end
    incl = acos(cos(att(:,1)).*cos(att(:,2)));
    if isfig==1
        global glv
        myfig, plot(att(:,end), [att(:,1:2),incl]/glv.deg); xygo('pr'); mylegend('p', 'r', 'incline');
    end
