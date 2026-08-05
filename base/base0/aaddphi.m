function att1 = aaddphi(att, phi)
% Get the calculated attitude, it can be denoted as 'att1 = att + phi'.
%
% Prototype: att1 = adelphi(att, phi)
% Inputs: att - attitude input
%         phi - platform misalignment angles from computed nav-frame to
%               ideal nav-frame
% Output: att1 - attitude output
%
% See also  aaddphi, qaddphi, qdelphi, qq2phi, qaddafa, qdelafa, qq2afa, aaddmu.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/07/2024
    if length(phi)==1, phi=[0;0;phi]; end % azimuth only, 2026-5-8
    att1 = q2att(qaddphi(a2qua(att),phi));