function dkga = dkga15(dkgii, dkgij, dkaii, dkaij)
% Set gyro/acc installation error vector, i.e. 
%
% Prototype: dka = dkga15(dkaii, dkaij)
% Inputs: dkgii - scale factor error, in ppm
%         dkgij - cross installation angle error, in arcsec
%         dkaii - scale factor error, in ppm
%         dkaij - cross installation angle error, in arcsec
% Output: dkga - installation error matrix, expressed as 15x1 vector
%
% See also  dkg9, dka6.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 02/07/2026
    if nargin<2, dkgij=dkgii; end
    if nargin<3, dkaii=dkgii; dkaij=dkgij; end
    dkg = dkg9(dkgii, dkgij);
    dka = dka6(dkaii, dkaij);
    dkga = [dkg; dka];
