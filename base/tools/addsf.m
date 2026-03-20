function x = addsf(x, sf, clm)
% Add scale factors.
%
% Prototype:  x = addsf(x, sf, clm)
% Inputs: x_in - input data with bias
%         sf - scale factor to multiply
%         clm - data column to add
% Outputs: x - output data with scale factors
%
% See also  mulsf, addslope, delbias, imudeldrift.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/12/2025 
    if nargin<3, clm=1:length(sf); end
    if length(sf)==1, sf=repmat(sf,1,length(clm)); end
    x = mulsf(x, 1+sf, clm);
