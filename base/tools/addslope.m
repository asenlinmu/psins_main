function data = addslope(data, kslope, nclm)
% Add slope to n-column data.
%
% Prototype: data = addslope(data, kslope, nclm)
% Inputs: data - data input
%         kslope - slope
%         nclm - n-columns index
%
% See also  addsf, addaging, adddt.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/09/2025
    if nargin<3, nclm=1; end
    if length(kslope)==1, kslope=repmat(kslope,length(nclm),1); end
    for k=1:length(nclm)
        data(:,nclm(k)) = data(:,nclm(k)) + (data(:,end)-data(1,end))*kslope(k);
    end
