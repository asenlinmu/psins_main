function idx = closest(data, val, UpDown)
% Find data index closest to val.
%
% Prototype: idx = closest(data, val)
% Inputs: data - 1-column data array or 2 column with time tag 
%         val - bound values
%         UpDown - increase or descend close flag
%                  1 for increase close, -1 for descend close, 2 for both
% Outputs: idx - data index or time tag
%
% See also  alignsars, alignvn, aligni0, alignsbtp, insupdate.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/02/2026
    if nargin<3, UpDown=2; end
    idx = [];
    for k=1:length(val)
        data0 = data(:,1)-val(k);
        if UpDown==1||UpDown==2   % increase close
            idx = [idx; find(data0(2:end)>=0 & data0(1:end-1)<0)];
        end
        if UpDown==-1||UpDown==2   % descend close
            idx = [idx; find(data0(2:end)<=0 & data0(1:end-1)>0)];
        end
    end
    idx = sort(idx);
    if size(data,2)>1
        idx = data(idx,end);  % the last column is time tag
    end