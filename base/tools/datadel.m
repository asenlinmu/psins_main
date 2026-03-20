function [data, idx] = datadel(data0, t1, t2, isTShift)
% delete data between time tags t1 & t2.
%
% Prototype: [data, idx] = datadel(data0, t1, t2)
% Inputs: data0 - input data, whose last column should be time index
%         t1, t2 - start & end time tags to delete
%         isTShift - time tag shift
% Outputs: data, idxi - output data & index in data0
%
% See also  datacut, getat, combinet.

% Copyright(c) 2009-2020, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/10/2020, 07/12/2024
    if nargin<4, isTShift=0; end
    if size(t1,2)==2 %  07/12/2024
        for k=1:size(t1,1), data0 = datadel(data0, t1(k,1), t1(k,2)); end
        data = data0; idx=[];
        return;
    end
    if nargin<3, t2=data0(end,end); end
    i1 = find(data0(:,end)>=t1, 1, 'first');
    i2 = find(data0(:,end)<=t2, 1, 'last');
    idx = [1:(i1-1),(i2+1):length(data0)]';
    if isTShift==1  % 12/02/2026
        data0((i2+1):end,end) = data0((i2+1):end,end)+(data0(i1-1,end)-data0(i2,end));
    end
    data = data0(idx,:);

