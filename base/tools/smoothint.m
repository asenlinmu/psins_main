function data = smoothint(data, t0t1, n, isfig)
% Smooth data within time [t0,t1] extended by n-points.
%
% Prototype: data = smoothint(data, t0t1, n, isfig)
% Inputs: data - data to processing
%         t0t1 - time interval [t0,t1]
%         n - extended number of points
%         isfig - figure flag
% Outputs: data - data after processing
%
% See also  smoothol, smoothn.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/04/2026
    if nargin<4, isfig=1; end
    if nargin<3, n=10; end
    if numel(t0t1)>2
        data0 = data;
        for k=1:size(t0t1,1), data = smoothint(data, t0t1(k,1:2), n, 0); end
        if isfig==1, myfig, plot(data(:,2), [data0(:,1),data(:,1)]); xygo('val'); end
        return;
    end
    i0 = find(data(:,end)>t0t1(1),1,'first')-n;
    i1 = find(data(:,end)<t0t1(2),1,'last')+n;
    if isfig==1, myfig, plot(data(:,2), data(:,1)); xygo('val'); end
    data(i0:i1,1) = interp1(data([i0:i0+n,i1-n:i1],2),data([i0:i0+n,i1-n:i1],1),data(i0:i1,2),'spline');
    if isfig==1, plot(data(:,2), data(:,1)); end
