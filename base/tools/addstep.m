function data = addstep(data, t0, s, n, isfig)
% Add step to data at time 't0' with 's' and smooth with n-point.
%
% Prototype: data = addstep(data, t0, s, n, isfig)
% Inputs: data - data to processing
%         t0 - time point
%         s - step value
%         s - n-point smooth
%         isfig - figure flag
% Output: data - data after processing
%
% Example
%   dd = appendt((0:0.01:5)',1);  addstep(dd, 250, 1, 10, 1);
%
% See also  addslope, smoothol, smoothn.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 17/04/2026
    if nargin<5, isfig=0; end
    if nargin<4, n=10; end
    if nargin<3, s=1; end
    idx = find(data(:,end)>t0,1,'first');  data0=data;
    data(idx:end,1) = data(idx:end,1)+s;
    % s = smooth(data(idx-2*n:idx+2*n,1),n);
    [b, a] = fir1(n,1/n);   s = filtfilt(b, a, data(idx-2*n:idx+2*n,1));
    data(idx-n-1:idx+n-1,1) = s(n:3*n);
    if isfig==1
        myfig(data0(:,end), [data0(:,1),data(:,1)]);
    end