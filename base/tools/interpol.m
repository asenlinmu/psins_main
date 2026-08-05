function data = interpol(data, olth, isfig)
% Linear interpolation outliers.
%
% Prototype: data = interpol(data, olth, isfig)
% Inputs: data - data to processing
%         olth - outlier threshold
%         isfig - figure flag
% Outputs: data - data after processing
%
% Example
%    x = randn(10,1);  x(3)=100;  interpol(x);
%
% See also  deloutlier, smoothol.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/03/2026
    if nargin<3, isfig=1; end
    if nargin<2, olth=mean(abs(data(:,1))); end
    [M, N] = size(data);
    if isfig==1,  myfig(data);  end
    for n=1:N
        for m=2:M-1
            mn = (data(m-1,n)+data(m+1,n))/2;
            df1 = abs(data(m-1,n)-data(m+1,n));
            df2 = abs(data(m,n)-mn);
            if df1<olth && df2>olth, data(m,n) = mn;  end
        end
    end
    if isfig==1
        plot(data, '--');
    end
    
