function qztest(data, ifimuplot)
% quantization test.
%
% Prototype: qztest(data)
% Input: data - data to test
%%
% See also  quantiz, imuqz, ttest.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/11/2025
global glv
    if nargin<2, ifimuplot=0; end
    [m,n] = size(data);
    x = (1:m)'/m;
    if ifimuplot==1, imuplot(data); end
    myfig;
    if n>=6  % qztest(imu)
        for k=1:3, subplot(3,2,2*k-1), y=(data(:,k)/glv.sec);  histogram(y,'Normalization','probability'); xygo('\Delta\theta / \prime\prime', 'N%'); end
%        for k=1:3, subplot(3,2,2*k-1), y=sort(data(:,k)/glv.sec);  plot(x,y); xygo('%','\Delta\theta / \prime\prime'); end
        for k=4:6, subplot(3,2,2*(k-3)), y=(data(:,k)/0.001);  histogram(y,'Normalization','probability'); xygo('\DeltaV / mm', 'N%');  end
%        for k=4:6, subplot(3,2,2*(k-3)), y=sort(data(:,k)/0.001);  plot(x,y); xygo('%','\DeltaV / mm');  end
    else
%        y=sort(data(:,1));  plot(x,y); xygo('%', '\DeltaVal');
        y=(data(:,1));  histogram(y,'Normalization','probability'); xygo('\DeltaVal','N');
    end
    
