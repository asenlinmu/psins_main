function [x, y] = gcaxy()
% Get x/y-data from gca (current axis)
%
% Prototype: [x, y] = gcaxy()
% Outputs: x,y - x/y data
%
% Examples
%    myfig, plot(randn(10,3)); [x,y]=gcaxy();
%
% See also gcalim.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/10/2025
    lines = findall(gca, 'type', 'line');
    x1{1} = get(lines(1), 'xdata')';  y1{1} = get(lines(1), 'ydata');  n=length(x1{1})';
    for k=2:length(lines)
        x1{k} = get(lines(k), 'xdata')';  y1{k} = get(lines(k), 'ydata')';
        if n~=length(x1{k}), n=0; end
    end
    if n~=0  % all the same length
        x = x1{1};
        for k=1:length(lines), y(:,k)=y1{k}; end
    else
        x = x1; y = y1;
    end
    if nargout==1, x=y; end
