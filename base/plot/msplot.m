function msplot(mnp, x, y, xstr, ystr)
% my sub plot, m-row,n-column,p^th-subplot
global msplot_mn
    if mnp>=10, msplot_mn=fix(mnp/10)*10;
    else,       mnp=msplot_mn+mnp;       end
    if nargin<3, y=x; x=1:length(y); xstr='k'; ystr='value'; end  % msplot(mnp, y)
    if mod(mnp,10)==1, myfigure; end   % 如果是第一幅小图，则新建一个figure
    subplot(mnp); plot(x, y, 'linewidth', 2); grid on;
    if nargin==4, ystr = xstr; xstr = '\itt\rm / s'; end  % 如果只输入一个字符串，则默认xlabel为时间
    if exist('ystr', 'var'), xlabel(xstr); ylabel(ystr); end