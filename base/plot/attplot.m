function attplot(varargin)
% att plot.
%
% Prototype: res = attplot(varargin)
% Inputs: varargin - atts (more than one att), attplot(att1, att2, att3, ...)
%          
% See also  insplot.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/01/2023
global glv
    myfig;
    if nargin==2
        if length(varargin{2})<=2  % attplot(att, 10);  rotserr
            att = varargin{1};  th = abs(varargin{2});
            if length(th)==1, th=[th;th]; end
            th1=[-th(1),th(1)]; th2=[-th(2),th(2)]; % th in arcsec
            attc = att2c(att);  datt = diff(attc([1,end],1:3));
            subplot(321), plot(att(:,end), att(:,1)/glv.deg), xygo('p');
                          title(sprintf('\\it\\theta\\rm_0 = %.4f',att(1,1)/glv.deg));
            subplot(323), plot(att(:,end), att(:,2)/glv.deg), xygo('r');
                          title(sprintf('\\it\\gamma\\rm_0 = %.4f',att(1,2)/glv.deg));
            subplot(325), plot(att(:,end), att(:,3)/glv.deg), xygo('y');
                          title(sprintf('\\it\\psi\\rm_0 = %.4f',  att(1,3)/glv.deg));
            subplot(322), plot(att(:,end), (att(:,1)-att(1,1))/glv.sec), xygo('\delta\it\theta\rm / ( \prime\prime )'); ylim(th1);
                          title(sprintf('%.2f, %.3f',(att(end,1)-att(1,1))/glv.sec, datt(1)/glv.deg));
            subplot(324), plot(att(:,end), (att(:,2)-att(1,2))/glv.sec), xygo('\delta\it\gamma\rm / ( \prime\prime )'); ylim(th1);
                          title(sprintf('%.2f, %.3f',(att(end,2)-att(1,2))/glv.sec, datt(2)/glv.deg));
            subplot(326), plot(att(:,end), (att(:,3)-att(1,3))/glv.sec), xygo('\delta\it\psi\rm / ( \prime\prime )'); ylim(th2);
                          title(sprintf('%.2f^{\\prime\\prime}, %.3f\\circ (%.3fppm@%.2fr)',...
                              (att(end,3)-att(1,3))/glv.sec, datt(3)/glv.deg, (att(end,3)-att(1,3))/datt(3)*1e6,datt(3)/(2*pi))); % rotserr
            return;
        end
        if ischar(varargin{2})
            att = varargin{1};
            switch varargin{2},
                case 'std',  % attplot(att, 'std')
                    m = mean(att); s = std(att);  mm = max(att)-min(att);
                    subplot(311), plot(att(:,1)/glv.deg,'-o','linewidth',1), xygo('k','p');
                                  title(sprintf('mean = %.5f (\\circ), std = %.3f (\\prime\\prime), max-min = %.1f (\\prime\\prime)', ...
                                      m(1)/glv.deg,s(1)/glv.sec,mm(1)/glv.sec));
                    subplot(312), plot(att(:,2)/glv.deg,'-o','linewidth',1), xygo('k','r');
                                  title(sprintf('mean = %.5f (\\circ), std = %.3f (\\prime\\prime), max-min = %.1f (\\prime\\prime)', ...
                                      m(2)/glv.deg,s(2)/glv.sec,mm(2)/glv.sec));
                    subplot(313), plot(att(:,3)/glv.deg,'-o','linewidth',1), xygo('k','y');
                                  title(sprintf('mean = %.5f (\\circ), std = %.1f (\\prime\\prime), max-min = %.1f (\\prime\\prime)', ...
                                      m(3)/glv.deg,s(3)/glv.sec,mm(3)/glv.sec));
                case 'y-pr',  % attplot(att, 'y-pr')
                    subplot(131), plot(att(:,3)/glv.deg, att(:,1)/glv.deg,'linewidth',1), xygo('y','p'); title('( a )');
                    subplot(132), plot(att(:,3)/glv.deg, att(:,2)/glv.deg,'linewidth',1), xygo('y','r'); title('( b )');
                    subplot(133), plot(att(:,2)/glv.deg, att(:,1)/glv.deg,'linewidth',1), xygo('r','p'); title('( c )');
            end
            return;
        end
    end
	nextlinestyle(-1);
    for k=1:nargin
        att = varargin{k};
        if size(att,2)==3, % roll==0
            subplot(211), plot(att(:,end), att(:,1)/glv.deg, nextlinestyle(1)), xygo('p');
            subplot(212), plot(att(:,end), att(:,2)/glv.deg, nextlinestyle(0)), xygo('y');
        else
            subplot(311), plot(att(:,end), att(:,1)/glv.deg, nextlinestyle(1)), xygo('p');
            subplot(312), plot(att(:,end), att(:,2)/glv.deg, nextlinestyle(0)), xygo('r');
            subplot(313), plot(att(:,end), att(:,3)/glv.deg, nextlinestyle(0)), xygo('y');
        end
    end
