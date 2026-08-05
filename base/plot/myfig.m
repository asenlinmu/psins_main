function h = myfig(namestr, y_label)
% Short for myfigure.
%
% See also  myfigure, nextlinestyle.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 15/02/2014, 23/03/2022
    if ~exist('namestr','var')
        h0 = myfigure;
    else
        if ~ischar(namestr)  % myfig(data, ylabel);
            if numel(namestr)==1
                myfig; subplot(namestr); return;  % myfig, subplot(ijk);
            end
            if nargin<2, y_label='val'; end
            if ~ischar(y_label)
                myfig, plot(namestr,y_label); xygo('val');  % myfig, plot(t,data);
                return;
            end
            myfig; plot(namestr); xygo(y_label);  % myfig, plot(data);
            return;
        end
        h = myfigure(namestr);
    end
    if nargout==1
        h = h0;
    end