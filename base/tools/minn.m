function res = minn(scr, n, dim)
% Find the n-th min element.
%
% Prototype: res = minn(scr, n, dim)
% Inputs: scr - data source input
%         n - the n minimum element number
%         dim - =1 max along rows, =2 max along columns
%
% See also  maxn, max2, medianp, meann, sumn, avar.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/04/2026
    if nargin<3
        dim = 1;
    end
    scr = sort(scr, dim, 'ascend');
    if dim==1
        res = scr(n,:)';
    else  % dim==2
        res = scr(:,n);
    end
