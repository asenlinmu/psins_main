function [data, lns] = gpstxtload(fname, header, idx, separator)
% GPS txt file load.
%
% See also  gpsplot, imuplot.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 11/12/2025
    if nargin<4, separator=','; end
    fid = fopen(fname,'r');
    hlen = length(header);  N = length(idx);
    data = [];
    m = 1;  k = 1;
    while 1
        tline = fgetl(fid);  k = k+1;
        if ~ischar(tline), break; end
        if ~strcmp(tline(1:hlen),header), continue; end
        res = strfind(tline,separator);
        if isempty(res), continue; end
        if mod(m,1000)==1, data=[data;zeros(1000,N+1)]; end
        for n=1:N
            data(m,n) = sscanf(tline(res(idx(n))+1:end),'%f');
        end
        data(m,end) = k;
        m = m+1;
    end
    data(m:end,:) = [];
    lns = data(:,end);  data(:,end) = [];
    fclose(fid);
