function data = txtcut(fname, clms, lns)
% txt file cut, with specified column and rows.
%
% Prototype: data = txtcut(fname, clms, lns)
% Inputs: fname - txt file name
%         clms - column to be extracted
%         lns - from lns(1) to lns(2) lines to be extracted
% Outputs: data - output double data array
%
% Example:
%   dd = txtcut('xxxfile.txt', [1:34,45:56], [2,inf]);
%
% See also  datacut, txtfilecut, txtfile, txtextract.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 13/11/2025
    if nargin<3, lns=[1;inf]; end
    if length(lns)<2, lns=[lns;inf]; end
    fid = fopen(fname,'r');  k=0;
    info = dir(fname); tl = fgetl(fid);  records = min(fix(info.bytes/length(tl)),diff(lns));
    frewind(fid);
    fout = fopen(['cut',fname],'w');
    timebar(1,records,'Extract txt data');
    while 1
        tl = fgetl(fid); k=k+1;
        if ~ischar(tl), break, end
        if k>=lns(1) && k<=lns(2)
            fprintf(fout, '%s\n', tl(clms));
        elseif k>lns(2)
            break;
        end
        timebar;
    end
    fclose(fid);
    fclose(fout);
    if nargout==1
        data = load(['cut',fname]);
    end