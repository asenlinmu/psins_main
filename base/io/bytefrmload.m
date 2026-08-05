function bytes = bytefrmload(fname, clms, rows, skip)
% Load binary file as byte-frame array.
%
% Prototype: bytes = bytefrmload(fname, clms, rows, skip)
% Inputs: fname - file name
%         clms - column number of the bin file.
%         rows - rows to read, inf for read all
%         skip - skip bytes from the beginning of the file
% Output: bytes - byte-frame array read from the binary file
%
% Example: 
%    bts = load('xxx.bin', 25, inf);  open bts;  % open byte-frame for check like by UltraEdit
%    ttest(cnt2t(bys(:,1)));
%
% See also  checkfrmfile, binfrmfile, binfile, binfile32.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 04/05/2026
    if nargin<4, skip=0; end
    if nargin<3, rows=100; end
    fid = fopen(fname, 'rb');
    if skip>0, fseek(fid, skip, 'bof'); end
    bytes = fread(fid, [clms,rows], 'uint8')';
    fclose(fid);
