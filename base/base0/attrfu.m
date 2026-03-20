function att = attrfu(att, dirstr)
% Trans R-F-U Euler attitude angles to other direction attitude.
%
% Prototype: att = attrfu(att, dirstr)
% Inputs: att - attitude in
%         dirstr - raw SIMU X-Y-Z orientations including three characters, 
%               the orientation abbreviations are:
%               'U': Upper; 'D': Down; 'R': Right; 'L': Left; 'F': Front; 'B': Back;
%               'E': East; 'W': West; 'N': North; 'S': South.
% Output: att - attitude out
%
% Example
%   att = attrfu([0;0;0]*glv.deg, 'flu')/glv.deg,
%   att = attrfu([0;0;0]*glv.deg, 'urf')/glv.deg,
%
% See also  atttrans, imurfu, axxx2a, attrf, attrfu1.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/11/2025
    [~, Cbb] = imurfu([], dirstr);
    if size(att,2)==1
        att(1:3)=m2att(a2mat(att(1:3))*Cbb');
    else
        att(:,1:3)=m2attBatch(m3xm3(a2matBatch(att(:,1:3)),Cbb'));
    end