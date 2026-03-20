function v = m3xv3(m, v)
% 3x3 matrix multiply 3x1 vector by batch process.
%
% Prototype: v = m3xv3(m, v)
% Inputs: m, v - input matrix or array
% Output: v - output matrix array
% 
% Examples
%   m = randn(3); v = randn(10,3);
%   v = m3xv3(m,v)
%
%   m = randn(10,9); v = randn(10,3);
%   v = m3xv3(m,v)
%
% See also  m3xm3, VmkXkn, m2attBatch, m2quaBatch.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/02/2025
    if size(m,2)==3,  m = [m(1,1:3), m(2,1:3), m(3,1:3)];  end
    if size(v,2)==1,  v = v';  end
    v = [ m(:,1).*v(:,1)+m(:,2).*v(:,2)+m(:,3).*v(:,3), ...
          m(:,4).*v(:,1)+m(:,5).*v(:,2)+m(:,6).*v(:,3), ...
          m(:,7).*v(:,1)+m(:,8).*v(:,2)+m(:,9).*v(:,3),  ];

