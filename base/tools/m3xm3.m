function m = m3xm3(m1, m2)
% 3x3 matrix multiply 3x3 matrix by batch process.
%
% Prototype: m = m3xm3(m1, m2)
% Inputs: m1, m2 - input matrix or array
% Output: m - output matrix array
% 
% Examples
%   m1 = randn(3); m2 = randn(3);
%   m = m3xm3(m1,m2);
%   err = m - m1*m2;
%
%   m1 = randn(10,9); m2 = randn(3);
%   m = m3xm3(m1,m2);
%
% See also  m3xv3, VmkXkn, m2attBatch, m2quaBatch.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/01/2025
    if size(m1,2)==3,  m1 = [m1(1,1:3), m1(2,1:3), m1(3,1:3)];  end
    if size(m2,2)==3,  m2 = [m2(1,1:3), m2(2,1:3), m2(3,1:3)];  end
    m = [ m1(:,1).*m2(:,1)+m1(:,2).*m2(:,4)+m1(:,3).*m2(:,7), ...
          m1(:,1).*m2(:,2)+m1(:,2).*m2(:,5)+m1(:,3).*m2(:,8), ...
          m1(:,1).*m2(:,3)+m1(:,2).*m2(:,6)+m1(:,3).*m2(:,9), ...
          m1(:,4).*m2(:,1)+m1(:,5).*m2(:,4)+m1(:,6).*m2(:,7), ...
          m1(:,4).*m2(:,2)+m1(:,5).*m2(:,5)+m1(:,6).*m2(:,8), ...
          m1(:,4).*m2(:,3)+m1(:,5).*m2(:,6)+m1(:,6).*m2(:,9), ...
          m1(:,7).*m2(:,1)+m1(:,8).*m2(:,4)+m1(:,9).*m2(:,7), ...
          m1(:,7).*m2(:,2)+m1(:,8).*m2(:,5)+m1(:,9).*m2(:,8), ...
          m1(:,7).*m2(:,3)+m1(:,8).*m2(:,6)+m1(:,9).*m2(:,9) ];
    if size(m,1)==1, m=[m(1:3); m(4:6); m(7:9)];  end

