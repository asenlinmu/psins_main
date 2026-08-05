function [M, idxl, idxu] = unvech(V)
% inverse operation Half-VECtorization 'vech'.
% 
% Prototype: M = unvech(V)
% Input: V - a column-vector with order of (n*(n+1)/2)x1
% Output: M - nxn symmetric matrix
%         idxl,idxu - tri-lower(-1)/upper(1) matirx linear index
%
% Example
%    n=4; V=randn(n*(n+1)/2,1), [M,idxl,idxu] = unvech(V),
%
% See also  vech.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/07/2026
    n = floor(sqrt(length(V)*2));
    mask = tril(true(n));
    M = zeros(n);   M(mask) = V;
    if nargout>1
        idxl = find(tril(mask,-1));
        idxu = idxl;  k=1;
        for I=1:n
            for J=I+1:n, idxu(k)=(J-1)*n+I; k=k+1; end
        end
        M(idxu) = M(idxl);
    else
        M = M+M'-diag(diag(M));
    end