function [V, idx] = vech(M)
% Half-VECtorization for symmetric matrix.
% 
% Prototype: V = vech(M)
% Input: M - nxn symmetric matrix
% Outputs: V - a column-vector with order of (n*(n+1)/2)x1
%          idx - linear index extract from M
%
% Example
%    M = randn(3);  M = M*M', V = vech(M),
%
% See also  unvech.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/07/2026
    mask = tril(true(size(M)));
    V = M(mask);
    if nargout>1
        idx = find(mask);
    end