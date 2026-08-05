function q = rv2qBatch(rv, q0)
% Calculate quaternion rotation vector by batch processing.
%
% Prototype: q = rv2qBatch(rv, q0)
% Inputs: rv - rotation vector serial between [qnbk_1; qnbk]
%         q0 - initial attitude quaternion
% Outputs: q - quaternion serial, q^n_bk
%
% See also  q2rvBatch, imutransatt, qq2rv, qq2phi, qq2afa, rv2q.

% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/05/2025
    if nargin<2, q0=[1;0;0;0]; end
    q = zeros(length(rv)+1,4);  q(1,:) = q0';
    for k=2:length(q)
        q(k,:) = qupdt(q(k-1,:)', rv(k-1,1:3)');
    end
    