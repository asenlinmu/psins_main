function [rv, qnb0] = q2rvBatch(qnb)
% Calculate rotation vector from attitude quaternions.
%
% Prototype: [rv, qnb0] = q2rvBatch(qnb)
% Input: qnb - attitude quaternion serial
% Output: rv - rotation vector serial between [qnbk_1; qnbk]
%         qnb0 - initial attitude quaternion
%
% See also  rv2qBatch, qq2rv, qq2phi, qq2afa.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 29/11/2024
    %rv = q2rv(qmul(qconj(q0),q1));
    q1 = [qnb(1:end-1,1), -qnb(1:end-1,2:4)];  q2 = qnb(2:end,:);
    q = [ q1(:,1) .* q2(:,1) - q1(:,2) .* q2(:,2) - q1(:,3) .* q2(:,3) - q1(:,4) .* q2(:,4),...
          q1(:,1) .* q2(:,2) + q1(:,2) .* q2(:,1) + q1(:,3) .* q2(:,4) - q1(:,4) .* q2(:,3),...
          q1(:,1) .* q2(:,3) + q1(:,3) .* q2(:,1) + q1(:,4) .* q2(:,2) - q1(:,2) .* q2(:,4),...
          q1(:,1) .* q2(:,4) + q1(:,4) .* q2(:,1) + q1(:,2) .* q2(:,3) - q1(:,3) .* q2(:,2) ];
    %% q2rv
    %if(q(1)<0), q = -q; end
    idx = q(:,1)<0;
    if ~isempty(idx), q(idx,:)=-q(idx,:); end
    n2 = acos(q(:,1));
    %if n2>1e-40, k = 2*n2/sin(n2);
    %else         k = 2; end
    k = repmat(2,length(q),1);
    idx = n2>1e-40;
    if ~isempty(idx), k(idx)=2*n2(idx)./sin(n2(idx)); end
    rv = [k.*q(:,2),k.*q(:,3),k.*q(:,4)];
    qnb0 = qnb(1,1:4)';
    