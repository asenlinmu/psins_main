function RTSProcessing1(Phikk_1, Xk, Pk, Xkk_1, Pkk_1, tk)
% RTS smoother, please see test_RTS_simple_example for a simple example.
%
% Prototype: RTSProcessing(Phikk_1, Xk, Pk, Xkk_1, Pkk_1, tk)
% Inputs: Phikk_1 - state transition matrix
%         Xk, Pk - state vector & covariance matrix @ tk
%         Xkk_1, Pkk_1 - state vector & covariance matrix forcast from tk_1 to tk
%         tk - time tag
%
% See also  POSProcessing, avpinterp1imu, sinsgps34.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/06/2026
global grts  % Global TRS structure variable
    if nargin==2 % init: POSProcessing(n, N)
        if isstruct(Phikk_1)  % POSProcessing(kf, tk)
            RTSProcessing(Phikk_1.Phikk_1, ...
                Phikk_1.xk, Phikk_1.Pxk, Phikk_1.xkk_1, Phikk_1.Pxkk_1, Xk);
            return;
        end
        n = Phikk_1;  N = Xk;
        if isfield(grts,'Xk')
            if n==length(grts.Xk(:,1)) && N==grts.N % already allocate
                grts.k=0; return;
            end
        end
        grts.k=0;  grts.N=N;
        grts.Phikk_1 = zeros(n,n,N);
        grts.Xk = zeros(n,N);  grts.Xkk_1 = grts.Xk;  grts.Xsk = grts.Xk;  
        grts.Pk = zeros(n,n,N);  grts.Pkk_1 = grts.Pk;  grts.Psk = grts.Pk;
        grts.tk = zeros(1,N);
        grts.mem0 = (n^2*N*4 + n*N*3 + N)*8;  grts.mem1 = grts.mem0;  % memory pre-allocated in bytes
    elseif nargin>=6 % state vector & covariance matrix push in
        grts.k = grts.k+1;
        if grts.k>grts.N  % alloc 10% more memory
            grts.N = fix(1.1*grts.N);
            grts.Phikk_1(:,:,grts.N) = 0;
            grts.Xk(:,grts.N) = 0;  grts.Xkk_1(:,grts.N) = 0;
            grts.Pk(:,:,grts.N) = 0;  grts.Pkk_1(:,:,grts.N) = 0;
            grts.tk(grts.N) = 0;
        end
        grts.Phikk_1(:,:,grts.k) = Phikk_1;
        grts.Xk(:,grts.k) = Xk; grts.Xkk_1(:,grts.k) = Xkk_1;
        grts.Pk(:,:,grts.k) = Pk; grts.Pkk_1(:,:,grts.k) = Pkk_1;
        grts.tk(grts.k) = tk;
    else  % POSProcessing(K);  TRS processing
        if nargin<1, K=grts.k; else, K=find(Phikk_1>grts.tk,1,'last'); end
        grts.Xsk = grts.Xk(:,1:K);  grts.Psk = grts.Pk(:,:,1:K);
        n = length(grts.Xk(:,1));  m=1:(n+1):n^2;
        idx0 = diag(grts.Pkk_1(:,:,grts.k))<eps^2;  idx0m = m(idx0); % find 0-var
        timebar(1,grts.k, 'RTS processing.');
        grts.xkpk = zeros(K,2*n+1);
        k=K; grts.xkpk(k,:) = [grts.Xsk(:,k); diag(grts.Psk(:,:,k)); grts.tk(k)]';
        grts.mem1 = (n^2*K*4 + n*K*5 + K*2)*8;  % memory needed in bytes
        for k=K-1:-1:1
            Pkk_1 = grts.Pkk_1(:,:,k+1); Pkk_1(idx0m)=1.0;
%            Ksk = grts.Pk(:,:,k)*grts.Phikk_1(:,:,k+1)'/Pkk_1;
            Ksk = grts.Pk(:,:,k)*grts.Phikk_1(:,:,k+1)'*invbc(Pkk_1);
            grts.Xsk(:,k) = grts.Xk(:,k)+Ksk*(grts.Xsk(:,k+1)-grts.Xkk_1(:,k+1));
            grts.Psk(:,:,k) = grts.Pk(:,:,k)+Ksk*(grts.Psk(:,:,k+1)-grts.Pkk_1(:,:,k+1))*Ksk';
%            grts.Psk(:,:,k) = (grts.Psk(:,:,k)+grts.Psk(:,:,k)')/2;
%            D = diag(grts.Psk(:,:,k));  D(idx0) = 0;  % no need
            grts.xkpk(k,:) = [grts.Xsk(:,k); diag(grts.Psk(:,:,k)); grts.tk(k)]';
            timebar;
        end
        timebar(-1);
    end
