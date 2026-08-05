function RTSProcessing(Phikk_1, Xk, Pk, Xkk_1, Pkk_1, tk)
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
        % if isfield(grts,'Xk')
        %     if n==length(grts.Xk(:,1)) && N==grts.N % already allocate
        %         grts.k=0; return;
        %     end
        % end
        grts = [];
        grts.k=0;  grts.N=N;  grts.phim=n;  grts.nn1=n*(n+1)/2;
        grts.Phikk_1 = zeros(n,n,N);
        grts.Pk = zeros(grts.nn1,N);  grts.Pkk_1 = grts.Pk;
        grts.Xk = zeros(n,N);  grts.Xkk_1 = grts.Xk;
        grts.tk = zeros(1,N);
        grts.mem0 = (n^2 + grts.nn1*2 + n*2 + 1)*N*8;  % memory pre-allocated in bytes
        grts.idx = find(tril(ones(n)));
    elseif nargin>=6 % state vector & covariance matrix push in
        grts.k = grts.k+1;
        if grts.k>grts.N  % alloc 10% more memory
            grts.N = fix(1.1*grts.N);
            grts.Phikk_1(:,:,grts.N) = 0;
            grts.Pk(:,grts.N) = 0;  grts.Pkk_1(:,grts.N) = 0;
            grts.Xk(:,grts.N) = 0;  grts.Xkk_1(:,grts.N) = 0;
            grts.tk(grts.N) = 0;
        end
        m = size(Phikk_1,1);
        if grts.phim>m  % reshape to save memory, if exist many random-constant state
            grts.phim = m; grts.Phikk_1 = grts.Phikk_1(1:m,:,:);
        end
        grts.Phikk_1(:,:,grts.k) = Phikk_1;
        grts.Xk(:,grts.k) = Xk; grts.Xkk_1(:,grts.k) = Xkk_1;
        grts.Pk(:,grts.k) = Pk(grts.idx); grts.Pkk_1(:,grts.k) = Pkk_1(grts.idx);
        grts.tk(grts.k) = tk;
    else  % POSProcessing(K);  TRS processing
        if nargin<1, K=grts.k; else, K=find(Phikk_1>grts.tk,1,'last'); end
        Xsk = grts.Xk(:,K);  [Psk, idxl, idxu] = unvech(grts.Pk(:,K));
        n = length(grts.Xk(:,1));  m=1:(n+1):n^2;
        idx0 = diag(Psk)<eps^2;  idx0m = m(idx0); % find 0-var
        timebar(1,grts.k, 'RTS processing.');
        grts.xkpk = zeros(2*n+1,K);  Pkk_1 = zeros(n);  Pk = Pkk_1;  Phikk_1 = eye(n);
        k=K;  grts.xkpk(:,k) = [Xsk; diag(Psk); grts.tk(k)];
        for k=K-1:-1:1
            Pkk_1(grts.idx)=grts.Pkk_1(:,k+1); Pkk_1(idxu)=Pkk_1(idxl);  Pkk_1(idx0m)=1.0;
            Pk(grts.idx)=grts.Pk(:,k); Pk(idxu)=Pk(idxl);
            Phikk_1(1:grts.phim,:) = grts.Phikk_1(:,:,k+1);
            Ksk = Pk*Phikk_1'*invbc(Pkk_1);
            Xsk = grts.Xk(:,k)+Ksk*(Xsk-grts.Xkk_1(:,k+1));
            Psk = Pk+Ksk*(Psk-Pkk_1)*Ksk'; % Psk = (Psk+Psk')/2;
            grts.xkpk(:,k) = [Xsk; diag(Psk); grts.tk(k)];
            timebar;
        end
        grts.xkpk = grts.xkpk';
        timebar(-1);
        grts.mem1 = (grts.phim*n + grts.nn1*2 + n*2 + 1 + 2*n+1)*K*8;  % memory needed in bytes
    end
