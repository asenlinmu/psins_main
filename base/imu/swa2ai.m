function ai = swa2ai(swa, k, k1, phi0, wf)
% HRG standing wave angle(swa) to angular increament(ai), ai2swa
global glv
    if nargin<5, wf=0; end
    if nargin<4, phi0=0; end
    if nargin<3, k1=10*glv.dph; end
    if nargin<2, k=0.27; end
    if length(k)==1, k=repmat(k,3,1); end
    if length(k1)==1, k1=repmat(k1,3,1); end
    if length(phi0)==1, phi0=repmat(phi0,3,1); end
    if size(wf,2)==1, wf=repmat(wf,1,3); end
    if size(wf,1)==1, wf=repmat(wf,size(swa,1)-1,1); end
    ts = diff(swa(1:2,end));
    [~,N] = size(swa);
    if N==2, N3=1;  % 1 HRG
    elseif N==5, N3=4;  % 4 HRG
    elseif N==7, N3=3;  % 3 HRG IMU
    end
    ai = swa(2:end,1:N3);
    for n=1:N3
        ai(:,n) = -1/k(n)*(swa(2:end,n)-swa(1:end-1,n)+(k1(n)*cos(2*(swa(1:end-1,n)+swa(2:end,n)-2*phi0(n)))+wf(:,n))*ts);
    end
    ai=[ai,swa(2:end,N3+1:end)];
