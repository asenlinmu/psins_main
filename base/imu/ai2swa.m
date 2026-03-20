function swa = ai2swa(ai, k, k1, phi0, a0, wf)
% angular increament(ai) to HRG standing wave angle(swa), swa2ai
global glv
    if nargin<6, wf=0; end   % angular force, in deg/s
    if nargin<5, a0=0; end  % initial standing wave angle
    if nargin<4, phi0=0; end  % ADB angle
    if nargin<3, k1=10*glv.dph; end % ADB amplititude
    if nargin<2, k=0.27; end  % 
    if length(k)==1, k=repmat(k,3,1); end
    if length(k1)==1, k1=repmat(k1,3,1); end
    if length(phi0)==1, phi0=repmat(phi0,3,1); end
    if length(a0)==1, a0=repmat(a0,3,1); end
    if size(wf,2)==1, wf=repmat(wf,1,3); end
    if size(wf,1)==1, wf=repmat(wf,size(ai,1),1); end
    ts = diff(ai(1:2,end));  % imu input
    ai = [ai(1,:); ai];  % add first line
    [M,N] = size(ai); % N3=min(N,3);  swa = ai(:,1:N3);
    if N==2, N3=1;  % 1 HRG
    elseif N==5, N3=4;  % 4 HRG
    elseif N==7, N3=3;  % 3 HRG IMU
    end
    swa = ai(:,1:N3);
    for n=1:N3
        swa(1,n) = a0(n);
        for m=2:M
            km = swa(m-1,n)-k(n)*ai(m,n)-wf(m-1,n)*ts; swa(m,n) = km;
            for j=1:3
                swa(m,n) = km-k1(n)*ts*cos(2*(swa(m,n)+swa(m-1,n)-2*phi0(n)));
            end
        end
    end
    swa=[swa,ai(:,N3+1:end)]; swa(1,end)=swa(1,end)-ts;
