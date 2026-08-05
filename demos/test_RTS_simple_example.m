% A simple example for RTS smoother
% See also test_SINS_GPS_RTS_153
% 26/06/2026
global grts;
Ts = 0.5;
Phi = [1 Ts; 0 1]; G = [0;1]; H = [1 0];
Q = 0.01;  R = 3;  sQ = sqrt(Q); sR = sqrt(R);
len = 100;  res = zeros(len, 8);  m = 3;
zk=0; xk = [1;0.1]; Xk = [10;1];  Pk = diag([10;1]);
RTSProcessing(2,len);
for k=1:len
    % dynamic model simu
    xk = Phi*xk + G*sQ*randn(1);
    if mod(k,m)==0
        zk = H*xk + sR*randn(1);
    end
    % KF
    Xk_1 = Xk;  Pk_1 = Pk;
    Xkk_1 = Phi*Xk_1;
    Pkk_1 = Phi*Pk_1*Phi' + G*Q*G';
    if mod(k,m)==0
        Kk = Pkk_1*H'/(H*Pkk_1*H'+R);
        Xk = Xkk_1+Kk*(zk-H*Xkk_1);
        Pk = (eye(2)-Kk*H)*Pkk_1;
    else
        Xk = Xkk_1; Pk = Pkk_1;
    end
    % RTS
    RTSProcessing(Phi,Xk,Pk,Xkk_1,Pkk_1,k);
    % save res
    res(k,:) = [xk; Xk; diag(Pk); zk; k]';
end
RTSProcessing();
myfig;
subplot(221), plot(res(:,end), res(:,[1,3]), grts.tk, grts.xkpk(:,1));  xygo('k','Xk1')
legend('Xk_{real}','Xk_{KF}','Xk_{RTS}');
subplot(222), plot(res(:,end), res(:,[2,4]), grts.tk, grts.xkpk(:,2));  xygo('k','Xk2')
subplot(223), plot(res(:,end), sqrt(res(:,5)), grts.tk, sqrt(grts.xkpk(:,3)));  xygo('k','sPk11')
legend('sPk_{KF}','sPk_{RTS}');
subplot(224), plot(res(:,end), sqrt(res(:,6)), grts.tk, sqrt(grts.xkpk(:,4)));  xygo('k','sPk22')
