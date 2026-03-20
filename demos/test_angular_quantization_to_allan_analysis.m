% Angular quantization effect to Allan variance analysis
% Copyright(c) 2009-2025, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/11/2025
N = 0.0001*glv.dpsh;  ts = 0.01;  T = 10000;  len = T/ts;  t = (1:len)'*ts;
w = randn(len,1)*N*sqrt(ts);  % angular rate
Delta=[0,0.1,0.5,2]*glv.sec; n=length(Delta); yk=zeros(len,n); yk1=yk;
myfig;
for k=1:n
    y = quantiz(w+0.01*glv.dph*ts, Delta(k)); % with little-bias
    y1 = quantiz(w+10*glv.dph*ts, Delta(k));  % with big-bias
    subplot(n,2,2*k-1), plot(t, [y,y1]/ts/glv.dph);
    xygo(sprintf('w%d / (\\circ/h)',k)); title(sprintf('\\Delta=%.3f',Delta(k)/glv.sec));
    yk(:,k) = y;  yk1(:,k) = y1;
end
[s,tau] = avars(yk/ts/glv.dph,ts);  close(gcf);
[s1,tau1] = avars(yk1/ts/glv.dph,ts);  close(gcf); subplot(1,2,2);
loglog(tau,s,'linewidth',2);  xygo('\tau / s', '\sigma_A / (\circ/h)');
loglog(tau1,s1,'--','linewidth',2);
legend('w1','w2','w3','w4', 'w1 big-bias','w2 big-bias','w3 big-bias','w4 big-bias');


