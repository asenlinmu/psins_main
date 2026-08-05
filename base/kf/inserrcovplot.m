function inserrcovplot(sPk, t, iifactor, jjavp)
% Plot to INS error covariances analysis.
% Examples
%   inserrcovplot(sPk, t, 'ebx');
%   inserrcovplot(sPk, t, 'eb', 'dpos');
%
% See also  inserrcov.

% Copyright(c) 2009-2026, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 19/04/2026
global glv;
    str = {'phiE','phiN','phiU', 'dvE','dvN','dvU', 'dlat','dlon','dH', 'ebx','eby','ebz', 'dbx','dby','dbz', ...
            'dkgxx','dkgyx','dkgzx','dkgxy','dkgyy','dkgzy','dkgxz','dkgyz','dkgzz', ...
            'dkaxx','dkayx','dkazx','dkaxy','dkayy','dkazy','dkaxz','dkayz','dkazz', ...
            'dkA2x','dkA2y','dkA2z', 'wgx','wgy','wgz', 'wax','way','waz'};
    unit = [glv.sec;glv.sec;glv.min; 1;1;1; 1/glv.Re;1/glv.Re;1];
    %%
    if nargin<4  % inserrcovplot(t, sPk, ifactor)
        for ii=1:length(str)
            if strcmp(str{ii},iifactor)==1, break; end
        end
        myfig,
        % subplot(331); title([iifactor,'->...']);
        % for k=1:9, subplot(3,3,k), plot(t, sPk(:,ii,k)/unit(k),'linewidth',2); xygo(str{k}); end
        subplot(311); plot(t, [sPk(:,ii,1),sPk(:,ii,2),sPk(:,ii,3)]/glv.min,'linewidth',2); xygo('phi'); mylegend('phiE','phiN','phiU'); title([iifactor,'->...']);
        subplot(312); plot(t, [sPk(:,ii,4),sPk(:,ii,5),sPk(:,ii,6)],'linewidth',2); xygo('dv'); mylegend('dvE','dvN','dvU');
        subplot(313); plot(t, [[sPk(:,ii,7),sPk(:,ii,8)]*glv.Re,sPk(:,ii,9)],'linewidth',2); xygo('dP'); mylegend('dlat','dlon','dH');
        return;
    end
    %%
    strs = {'phi','dv','dpos','eb','db','dkg*x','dkg*y','dkg*z','dka*x','dka*y','dka*z','dkA2','wg','wa'};
    ii = 1:3;
    for k=1:length(strs)
        if strcmp(strs{k},iifactor)==1, ii=(k-1)*3+1:k*3; break; end
    end
    switch(iifactor)
        case 'dkgx*', ii=[16,19,22];        case 'dkgy*', ii=[17,20,23];        case 'dkgz*', ii=[18,21,24];
        case 'dkax*', ii=[25,28,31];        case 'dkay*', ii=[26,29,32];        case 'dkaz*', ii=[27,30,33];
        case 'dkgii', ii=[16,20,24];        case 'dkaii', ii=[25,29,33];
    end
    xstr{1}=str{ii(1)}; xstr{2}=str{ii(2)}; xstr{3}=str{ii(3)};
    switch(iifactor)
        case 'dkgij', ii=[19,22,23]; xstr{1}='dkgxy&yx'; xstr{2}='dkgxz&zx'; xstr{3}='dkgyz&zy';  ii2=[17,18,21];
        case 'dkaij', ii=[28,31,32]; xstr{1}='dkaxy&yx'; xstr{2}='dkaxz&zx'; xstr{3}='dkayz&zy';  ii2=[26,27,30];
    end
    jj = 7:9;
    for k=1:3
        if strcmp(strs{k},jjavp)==1, jj=(k-1)*3+1:k*3; break; end
    end
    if exist('ii2','var')
        for m=1:3
            for n=1:3,  sPk(:,ii(n),jj(m)) = sqrt(sPk(:,ii(n),jj(m)).^2+sPk(:,ii2(n),jj(m)).^2);  end
        end
    end
    myfig; kk=1;
    for m=1:3
        for n=1:3
            subplot(3,3,kk); kk=kk+1;
            plot(t, sPk(:,ii(m),jj(n))/unit(jj(n)),'linewidth',2);  xygo(str{jj(n)});  title([xstr{m},'->',str{jj(n)}]);
        end
    end
