function [imu, ts, t0, bias, scale, mami_pos0] = imuzip(imu, sg, sa, ts, t0, pos0, fname)
% Zip IMU double-data to 0.5/1/2 bytes array, or reverse.
%
% Prototype: [imu, ts, t0, bias, scale, mami] = imuzip(imu, sg, sa, ts, t0, fname)
% Inputs: imu - IMU data input
%         sg - scale for gyro, [1/128,127]*arcsec, default 1 arcsec
%         sa - scale for acc, [1/128,127]*100ug*s, default 100ug*s
%         ts - sample interval, [1,127*127*127/1000=2048Hz]
%         t0 - IMU start time, [-38d,127^5*0.0001=3.3e6sec=38d]
%         pos0 - initial position [lat; lon; hgt]
%         fname - file name to write after zip
% Outputs: imu, ts, t0 - as input
%          bias - [-128,127]*(0.1*dps),(0.01g)
%          scale - [1/128,127]*(arcsec),(100ug*s)
%          mami_pos0 - max & min IMU interger value, [-8,7] for 0.5byte,
%                 [-128,127] for 1byte, [-32768,32767] for 2bytes;
%                 or initial position [lat; lon; hgt]
%
% Example:
%    imu0 = imustatic(avpset([10;20;30],0,glv.pos0), 0.01, 100, imuerrset(10000,1000,1,10));
%    [imu1, ts, t0, bias, scale, mm] = imuzip(imu0, 1/2, 1/2, 0.01, 1000, glv.pos0, 'imu.bin');
%    [imu1, ts, t0, bias, scale, mm] = imuzip(imu0, 1/2, 1/2, [], [], 'imu.bin');
%    [imu1, ts, t0, bias, scale, mm] = imuzip(imu0, 4, 1/8, [], 1000);
%    save imu1.mat imu1;  load imu1.mat;
%    [imu2, ts, t0, bias, scale, pos0] = imuzip(imu1);
%    imuplot(adddt(imu0,t0-ts),imu2); plotn(cumsum(imu2-adddt(imu0,t0-ts)));
%
% See also  imu2int16, imufile, mat2binfile, binfile.

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 07/11/2024
    arcdeg = pi/180; arcsec = arcdeg/3600;  g0 = 9.78;  dps = arcdeg/1; g0s = g0*1;
    if ~isinteger(imu) % zip
        if nargin<6, pos0=[]; end;  if isempty(pos0), pos0=[0;0;0]; end
        if nargin<5, t0=[]; end;  if isempty(t0), t0=imu(1,end); end
        if nargin<4, ts=[]; end;  if isempty(ts), ts=(imu(end,end)-imu(1,end))/(length(imu)-1); end
        frq001 = fix(1000/ts);  ts = 1000/frq001;  imu=imu(:,1:6);
        if nargin<3, sa=[]; end;  if isempty(sa), sa=1; end;  sa0=sa; if sa<1, sa=-1/sa; end
        if nargin<2, sg=[]; end;  if isempty(sg), sg=1; end;  sg0=sg; if sg<1, sg=-1/sg; end
        scale=[[1,1,1]*sg0*arcsec, [1,1,1]*sa0*0.0001*g0s];
        mn = mean(imu);
        bias = fix([mn(1:3)/(0.1*dps*ts), mn(4:6)/(0.01*g0*ts)]);
        b = [bias(1:3)*(0.1*dps*ts), bias(4:6)*(0.01*g0*ts)];
        for k=1:6, imu(:,k)=imu(:,k)-b(k); end
        imu = cumsum(imu);
        for k=1:6, imu(:,k) = imu(:,k)/scale(k); end; scale=scale([1,4]);
        imu = int32(diff([zeros(1,6,'int64'); int64(imu(:,1:6))]));
        s=0; t=t0; if t<0, t=-t; s=-1; end
        t=fix(t/0.0001); t06=mod(t,127); t=fix(t/127); t05=mod(t,127); t=fix(t/127); t04=mod(t,127); t=fix(t/127);
                         t03=mod(t,127); t=fix(t/127); t02=mod(t,127); t=fix(t/127); t01=mod(t,127);
        slat=0; lat=pos0(1); if lat<0, lat=-lat; slat=-1; end
        lat=fix(lat/(0.0025*arcsec)); lat04=mod(lat,127); lat=fix(lat/127); lat03=mod(lat,127); lat=fix(lat/127);
                                      lat02=mod(lat,127); lat=fix(lat/127); lat01=mod(lat,127);
        slon=0; lon=pos0(2); if lon<0, lon=-lon; slon=-1; end
        lon=fix(lon/(0.0025*arcsec)); lon04=mod(lon,127); lon=fix(lon/127); lon03=mod(lon,127); lon=fix(lon/127);
                                      lon02=mod(lon,127); lon=fix(lon/127); lon01=mod(lon,127);
        shgt=0; hgt=pos0(3); if hgt<0, hgt=-hgt; shgt=-1; end
        hgt=fix(hgt/0.01); hgt04=mod(hgt,127); hgt=fix(hgt/127); hgt03=mod(hgt,127); hgt=fix(hgt/127);
                           hgt02=mod(hgt,127); hgt=fix(hgt/127); hgt01=mod(hgt,127);
        % 1st row i1=[byte_flag, scale_gyro(arcsec), scale_acc(100ug), fs(0.001Hz,3bytes)]
        % 2st row i2=[t0(0.1ms,6bytes)]
        % 3nd row i3=[bias_gx, bias_gy, bias_gz,  bias_ax, bias_ay, bias_az]
        % 4-5 row i4i5=[lat,lon(0.0025arcsec), hgt(0.001m)](4bytes/each)
        f = frq001; f3 = mod(f,127); f = fix(f/127);
        i1 = int16([1, sg, sa, fix(f/127), mod(f,127), f3]);  intstr = 'int8';
        i2 = int16([s*128+t01, t02, t03, t04, t05, t06]);
        i3 = int16(bias);
        i4 = int16([slat*128+lat01, lat02, lat03, lat04, slon*128+lon01, lon02]);
        i5 = int16([lon03, lon04, shgt*128+hgt01, hgt02, hgt03, hgt04]);
        mami = [max(max(imu(:,1:3))), min(min(imu(:,1:3))), max(max(imu(:,4:6))), min(min(imu(:,4:6)))];
        if isempty(find(bias>127|bias<-128,1)) && mami(1)<=127 && mami(2)>=-128 && mami(3)<=127 && mami(4)>=-128
            if mami(1)<=7 && mami(2)>=-8 && mami(3)<=7 && mami(4)>=-8  % 0.5byte(4bits)
                intstr = 'uint8';  i1(1) = 0;
                imu=imu+8; imu=imu(1:2:end,:)*16+imu(2:2:end,:);
                imu = [uint8([i1; i2; i3; i4; i5]+128); uint8(imu)];
            else  % 1byte
                imu = [int8([i1; i2; i3; i4; i5]); int8(imu)];
            end
        else
            intstr = 'int16';  i1(1) = 2*256;  % NOTE: byte_flag==1 for int8, ==2 for int16
            imu = [[i1; i2; i3; i4; i5]; int16(imu)];
        end
        if exist('fname','var'),
            if strcmp(fname(end-3:end),'.mat')==1
                save(fname, 'imu');
            else
                fid=fopen(fname,'w'); fwrite(fid, imu', intstr); fclose(fid);
                % mat2intfile(fname, imu);
            end
        end
        mami_pos0 = mami;
    else % unzip
        % 1st row i1=[byte_flag, scale_gyro(arcsec), scale_acc(100ug), fs(0.001Hz,3bytes)]
        % 2st row i2=[t0(0.1ms,6bytes)]
        % 3nd row i3=[bias_gx, bias_gy, bias_gz,  bias_ax, bias_ay, bias_az]
        % 4-5 row i4i5=[lat,lon(0.0025arcsec=0.07m), hgt(0.01m)](4bytes/each)
        iii = imu(1:5,:);  if iii(1,1)==128, iii=int8(int16(iii)-128); end
        i1 = double(iii(1,:));  % byte_flag=i1(1); be used in C/C++
        sg=i1(2); if sg<0, sg=-1/sg; end;
        sa=i1(3); if sa<0, sa=-1/sa; end;
        scale=[[1,1,1]*sg*arcsec, [1,1,1]*sa*0.0001*g0s];
        ts=1000/(i1(4)*127^2+i1(5)*127+i1(6));
        i2 = double(iii(2,:));
        s=1; if i2(1)<0, i2(1)=i2(1)+128; s=-1; end
        t0=s*(i2(1)*127^5+i2(2)*127^4+i2(3)*127^3+i2(4)*127^2+i2(5)*127^1+i2(6))*0.0001;  % 0.1ms
        i3 = double(iii(3,:));  b=[i3(1:3)*0.1*dps*ts, i3(4:6)*0.01*g0*ts];  bias=iii(3,:);
        i4 = double(iii(4,:));  i5 = double(iii(5,:));
        s=1; if i4(1)<0, i4(1)=i4(1)+128; s=-1; end
        lat = s*(i4(1)*127^3+i4(2)*127^2+i4(3)*127^1+i4(4));
        s=1; if i4(5)<0, i4(5)=i4(5)+128; s=-1; end
        lon = s*(i4(5)*127^3+i4(6)*127^2+i5(1)*127^1+i5(2));
        s=1; if i5(3)<0, i5(3)=i5(3)+128; s=-1; end
        hgt = s*(i5(3)*127^3+i5(4)*127^2+i5(5)*127^1+i5(6));
        mami_pos0 = [[lat; lon]*0.0025*arcsec; hgt*0.01];
        imu = imu(6:end,:);
        if i1(1)==0 % 0.5byte
              imu2=mod(imu,16);  imu1=(imu-imu2)/16;
              imu=zeros(length(imu)*2,6,'uint8'); imu(1:2:end,:)=imu1; imu(2:2:end,:)=imu2;
              imu=int8(imu)-8;
        end
        imu = double(imu);
        for k=1:6, imu(:,k)=imu(:,k)*scale(k)+b(k); end; scale=scale([1,4]);
        imu = [imu,t0+(0:length(imu)-1)'*ts];
    end
    