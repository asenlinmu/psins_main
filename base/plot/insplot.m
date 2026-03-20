function insplot(avp, ptype, varargin)
% avp plot.
%
% Prototype: insplot(avp, ptype)
% Inputs: avp - may be [att], [att,vn] or [att,vn,pos]
%         ptype - plot type define
%          
% See also  attplot, miniplot, inserrplot, kfplot, gpsplot, imuplot, magplot, dvlplot, addyawplot, tscaleset.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/10/2013, 09/11/2018
global glv
    t = avp(:,end)/glv.tscale(end);
    n = size(avp,2);
    if nargin<2
        if n==2,     ptype = 'y';  % yaw
        elseif n==3, ptype = 'l';  % lat-lon
        elseif n<5,	 ptype = 'a';
        elseif n<8,	 ptype = 'av';
        elseif n<11, ptype = 'avp';
        elseif n==11, ptype = 'avpo';
        elseif n<17, ptype = 'avped';
        elseif n<20, ptype = 'avpedl';
        else         ptype = 'avpedlt';
        end
    end
    %%
    switch ptype
        case 'y',
            myfigure;
            plot(t, avp(:,1)/glv.deg), xygo('y');
        case 'l',
            myfig;
            dxyz = pos2dxyz([avp(:,1:2),avp(:,1)*0]);
            plot(0, 0, 'rp');
            hold on, plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
        case 'A',
            myfig;
            plot(t, avp(:,1:2)/glv.deg), xygo('pr');
        case 'a',
            myfig;
            subplot(311), plot(t, avp(:,1)/glv.deg), xygo('p');
            subplot(312), plot(t, avp(:,2)/glv.deg), xygo('r');
            subplot(313), plot(t, avp(:,3)/glv.deg), xygo('y');
        case 'q',
            quat = q32q4(avp(:,1:3));
            avp(:,1:3) = q2attBatch(quat);
            insplot(avp, 'a');
        case 'q1',
            quat = q32q4(avp(:,1:3));
            avp(:,1:3) = q2att1Batch(quat);
            insplot(avp, 'a');
        case {'v','vn'}
            myfig;
            plot(t, avp(:,1:3)), xygo('V');
        case 'p',
            myfig;
            dxyz = pos2dxyz(avp(:,1:3));
            plot(t, dxyz(:,[2,1,3])); xygo('DP'); legend('\DeltaLat','\DeltaLon','\DeltaHgt');
        case 'av',
            myfigure;
            subplot(221), plot(t, avp(:,1:2)/glv.deg); xygo('pr');
            subplot(223), plot(t, avp(:,3)/glv.deg); xygo('y');
            subplot(222), plot(t, avp(:,4:5)); xygo('VEN');
            subplot(224), plot(t, avp(:,6)); xygo('VU');
        case 'vb',
            avp(:,4:6) = amulvBatch(avp(:,1:3),avp(:,4:6));
            if n<8, ptype='av';
            elseif n<11, ptype='avp';
            elseif n<17, ptype='avped'; end
            insplot(avp, ptype);
            if n<8, subplot(222); xygo('v_{x,y}^b / (m/s)'); subplot(224); xygo('v_z^b / (m/s)');
            else, subplot(323); xygo('V ^b / (m/s)'); end
        case 'avp',
            if size(avp,2)==9, t=1:length(t); end
            myfig;
            subplot(321), plot(t, avp(:,1:2)/glv.deg); xygo('pr'); legend('Pitch','Roll');
            subplot(322), plot(t, avp(:,3)/glv.deg); xygo('y'); legend('Yaw');
            % subplot(323), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V'); legend('V_E','V_N', 'V_U', '|V|');
            subplot(323), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2)]); xygo('V'); legend('V_E','V_N', 'V_U', 'V_G');
%             subplot(323), plot(t, avp(:,4:6)); xygo('V'); legend('V_E','V_N', 'V_U');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(325), plot(t, dxyz(:,[2,1,3])); xygo('DP'); legend('\DeltaLat','\DeltaLon','\DeltaHgt');
%             subplot(325), plot(t, [[avp(:,7)-avp(1,7),(avp(:,8)-avp(1,8))*cos(avp(1,7))]*glv.Re,avp(:,9)-avp(1,9)]); xygo('DP');
%             subplot(3,2,[4,6]), plot(r2d(avp(:,8)), r2d(avp(:,7))); xygo('lon', 'lat');
%                 hold on, plot(r2d(avp(1,8)), r2d(avp(1,7)), 'rp');
            subplot(3,2,[4,6]), plot(0, 0, 'rp');   % 19/04/2015
                hold on, plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
%                 hold on, plot((avp(:,8)-avp(1,8))*glv.Re*cos(avp(1,7)), (avp(:,7)-avp(1,7))*glv.Re); xygo('est', 'nth');
            legend(sprintf('LON0:%.2f, LAT0:%.2f (DMS), H0:%.1f (m)', r2dms(avp(1,8)),r2dms(avp(1,7)),avp(1,9)));
        case {'-h', 'avp-h'},  % no vU&hgt
            if size(avp,2)==9, t=1:length(t); end
            myfig;
            subplot(221), plot(t, avp(:,1:2)/glv.deg); xygo('pr'); legend('Pitch','Roll');
            subplot(223), plot(t, avp(:,3)/glv.deg); xygo('y'); legend('Yaw');
            subplot(222), plot(t, [avp(:,4:5)]); xygo('V'); legend('V_E','V_N');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(224), plot(t, dxyz(:,[2,1])); xygo('DP'); legend('\DeltaLat','\DeltaLon');
        case 'avp1',  % 1-by-1
            if size(avp,2)==9, t=1:length(t); end
            myfig;
            subplot(331), plot(t, avp(:,1)/glv.deg); xygo('p');
            subplot(332), plot(t, avp(:,2)/glv.deg); xygo('r');
            subplot(333), plot(t, avp(:,3)/glv.deg); xygo('y');
            subplot(334), plot(t, [avp(:,4)]); xygo('VE');
            subplot(335), plot(t, [avp(:,5)]); xygo('VN');
            subplot(336), plot(t, [avp(:,6)]); xygo('VU');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(337), plot(t, dxyz(:,1)); xygo('X');
            subplot(338), plot(t, dxyz(:,2)); xygo('Y');
            subplot(339), plot(t, dxyz(:,3)); xygo('Z');
        case 'Avp',
            if size(avp,2)==9, t=1:length(t); end
            myfig;
            subplot(321), ax=plotyy(t, avp(:,1)/glv.deg, t, avp(:,2)/glv.deg); xyygo(ax, 't', 'p', 'r');
            subplot(322), plot(t, avp(:,3)/glv.deg); xygo('y'); legend('Yaw');
            subplot(323), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+0*avp(:,6).^2)]); xygo('V'); legend('V_E','V_N', 'V_U', '|V|');
%             subplot(323), plot(t, avp(:,4:6)); xygo('V'); legend('V_E','V_N', 'V_U');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(325), plot(t, dxyz(:,[2,1,3])); xygo('DP'); legend('\DeltaLat','\DeltaLon','\DeltaHgt');
%             subplot(325), plot(t, [[avp(:,7)-avp(1,7),(avp(:,8)-avp(1,8))*cos(avp(1,7))]*glv.Re,avp(:,9)-avp(1,9)]); xygo('DP');
%             subplot(3,2,[4,6]), plot(r2d(avp(:,8)), r2d(avp(:,7))); xygo('lon', 'lat');
%                 hold on, plot(r2d(avp(1,8)), r2d(avp(1,7)), 'rp');
            subplot(3,2,[4,6]), plot(0, 0, 'rp');   % 19/04/2015
                hold on, plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
%                 hold on, plot((avp(:,8)-avp(1,8))*glv.Re*cos(avp(1,7)), (avp(:,7)-avp(1,7))*glv.Re); xygo('est', 'nth');
            legend(sprintf('LON0:%.2f, LAT0:%.2f (DMS), H0:%.1f (m)', r2dms(avp(1,8)),r2dms(avp(1,7)),avp(1,9)));
        case 'avpo',
            insplot(avp(:,[1:9,end]));
            if size(avp,2)~=11, dist = distance(avp(:,[7:9,end])); dist=dist(:,[1,end]); else, dist=avp(:,10:11); end
            subplot(3,2,5), plot(dist(:,2), dist(:,1), 'm');  legend('\DeltaLat','\DeltaLon','\DeltaHgt', 'Dist');  % plot OD distance
        case 'avpi', % for launch vehicle
            myfig,
            subplot(221), plot(avp(:,end), avp(:,1)/glv.deg), xygo('p');
            subplot(222), plot(avp(:,end), avp(:,2:3)/glv.deg), xygo('ry');
            subplot(223), plot(avp(:,end), avp(:,4:6)), xygo('V');  title('LCI  F/U/R')
            subplot(224), plot(avp(:,end), avp(:,7:9)), xygo('DP');
        case 'xyz', % for launch vehicle
            myfig,
            subplot(321), plot(avp(:,end), avp(:,1)/glv.deg), xygo('p');
            subplot(323), plot(avp(:,end), avp(:,2)/glv.deg), xygo('r');
            subplot(325), plot(avp(:,end), avp(:,3)/glv.deg), xygo('y');
            subplot(222), plot(avp(:,end), avp(:,4:6)), xygo('V');  legend('Vx','Vy','Vz');
            subplot(224), plot(avp(:,end), avp(:,7:9)), xygo('DP');  legend('X','Y','Z');
        case 'qvpi', % for launch vehicle
            quat = q32q4(avp(:,1:3));  avp(:,1:3) = q2attBatch(quat);
            insplot(avp,'avpi');
        case 'qvpi1', % for launch vehicle
            quat = q32q4(avp(:,1:3));  avp(:,1:3) = q2att1Batch(quat);
            insplot(avp,'avpi');
        case {'t/m', 't/h', 't/d'},  % AVP-plot where t-axis in day
            tscalepush(ptype);
            insplot([avp(:,1:9),avp(:,end)/tscaleget()],'avp');
            tscalepop();
        case 'ap',
            insplot([avp(:,1:3),zeros(length(avp),3),avp(:,4:end)],'avp');
            subplot(323), delete(gca); %cla;  xygo('N/A'); legend('');
        case 'vp',
            insplot([zeros(length(avp),3),avp],'avp');
        case 'vuh',  % plot VU & hgt
            if n>9, vuh=avp(:,[6,9,end]);   % avp input
            elseif n>6, vuh=avp(:,[3,6,end]); end  % vp input
            myfig;
            ax = plotyy(vuh(:,end), vuh(:,1), vuh(:,end), vuh(:,2));
            xlabel('t / s'); grid on;
            ylabel(ax(1), 'V_U / m/s'); ylabel(ax(2), 'h / m');
        case 'vnlat'
            if n>9, vnl=avp(:,[5,7,end]);   % avp input
            elseif n>6, vnl=avp(:,[2,4,end]); end  % vp input
            myfig;
            ax = plotyy(vnl(:,end), vnl(:,1), vnl(:,end), (vnl(:,2)-vnl(1,2))*glv.Re);
            xlabel('t / s'); grid on;
            ylabel(ax(1), 'V_N / m/s'); ylabel(ax(2), '\DeltaL / m');
        case 'avped'
            myfigure;
            subplot(321), plot(t, avp(:,1:2)/glv.deg); xygo('pr'); legend('Pitch','Roll');
            subplot(322), plot(t, avp(:,3)/glv.deg); xygo('y'); legend('Yaw');
            subplot(323), plot(t, [avp(:,4:6),normv(avp(:,4:6))]); xygo('V'); legend('V_E','V_N', 'V_U', '|V|');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(324), plot(t, dxyz(:,[2,1,3])); xygo('DP'); legend('\DeltaLat','\DeltaLon','\DeltaHgt');
%             subplot(324), plot(t, [[avp(:,7)-avp(1,7),(avp(:,8)-avp(1,8))*cos(avp(1,7))]*glv.Re,avp(:,9)-avp(1,9)]); xygo('DP');
            subplot(325), plot(t, avp(:,10:12)/glv.dph); xygo('eb'); legend('\epsilon_x','\epsilon_y','\epsilon_z');
            subplot(326), plot(t, avp(:,13:15)/glv.ug); xygo('db'); legend('\nabla_x','\nabla_y','\nabla_z');
        case 'avpedl'
            myfigure;
            subplot(421), plot(t, avp(:,1:2)/glv.deg); xygo('pr');
            subplot(422), plot(t, avp(:,3)/glv.deg); xygo('y');
            subplot(423), plot(t, [avp(:,4:6),normv(avp(:,4:6))]); xygo('V');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(424), plot(t, dxyz(:,[2,1,3])); xygo('DP');
            subplot(425), plot(t, avp(:,10:12)/glv.dph); xygo('eb');
            subplot(426), plot(t, avp(:,13:15)/glv.ug); xygo('db');
            subplot(427), plot(t, avp(:,16:18)); xygo('L');
        case 'avpedlt'
            insplot(avp, 'avpedl');
            subplot(428), plot(t, avp(:,19)); xygo('dT');
        case 'trjpy'  
            py = vn2att(avp(:,[4:6,end]));
            p1 = interp1(avp(:,end), avp(:,1), py(:,end));
            y1 = interp1(avp(:,end), avp(:,3), py(:,end));
            myfigure;
            subplot(321), plot(t, avp(:,1:2)/glv.deg, py(:,end), py(:,1)/glv.deg); xygo('pr'); legend('pitch','roll','pitch_{trj}');
            subplot(322), plot(t, avp(:,3)/glv.deg, py(:,end), py(:,3)/glv.deg); xygo('y'); legend('yaw','yaw_{trj}');
            subplot(323), plot(py(:,end), [p1-py(:,1)]/glv.deg); xygo('\delta\theta / ( \circ )'); legend('pitch_{trj}-pitch'); % attack angle
            subplot(324), plot(py(:,end), asin(sin(y1-py(:,3)))/glv.deg); xygo('\delta\psi / ( \circ )'); legend('yaw_{trj}-yaw'); % drift angle
            subplot(325), plot(t, [normv(avp(:,4:5)),avp(:,6)]); xygo('V'); legend('|V_{EN}|', 'V_U');
            subplot(326), plot(t, avp(:,4:5)); xygo('V'); legend('V_E', 'V_N');
        case 'DR',
            avptrue = setvals(varargin);
            myfigure;
            subplot(321), plot(t, avp(:,1:2)/glv.deg); xygo('pr');
            subplot(322), plot(t, avp(:,3)/glv.deg); xygo('y');
            subplot(323), plot(t, [avp(:,4:6),normv(avp(:,4:6))]); xygo('V');
            dxyz = pos2dxyz(avp(:,7:9));
            subplot(325), plot(t, dxyz(:,[2,1,3])); xygo('DP');
%             subplot(325), plot(t, [[avp(:,7)-avp(1,7),(avp(:,8)-avp(1,8))*cos(avp(1,7))]*glv.Re,avp(:,9)-avp(1,9)]); xygo('DP');
            subplot(3,2,[4,6]), plot(r2d(avp(:,8)), r2d(avp(:,7))); xygo('lon', 'lat');
                hold on, plot(r2d(avptrue(:,8)), r2d(avptrue(:,7)), 'r-');
                plot(r2d(avp(1,8)), r2d(avp(1,7)), 'rp');
            legend('DR trajectory', 'True trajectory', 'Location','Best');
    end
        