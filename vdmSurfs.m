%% �˺�����������VDM�ĸ��ֹ�������
% ���غ͹����Ѿ�ת����gearbox��, �ȴ�vdmcall.m�ĵ���
% ��ģ�ͺ�г��Ч�ʺ�����ת�ٵ�ͳһ, �����˹������棬�Լ���Ӧ����ת����ѹ����


%% ��������
% clc
% clear all
% close all
%% VDM������ز���

% %����Ч��
% etaVc = 0.65;  % VcЧ��	65%
% etaVp = etaVc;  % VpЧ��=VcЧ��, �ڴ˰�����ֻ�е�Ч���ܱ�֤ת�ٵ������ɵ���

% syms k % ����һ��ϵͳ���ű���, ����symsum�������ۼӵ�ʱ��Ҫ�õ�

N_p_low = floor(Npiston/2); % ��ѹ��������Ŀ, ������һ��, ��������
N_p_high = max(floor(N_p_low/2)-1, 2); % ��ѹƽ�Ƹ�/��ѹ���͸���Ŀ, �ǵ�ѹ���׵�1/3, ������1/2*1/3, ������������, ��������
N_p_retain = Npiston - N_p_low - N_p_high; % ���ѹ������Ŀ, ʣ����Ŀ

% �ֱ����ѹ�����ͽǶ�
piedeg = 2*pi()/Npiston; % ��һ��ռpie���ĽǶ�/����
ang1 = piedeg*(N_p_retain+1); % ����ѹ����ang1�ĽǶ�
ang2 = piedeg*(N_p_high-1); % �������ͽ�ang2�ĽǶ�


%% ������ز���
press_Min = max(press_Min,1); % ��λ�����ѹ���趨����С��1
E_tankUpper = press_Max*VtankL*log(press_Max/p0_c2); % ���޶��������
E_tankLower = press_Min*VtankL*log(press_Min/p0_c2); % ���޶��������
E_tankL = E_tankUpper - E_tankLower; % ���޶��������, EtankL����������

%% �Ը�����Ͻ�������
dltrange = 0:pi/4/200:pi/4;   % ����x��delta��ɨ�跶Χ Wobble angle
Hrange   = 0:Hmax/400:Hmax;   % ����y�� H  ��ɨ�跶Χ Netrol piston shift

dltindx = 0; % wobble angle ��ʼ�������
for dlt = dltrange  % wobble angle 0��-45��
    dltindx = dltindx+1;  % wobble angle ����ۼ�
    Hindx = 0;% NPS ��ʼ�������
    
    % ��ȫɨ���
    srangeIndx = find(Hrange <= RD*sin(dlt));% ͨ����������, �ҵ������и��ӵı�Ų����и�ֵ
    for  H = Hrange(srangeIndx)
    Hindx = Hindx + 1; % NPS ����ۼ�
            gama(Hindx,dltindx) = NaN;
            % Vc torque speed power��ֵ
            TqdVc(Hindx,dltindx) = NaN;
            VDMspdVc(Hindx,dltindx) = NaN;
            PvdmVc(Hindx,dltindx) = NaN;
            % Vp torque speed power��ֵ
            TqdVp(Hindx,dltindx) = NaN;
            VDMspdVp(Hindx,dltindx) = NaN;
            PvdmVp(Hindx,dltindx) = NaN;
    end
    
    srangeInd = find(Hrange > RD*sin(dlt)) ; % find �������������������λ�ú�
    for  H = Hrange(srangeInd) % ƫ����0-���ֵ20
    Hindx = Hindx + 1; % NPS ����ۼ�
            V1 = H + RD*sin(dlt); % ��ʼ���, ��ʼ�Ƕ�Ϊ0, cos��0��=1
            V2 = H + RD*sin(dlt)*cos(ang1); % ��̬���
            % gama(Hindx,dltindx)=min((V1/V2),(press_Max/p0_c2)); % ����ѹ����, ���Ҽ���λ
            % ȥ��ѹ������λ, ����gama�ļ���ֵ��H, dlt��������Ӧ
            gama(Hindx,dltindx) = (V1/V2); % ����ѹ����, ���Ҽ���λ    
            
            %-------------------------------------------
            % k2=etaVc���ǵ���̬����, �ڹܱ�������һ���ʱ��, ���������
            gama_pu = gama(Hindx,dltindx)/(press_Max/p0_c2); %���㵱ǰ��ѹ�ı���ֵper unit
            preslossdiscount = 0.4*sin(dlt)/sin(pi()/4);% ѹ��������ۿ�ϵ��, ��ǰ���0.4���Լ�����
            
            % ��ѹ��ת����1200rpm������1200round/min=1200round*6.28rad/round/60s=125rad/s
            VDMspdVc(Hindx,dltindx) = VDMspdL*(1-(1-gama_pu)*preslossdiscount)/2*2*(1-etaVc); % ��������ֵ, ����ΪetaVc=65%���Ź��ʴ�, ��(1-etaVc)=35%ת�ٴ�
            VDMspdVp(Hindx,dltindx) = VDMspdL*(1-(1-gama_pu)*preslossdiscount)/2*2*(1-etaVp); % ��������ֵ, ����ΪetaVp=65%���Ź��ʴ�, ��(1-etaVc)=35%ת�ٴ�
            
            %-------------------------------------------
            Tqfun1 = @(z) (H + RD.*sin(dlt))./(H + RD.*sin(dlt).*cos(z)).*p0_c2.*S_c1.*RD.*tan(dlt).*sin(z);
            Tqfun2 = @(z) gama(Hindx,dltindx)                        *p0_c2 *S_c1 *RD *tan(dlt) *sin(z);
            Tqd1 = integral(Tqfun1,0,ang1); % compression���ػ���
            Tqd2 = integral(Tqfun2,ang1,ang1+ang2); % exhaust���ػ���
            Tqd_sum_ideal = (Tqd1+Tqd2)/2/pi*Npiston; % �����ϵ���������, �ϼ�ֵ
            
            %air motorʱ����������ѹ���ش��ۿ�; compressorʱ��ѹ�����������ش��ۿ�, ������ʱ����yita, ��ʱ����yita
            TqdVc(Hindx, dltindx) =  min(Tqd_sum_ideal/etaVc,TqdL); % ����comprΪ������, ������ΪVDM����ӳ�������ߵ�����
            PvdmVc(Hindx, dltindx) = VDMspdVc(Hindx, dltindx)*TqdVc(Hindx,dltindx); % ����compѹ�����ʣ����������������, ���λת��ΪkW
            TqdVp(Hindx, dltindx) = -min(Tqd_sum_ideal*etaVp,TqdL); % ����expΪ������, ������ΪVDM������͵�����������
            PvdmVp(Hindx, dltindx) = VDMspdVp(Hindx,dltindx)*TqdVp(Hindx,dltindx); % ����exp���͹���, ���������������, ���λת��ΪkW
    end

end

        % max(max(TqdVc,[],1),[],2)
        % TqdL
            Pvdm_suply_negMax = 1.2*min(min(PvdmVp,[],1),[],2); % ����PvdmVp����������, Ҳ������һ��ѹ���������expander����, �Ǹ���
            
if VDM_curve_option %% VDM ��������ͼ
    Pvdm_unit= max(max(PvdmVc,[],1),[],2);
    gama_unit= max(max(gama,[],1),[],2);
    
    figure % ������ά����
    subplot(2,1,1)  % ���� VDM power����
    surf(dltrange, Hrange, 10*PvdmVc/1e3,'edgecolor','none') % 10 & 1e3 is just for Science Paper (2019/04/09) 
    xlabel('\delta (rad)')
    ylabel('H (m)')
    zlabel('P^{Vc} (kW)')
    view([-60 30])
    title('VDM compression power surface')
   
    subplot(2,1,2) %���� VDM power�ȸ���
    contour(dltrange,Hrange,10*PvdmVc/1e3,'ShowText','on') % 10 & 1e3 is just for Science Paper (2019/04/09) 
    title('VDM compression power contour')
    colorbar
    xlabel('\delta (rad)')
    ylabel('H (m)')

    figure 
    subplot(2,1,1) % ����gama����
    surf(dltrange, Hrange, gama,'edgecolor','none') 
    xlabel('\delta (rad)')
    ylabel('H (m)')
    zlabel('\gamma^{Vc}')
    view([-60 30])
    title('Compression ratio surface')

    subplot(2,1,2) % ����gama �ȸ���             
    contour(dltrange, Hrange, gama,'ShowText','on') 
    title('Compression ratio contour')
    colorbar
    xlabel('\delta (rad)')
    ylabel('H (m)')

    figure   % ����meshgrid
    contour(dltrange, Hrange, PvdmVc/Pvdm_unit,'r--','ShowText','on')
    set(gcf,'renderer','zbuffer');
    hold on
    contour(dltrange, Hrange, gama/gama_unit,'linewidth',1,'ShowText','on')
    title('Torque contour')
    hold off
    xlabel('\delta (rad)')
    ylabel('H (m)')
end
