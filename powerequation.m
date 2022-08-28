function power_Eq = powerequation(xdelta) 



gamaVc=5.4; % 7.0 Bar
Pvdm_need=250*1000 ; % 125kW
delta_ref0=pi/8 ; % ר�����ڼ���discount��

%% VDM������ز���

%����Ч��
etaVc=0.65;  % VcЧ��	65%
etaVp=etaVc;  % VpЧ��=VcЧ��  �ڴ˰����У�ֻ�е�Ч�����ܱ�֤ת�ٵ������ɵ���

% syms k % ����һ��ϵͳ���ű������Ȼ����symsum�������ۼӵ�ʱ��Ҫ�õ�
RD=2.0;% wobble plate ��Ƭֱ��
Hmax=1.8;% H��NPS����󳤶�
S_c1=2.35*23;% ��һ����ͷ���pistion,��ʾ�Ѿ�discount/offset�˵�λת�������scale up/down�������ֿ���������2.0-2.6֮��
% �����������͸�ѹ���͸ף���ѹ�����ף����ѹ����
Npiston=23;
N_p_low=floor(Npiston/2); % ��ѹ��������Ŀ,������һ�룬����������
N_p_high=max(floor(N_p_low/2)-1,2); % ��ѹƽ�Ƹ�/��ѹ���͸���Ŀ���ǵ�ѹ���׵�1/3��������1/2*1/3��������������������������
N_p_retain=Npiston-N_p_low-N_p_high; % ���ѹ������Ŀ��ʣ����Ŀ��

% �ֱ����ѹ�����ͽǶ�
piedeg=2*pi()/Npiston; % ��һ��ռpie���ĽǶ�/����
ang1=piedeg*(N_p_retain+1); % ����ѹ����ang1�ĽǶ�
ang2=piedeg*(N_p_high-1); % �������ͽ�ang2�ĽǶ�
TqdL=5000;% ���ؼ���ͻ�ͼ����
VDMspdL=1200*2*pi()/60/(1-etaVc);% ����VDM�ת��1200rpm��Ӧ125rad/s

%% ������ز���
p0_c2=1;% �����ѹΪ1bar���Ѿ���S_c1=1��ʱ��offset�˵�λת���������⣬��Ϊp0.*S_c1����ͬʱ���ֵ�
etaT=.9999;  % TankЧ���ǲ���99.99%,��control���������ˣ�TankЧ��Ϊ��ѹ�ĺ���
press_Min=1.5;% operational pressure,����Ҫ��֤�İ�ȫ������ѹ����ȫ��ѹ
press_Max=8.0; % ��ȫ��ѹ  % gamaL=25;% ѹ���ʼ���ͻ�ͼ���ޣ�ò��vdm���ѹ����6.195����˲��ܴ���6�������޽��㡣�Ѿ���ߵ�8.9
VtankL=100000*2.5;% ��λ��������
press_Min=max(press_Min,1); % ��λ�����ѹ���趨����С��1

E_tankUpper=press_Max*VtankL*log(press_Max/p0_c2);% ���޶��������
E_tankLower=press_Min*VtankL*log(press_Max/p0_c2);% ���޶��������
E_tankL=E_tankUpper-E_tankLower;% ���޶�������㣬EtankL����������





            %-------------------------------------------
            % k2=etaVc���ǵ���̬���̣��ڹܱ�������һ���ʱ�򣬳��������
            gama_pu=gamaVc/(press_Max/p0_c2); %���㵱ǰ��ѹ�ı���ֵper unit����ǰ��ѹռ�����ѹ�Ķ���
%             preslossdiscount=0.4*sin(dlt)/sin(pi()/4);% ѹ��������ۿ�ϵ������ǰ���0.4���Լ�����            

            preslossdiscount=0.4*sin(xdelta)/sin(pi()/4);% ѹ��������ۿ�ϵ������ǰ���0.4���Լ�����
                %��ѹ��ת����1200rpm������1200round/min=1200round*6.28rad/round/60s=125rad/s
                VDMspdVc_ref=VDMspdL*(1-(1-gama_pu)*preslossdiscount)/2  *2*(1-etaVc);% ��������ֵ������ΪetaVc=65%���Ź��ʴ�����(1-etaVc)=35%ת�ٴ��������ת�ٵ�ʵ������1/2�㣬�˵�*2*35%����35%��
                VDMspdVp_ref=VDMspdL*(1-(1-gama_pu)*preslossdiscount)/2  *2*(1-etaVp);% ��������ֵ������ΪetaVp=65%���Ź��ʴ�����(1-etaVc)=35%ת�ٴ�
                %-------------------------------------------
                %air motorʱ������������ѹ���ش��ۿۣ�compressorʱ����ѹ�����������ش��ۿۡ�
                %��������㵽CVT������˿ڵģ�������ʱ�����yita����ʱ�����yita

                PvdmVc=VDMspdVc_ref*TqdVc_coef*tan(xdelta);            % ����compѹ�����ʣ����������������,���λת��ΪkW
                PvdmVp=VDMspdVp_ref*TqdVp_coef*tan(xdelta);            % ����exp���͹��ʣ����������������,���λת��ΪkW
                
                if Pvdm_need>=0
                    power_Eq=PvdmVc-Pvdm_need;
                else
                    power_Eq=PvdmVp-Pvdm_need;
                end
    end  