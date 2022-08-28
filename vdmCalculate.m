% ����ȡ��vdmSurf��vdmcall
% ����ⷽ�̵ķ�ʽ���õ�delta��H
% �����ͱ���ȫ����Դ��vdmSurf�Լ�������ӿ�

%% ��������
% clc
% clear all
% close all
% gamaVc=pres_gama_Ratio; % 7.0 Bar
for i=1:15 % ��loop��Ϊ�˼����ڸ�����ѹ�£�����call���ʵķ���ֵ
gamaVc=0.5*i+0.5; % ������ѹ7.0 Bar% ԭ���� gamaVc=7.0����������ѹ��ķ��ع���
Pvdm_need=-250*1000 ; % �������� 125kW
delta_ref0=pi/8 ; % ר�����ڼ���Tq ��Ϊ�������discount��Ԥ���

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

%% ����
k1=RD.*(cos(ang1)*gamaVc-1)/(1-gamaVc);% ����gama����Ĳ���k1��H=k1*sin��delta��
k2=p0_c2.*S_c1.*RD;% ����tq����Ĳ���k2����ϵ��

Tqd1_coef = -k2*(k1+RD)/RD*(log(abs(k1+RD*cos(ang1)))-log(abs(k1+RD*cos(0)))); %close compression ���ֻ��ֽ��
%Tq1 ���ֽ��ln|k1+RD*cos(ang1)|
Tqd2_coef = -gamaVc*k2*(cos(ang1+ang2)-cos(ang1)); %open ejection���ֻ��ֽ��
%Tq2 ���ֽ��cos(ang1+ang2)
Tqd_sum_ideal_coef = (Tqd1_coef+Tqd2_coef)/2/pi*Npiston; % �����ϵ��������أ��ϼ�ֵ
% ���ֺ���2piƽ����Ȼ�����������

%air motorʱ������������ѹ���ش��ۿۣ�compressorʱ����ѹ�����������ش��ۿۡ�
%��������㵽CVT������˿ڵģ�������ʱ�����yita����ʱ�����yita
TqdVc_coef =  min(Tqd_sum_ideal_coef/etaVc,TqdL);  % ����comprΪ�����أ�������ΪVDM����ӳ�������ߵ�����
TqdVp_coef = -min(Tqd_sum_ideal_coef*etaVp,TqdL);  % ����expΪ�����أ�������ΪVDM������͵�����������

% ��Ϊdelta_ref1Ҳ��Ӱ�쵽���������Ի�Ӱ��speed��ȡֵ��������һ����ϵ�����̡�
% preslossdiscount=@(xdelta)  0.4*sin(xdelta*0+delta_ref0)/sin(pi()/4);% ѹ��������ۿ�ϵ������ǰ���0.4���Լ�����
preslossdiscount1=@(xdelta)  0.4*sin(xdelta)/sin(pi()/4);% ѹ��������ۿ�ϵ������ǰ���0.4���Լ�����
%��ѹ��ת����1200rpm������1200round/min=1200round*6.28rad/round/60s=125rad/s
gama_pu1=gamaVc/(press_Max/p0_c2); %���㵱ǰ��ѹ�ı���ֵper unit����ǰ��ѹռ�����ѹ�Ķ���
% ��������ֵ������ΪetaVc=65%���Ź��ʴ�����(1-etaVc)=35%ת�ٴ��������ת�ٵ�ʵ������1/2�㣬�˵�*2*35%����35%��
VDMspdVc_ref=@(xdelta)  VDMspdL*(1-(1-gama_pu1)*preslossdiscount1(xdelta))/2  *2*(1-etaVc);
VDMspdVp_ref=@(xdelta)  VDMspdL*(1-(1-gama_pu1)*preslossdiscount1(xdelta))/2  *2*(1-etaVp);

PvdmVc1=@(xdelta)  VDMspdVc_ref(xdelta)*TqdVc_coef*tan(xdelta);   % ����compѹ�����ʣ����������������,���λת��ΪkW
PvdmVp1=@(xdelta)  VDMspdVp_ref(xdelta)*TqdVp_coef*tan(xdelta);   % ����exp���͹��ʣ����������������,���λת��ΪkW

if Pvdm_need>=0
    power_Eq=@(xdelta) PvdmVc1(xdelta)-Pvdm_need;
else
    power_Eq=@(xdelta) PvdmVp1(xdelta)-Pvdm_need;
end

dlt_ref1=fzero(power_Eq,delta_ref0); % fzero �޷�ͨ�����ݸ����̸�ֵ��

dlt_ref1    =  max(min(dlt_ref1,pi/4),-pi/4); % ��delta��λ
H_ref1      =  k1* sin(dlt_ref1); %������� H
Pvdm_ref1   = (Pvdm_need>=0)*PvdmVc1(dlt_ref1)+(Pvdm_need<0)*PvdmVp1(dlt_ref1); % ���㱾��Pvdm_ref
VDMspd_ref1 = (Pvdm_need>=0)*VDMspdVc_ref(dlt_ref1)+(Pvdm_need<0)*VDMspdVc_ref(dlt_ref1); % ѡ��ת��
Tqd_ref1    =  Pvdm_ref1/VDMspd_ref1 ; % ��������

% �����¼���ع��ʣ���Ҫ��¼���ع��ʶ�Ӧ��ת�ٺ�����
Vrecord_MaxP_Tq(i,:) = [gamaVc,Pvdm_ref1,VDMspd_ref1,Tqd_ref1,dlt_ref1,H_ref1];
end

%% ����ֵ���ֵķ�ʽ���Ա�����(����֤�ɹ������ֹ�ʽ�������)
% % ���㱾�����е�����
%     Tqd_sum_ideal1=Tqd_sum_ideal_coef*tan(dlt_ref1);
% % ��Ϊת�������ȶ��õģ����ؼ���tq��ʱ�򣬲�care�Ƕ�
% % ʵ���ϣ�delta�Ƕ�Ҳ����������������ʵ����صģ����ǲ��û��֣�
%     Tqfun1 = @(z) (H_ref1+RD.*sin(dlt_ref1))./(H_ref1+RD.*sin(dlt_ref1).*cos(z)).*k2.*tan(dlt_ref1).*sin(z);
%     Tqfun2 = @(z) gamaVc                                                         *k2 *tan(dlt_ref1) *sin(z);
%     Tqd1_integral = integral(Tqfun1,0,ang1); %compression���ػ���
%     Tqd2_integral = integral(Tqfun2,ang1,ang1+ang2); %exhaust���ػ���
%     Tqd_sum_ideal_integral = (Tqd1_integral+Tqd2_integral)/2/pi*Npiston; % �����ϵ��������أ��ϼ�ֵ
