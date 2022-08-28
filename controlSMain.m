%% ���ܣ����ø������߲��Կ��Ʋ����Ƿ�֧��CAWTʵ�ֿɵ�����

clc
clear all
close all

% ��ͼ����
Cp_curve_option = 1;
VDM_curve_option = 1;

%% Part 1 �趨�ڲ�����: Blade��Generator��VDM��Air tank
%% Part 1-1 Blade 
run  bladeSurfs.m
vin = 3.0;      % cut-in speed
v1_rated = 6.5; % elec-rated speed
voff = 15;      % cut-out speed 13.5 ���15
voff_Cp_tail_eff = voff; % =13.5 Cp β������, �������ܲ�����, ��ͨ����β��������Ϊ��������

%% Part 1-2 Generator 
etaG = 0.99;    % ���Ч��    
etaX = 1;       % gearbox Ч�� 
etaVc = 0.65;   % VcЧ��	65%
etaVp = etaVc;  % VpЧ�� = VcЧ��, ֻ�е�Ч���ܱ�֤ת�ٵ������ɵ���
   
Pgen_rated = 250*1000; % ����� 250kW 
Pvdmin_rated = Pgen_rated/etaG/etaX/etaVc; % VDM �����
Pblade_rated = Pgen_rated/etaG/etaX + Pvdmin_rated; % ��е�����
    
aai = Pgen_rated/etaG/etaX/(v1_rated^3-vin^3); % aii=0.5*rho*Cp*AW
bbi = aai*vin^3; % ��ֵ����floss, ��Ϊ cut-in speed
v2_rated = ((Pblade_rated + bbi)/aai)^(1/3); % mech-rated speed, �ɸ���VDM����빦���Զ�����

P_speed_power = @(z) max(0, aai*z^3 - bbi); % ���ٵ����ܺ���
%  floss  = P_speed_power(vin);

Cp_max = max(optCp); % Cp ���ֵ-������pitchʱ�������
AW = aai/(0.5*1.225*Cp_max);  % 1/2*rho*Cp*Aw  *V3==ai  *V3
radius_Balde = sqrt(AW/pi()); % m

%% Part 1-3 VDM
RD = 2.0; % wobble plate ��Ƭֱ��(m)
Hmax = 1.8; % H��NPS����󳤶�(m)
S_c1 = 54; % ��һ����ͷ���pistion, ��ʾ�Ѿ�discount/offset�˵�λת�������scale up/down, ��������50֮��
Npiston = 23; % ����������, ��ѹ���͸�, ��ѹ������, ���ѹ����

TqdL = 5000;% ���ؼ���ͻ�ͼ����
VDMspdL = 1200*2*pi()/60/(1-etaVc);% ����VDM�ת��1200rpm��Ӧ125rad/s

%% Part 2-4 Air tank
p0_c2 = 1; % �����ѹΪ1bar, �Ѿ���S_c1=1ʱoffset�˵�λת���������⣨p0.*S_c1����ͬʱ���֣�
etaT = .9999;  % TankЧ��, Ҳ������Ϊ��ѹ�ĺ���
press_Min = 3.0; %2.9152; % operational pressure,����Ҫ��֤�İ�ȫ������ѹ����ȫ��ѹ1.5 ����©ֵΪ2.9152 % unit Ϊ Bar
press_Max = 8.0; % ��ȫ��ѹ  % gamaL=25;% ѹ���ʼ���ͻ�ͼ���ޣ�ò��vdm���ѹ����6.195����˲��ܴ���6�������޽��㡣�Ѿ���ߵ�8.9
VtankL = 100000*2.5; % ��λ��m^3
 
run vdmSurfs.m

%% Part 2 �趨�ⲿ����: ���������������ָ��
%% Part 1-1 ������ wind speed
% ������Է���
% V_wind_serial=[5.2 5.8 6.7 7.2 7.8 8.9 9.2 7.9 7.2 6.4 5.9 7.7 9.0 8.4 6.9 5.5 4.5 3.8 3.3 4.0 4.8 5.6 6.3 6.6];
% P_load_serial=[100 220 200 140 250 250 250 170 200 250 240 190 150 150 200 250 230 200 180 250 210 150 120 100]*1000;

% ��sheet4������, д�뵽sheet2
V_wind_serial= xlsread('controlSTransR1.xlsx','Sheet4','D2:D1009')';% 'D2:D1009'
N_V_wind = 1008; % 24*7 or 1008=24*6*7��ÿʮ����һ�����ݵ�
% ������ʵ����
% V_wind_serial=[10.26	9.2	7.61	6.69	5.06	6.25	7.18	8.36	9.98	9.5	10.39	8.97...
%     7.81	6.63	5.53	4.65	4.18	3.55	4.89	4.73	3.62	4.58	5.71	7.02];
% ���ٺ�load������λ
V_wind_serial = V_wind_serial(1:N_V_wind); % ��ȡ���г���

%% Part 2-2 ����ָ�� load demand
LoadScale = 1.3; 
P_load_serial= xlsread('controlSTransR1.xlsx','Sheet4','E2:E1009')'*1000; % 'H4:H171'��ʾ250kW������
P_load_serial = LoadScale*P_load_serial(1:N_V_wind); % ��ȡ���г���
% V_wind_serial=min(max(V_wind_serial,vin),voff); % trim V wind speed ��ȥ��ֵ
P_load_serial = min(max(P_load_serial,0), Pgen_rated); % trim Pload ��ȥ��ֵ


%% Part 3 ������ʼ��ѹ, ����, Hֵ, �õ� P_vdm need �趨����
pres_gama_Ratio = 0.8*press_Max; % �趨ϵͳ����ǰ��ʼ��ѹ, ͬʱpres_gama_Ratio*p0�ڷ���ѭ����Ҳ�᲻�ϸ���bar
E_tank = pres_gama_Ratio * p0_c2 * VtankL * log(pres_gama_Ratio)-E_tankLower; % ���㵱ǰ��ѹ����, Etank����������
T_step = 1/6;

Hindx_ref = 0;
VDM_gentleness_coef = 0.25; % ϵ�����ڿ��� VDM �ڽӽ����༫����ѹʱ�Ķ������ͳ̶�,
                            % �����������������ٲ�׽����, �׶��ǹ��ʱ仯̫���Ҳ������͡�
% ��ʼ��¼�ֵ
CapRecords = [vin v1_rated v2_rated voff radius_Balde Pblade_rated/1000 Pvdmin_rated/1000 Pgen_rated/1000 press_Max press_Min  E_tankL/1000 VtankL/1000 E_tank/1000];

%% Part 4 ���з���, ��ȡ��ʱ�����в���
wind_indx = 0;

for V_wind = V_wind_serial
    wind_indx = wind_indx + 1;
    
    P_load_ref = min(Pgen_rated, P_load_serial(wind_indx)); % ����power load dispatch��Ԥ��ֵ, �Ѿ�trim��������gen����
    Pblade_cal = min(P_speed_power(V_wind), Pblade_rated); % �������λ��blade power curve

    % Pvdm_gap=Pblade_cal-Pgen_rated/etaG; % ԭʼ����Ŀ����Pgen_rated
    Pvdm_gap = Pblade_cal - P_load_ref/etaG; % ��ǰ����Ŀ��Ϊ Pgen_rated, P_load_ref
    % etaT=1.0;  % TankЧ�ʺ㶨
    etaT = (2000 - pres_gama_Ratio*T_step)/2000;  % TankЧ��Ϊ��ѹ�ĺ���; ����ѹΪ7, ����Ϊ0.2, ��ʧΪǧ��֮0.7, ÿСʱ��ʧΪǧ��֮3.5, 24Сʱ��ʧΪ8.4%
    % Pvdmneed=(Pvdm_gap>=0)*min([Pvdm_gap,E_tankL-etaT*E_tank,Pvdmin_rated])+(Pvdm_gap<0)*max([Pvdm_gap,-etaT*E_tank,-Pvdmin_rated]); 
    % Pvdmneed=(Pvdm_gap>=0)*min([Pvdm_gap,((E_tankL+E_Tank_tolerance)-etaT*E_tank),Pvdmin_rated])+(Pvdm_gap<0)*max([Pvdm_gap,-etaT*E_tank,-Pvdmin_rated]); 
    
    % ���Ч��ϵ��, ������ȫ��ת���������
    Pvdmneed = (Pvdm_gap>=0)*min([Pvdm_gap, (E_tankL-etaT*E_tank)/etaVc/T_step*VDM_gentleness_coef, Pvdmin_rated]) +...
               (Pvdm_gap<0)*max([Pvdm_gap, -etaT*E_tank*etaVp/T_step*min(1,VDM_gentleness_coef+0.5), -Pvdmin_rated]); 
    % ��P_vdm����0, ����������ʣ������; ��P_vdmС��0, �����������Ѵ�������; ���������������߽� (���������߽�, vdm���ʱ߽�), gap��needһ��
    % T_step��Ϊ�˽���λͳһ�����ʲ�
    % �����ٽ��벻��������, ������ȫ��VDM���е�, P_vdm_need ʵ����ȫ�����¼���, load powerȫ����VDM����
    Pvdmneed = -(1-((V_wind >= vin) & (V_wind <= voff)))* P_load_ref/etaG + ((V_wind >= vin) & (V_wind <= voff))* Pvdmneed;

    % ��ʼͨ��Scall.m, ѭ��ʹ��Surfs.m�ļ�
    run vdmScall.m
    % ����VDMʵ���ܹ�ʵ�ֵ����ݼ�VDM���в���
    run bladeScall.m
    % ����blade, tank, CVT, gen�����в���
    run vdmRescall.m

    % ��¼����ֵ
    OprRecords(wind_indx,:) = [V_wind, Pblade_cal/1000, Pvdm_gap/1000, Pvdmneed/1000, Pblade_ref/1000, Pvdm_ref/1000,  Pgenin_ref*etaG/1000, P_load_ref/1000, E_tank/1000, ...
                               CVT_Ratio_ref, gama_ref, pres_gama_Ratio,  ...
                               Cp_ref, beta_ref, numda_ref, H_ref,dlt_ref, ...
                               omg_shaft1_ref,Tq_gen_ref,VDMspd_ref, Tqd_ref, P_spin_Resv/1000 ];
end

% �����д��excel������
xlswrite('controlSTransR1.xlsx','   ','sheet2','B4:B1011')
xlswrite('controlSTransR1.xlsx', CapRecords,'sheet2','B2')
xlswrite('controlSTransR1.xlsx', OprRecords,'sheet2','B4')
disp('Done writing')
winopen('controlSTransR1.xlsx')

