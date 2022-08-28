vin = 3.0;      % cut-in speed
v1_rated = 6.5; % elec-rated speed
voff = 15;      % cut-out speed 13.5 ���15
voff_Cp_tail_eff = voff; % =13.5 Cp β������, �������ܲ�����, �Ѿ�ͨ����β��������Ϊ��������

%% Part 2-2 Generator 
etaG = 0.99;    % ���Ч��    
etaX = 1;       % gearbox Ч�� 
etaVc = 0.65;   % VcЧ��	65%
etaVp = etaVc;  % VpЧ�� = VcЧ��  �ڴ˰����У�ֻ�е�Ч���ܱ�֤ת�ٵ������ɵ���
   
Pgen_rated = 250*1000; % ����� 250kW 
Pvdmin_rated = Pgen_rated/etaG/etaX/etaVc; % VDM �����
Pblade_rated = Pgen_rated/etaG/etaX + Pvdmin_rated; % ��е�����
    
aai = Pgen_rated/etaG/etaX/(v1_rated^3-vin^3); %aii=0.5*rho*Cp*AW
bbi = aai*vin^3; % ���ֵ�͵���floss, ��Ϊ cut-in speed
v2_rated = ((Pblade_rated + bbi)/aai)^(1/3); % mech-rated speed �ɸ����Զ� vdm ����빦�ʼ����

P_speed_power = @(z) max(0, aai*z^3 - bbi);

V_wind_serial= xlsread('controlSTransR1.xlsx','Sheet4','D2:D1009')';% 'D2:D1009'
N_V_wind = 1008; % 24*7 or 1008=24*6*7��ÿʮ����һ�����ݵ�
% ������ʵ����
% V_wind_serial=[10.26	9.2	7.61	6.69	5.06	6.25	7.18	8.36	9.98	9.5	10.39	8.97...
%     7.81	6.63	5.53	4.65	4.18	3.55	4.89	4.73	3.62	4.58	5.71	7.02];
% ���ٺ�load������λ
V_wind_serial = V_wind_serial(1:N_V_wind); % ��ȡ���г���
LoadScale = 1.3;
P_load_serial= xlsread('controlSTransR1.xlsx','Sheet4','E2:E1009')'*1000; % 'H4:H171'��ʾ250kW������
P_load_serial = LoadScale*P_load_serial(1:N_V_wind); % ��ȡ���г���
% V_wind_serial=min(max(V_wind_serial,vin),voff);% trim V wind speed ��ȥ��ֵ
P_load_serial = min(max(P_load_serial,0), Pgen_rated); % trim Pload ��ȥ��ֵ
P_load = P_load_serial;
for Ind = 1:N_V_wind
    WT(Ind) = P_speed_power(V_wind_serial(Ind));
    WTCur(Ind) = WT(Ind);
    if WT(Ind) >= Pgen_rated
       WT(Ind) = Pgen_rated;
    end
    if WTCur(Ind) > P_load_serial(Ind)
        WTCur(Ind) = P_load_serial(Ind);
    end
end
loadcoverage = sum(WTCur)/sum(P_load)
save WT.mat WT WTCur P_load