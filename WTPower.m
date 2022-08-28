vin = 3.0;      % cut-in speed
v1_rated = 6.5; % elec-rated speed
voff = 15;      % cut-out speed 13.5 最多15
voff_Cp_tail_eff = voff; % =13.5 Cp 尾巴翘起, 函数可能不单调, 已经通过劫尾方法处理为单调函数

%% Part 2-2 Generator 
etaG = 0.99;    % 电机效率    
etaX = 1;       % gearbox 效率 
etaVc = 0.65;   % Vc效率	65%
etaVp = etaVc;  % Vp效率 = Vc效率  在此案例中，只有等效才能保证转速的连续可调性
   
Pgen_rated = 250*1000; % 发电机 250kW 
Pvdmin_rated = Pgen_rated/etaG/etaX/etaVc; % VDM 额定功率
Pblade_rated = Pgen_rated/etaG/etaX + Pvdmin_rated; % 机械额定功率
    
aai = Pgen_rated/etaG/etaX/(v1_rated^3-vin^3); %aii=0.5*rho*Cp*AW
bbi = aai*vin^3; % 这个值就等于floss, 即为 cut-in speed
v2_rated = ((Pblade_rated + bbi)/aai)^(1/3); % mech-rated speed 可根据自动 vdm 额定输入功率计算的

P_speed_power = @(z) max(0, aai*z^3 - bbi);

V_wind_serial= xlsread('controlSTransR1.xlsx','Sheet4','D2:D1009')';% 'D2:D1009'
N_V_wind = 1008; % 24*7 or 1008=24*6*7，每十分钟一个数据点
% 输入真实风速
% V_wind_serial=[10.26	9.2	7.61	6.69	5.06	6.25	7.18	8.36	9.98	9.5	10.39	8.97...
%     7.81	6.63	5.53	4.65	4.18	3.55	4.89	4.73	3.62	4.58	5.71	7.02];
% 风速和load功率限位
V_wind_serial = V_wind_serial(1:N_V_wind); % 截取数列长度
LoadScale = 1.3;
P_load_serial= xlsread('controlSTransR1.xlsx','Sheet4','E2:E1009')'*1000; % 'H4:H171'表示250kW满负荷
P_load_serial = LoadScale*P_load_serial(1:N_V_wind); % 截取数列长度
% V_wind_serial=min(max(V_wind_serial,vin),voff);% trim V wind speed 截去峰值
P_load_serial = min(max(P_load_serial,0), Pgen_rated); % trim Pload 截去峰值
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