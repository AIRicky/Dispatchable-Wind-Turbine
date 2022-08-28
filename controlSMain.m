%% 功能：采用负荷曲线测试控制策略是否支持CAWT实现可调度性

clc
clear all
close all

% 作图控制
Cp_curve_option = 1;
VDM_curve_option = 1;

%% Part 1 设定内部参数: Blade、Generator、VDM、Air tank
%% Part 1-1 Blade 
run  bladeSurfs.m
vin = 3.0;      % cut-in speed
v1_rated = 6.5; % elec-rated speed
voff = 15;      % cut-out speed 13.5 最多15
voff_Cp_tail_eff = voff; % =13.5 Cp 尾巴翘起, 函数可能不单调, 已通过劫尾方法处理为单调函数

%% Part 1-2 Generator 
etaG = 0.99;    % 电机效率    
etaX = 1;       % gearbox 效率 
etaVc = 0.65;   % Vc效率	65%
etaVp = etaVc;  % Vp效率 = Vc效率, 只有等效才能保证转速的连续可调性
   
Pgen_rated = 250*1000; % 发电机 250kW 
Pvdmin_rated = Pgen_rated/etaG/etaX/etaVc; % VDM 额定功率
Pblade_rated = Pgen_rated/etaG/etaX + Pvdmin_rated; % 机械额定功率
    
aai = Pgen_rated/etaG/etaX/(v1_rated^3-vin^3); % aii=0.5*rho*Cp*AW
bbi = aai*vin^3; % 该值等于floss, 即为 cut-in speed
v2_rated = ((Pblade_rated + bbi)/aai)^(1/3); % mech-rated speed, 可根据VDM额定输入功率自动计算

P_speed_power = @(z) max(0, aai*z^3 - bbi); % 风速到风能函数
%  floss  = P_speed_power(vin);

Cp_max = max(optCp); % Cp 最大值-表明无pitch时最大能力
AW = aai/(0.5*1.225*Cp_max);  % 1/2*rho*Cp*Aw  *V3==ai  *V3
radius_Balde = sqrt(AW/pi()); % m

%% Part 1-3 VDM
RD = 2.0; % wobble plate 盘片直径(m)
Hmax = 1.8; % H（NPS）最大长度(m)
S_c1 = 54; % 单一活塞头面积pistion, 表示已经discount/offset了单位转换引起的scale up/down, 可设置在50之间
Npiston = 23; % 活塞缸总数, 高压推送缸, 低压吸气缸, 封闭压缩缸

TqdL = 5000;% 力矩计算和绘图上限
VDMspdL = 1200*2*pi()/60/(1-etaVc);% 设置VDM额定转速1200rpm对应125rad/s

%% Part 2-4 Air tank
p0_c2 = 1; % 相对气压为1bar, 已经在S_c1=1时offset了单位转换引起的误解（p0.*S_c1总是同时出现）
etaT = .9999;  % Tank效率, 也可设置为气压的函数
press_Min = 3.0; %2.9152; % operational pressure,至少要保证的安全操作气压。安全低压1.5 ，补漏值为2.9152 % unit 为 Bar
press_Max = 8.0; % 安全高压  % gamaL=25;% 压缩率计算和绘图上限，貌似vdm最大压缩比6.195，因此不能大于6，否则无交点。已经提高到8.9
VtankL = 100000*2.5; % 单位是m^3
 
run vdmSurfs.m

%% Part 2 设定外部参数: 风侧风速与网侧调度指令
%% Part 1-1 风侧风速 wind speed
% 输入测试风速
% V_wind_serial=[5.2 5.8 6.7 7.2 7.8 8.9 9.2 7.9 7.2 6.4 5.9 7.7 9.0 8.4 6.9 5.5 4.5 3.8 3.3 4.0 4.8 5.6 6.3 6.6];
% P_load_serial=[100 220 200 140 250 250 250 170 200 250 240 190 150 150 200 250 230 200 180 250 210 150 120 100]*1000;

% 从sheet4拿数据, 写入到sheet2
V_wind_serial= xlsread('controlSTransR1.xlsx','Sheet4','D2:D1009')';% 'D2:D1009'
N_V_wind = 1008; % 24*7 or 1008=24*6*7，每十分钟一个数据点
% 输入真实风速
% V_wind_serial=[10.26	9.2	7.61	6.69	5.06	6.25	7.18	8.36	9.98	9.5	10.39	8.97...
%     7.81	6.63	5.53	4.65	4.18	3.55	4.89	4.73	3.62	4.58	5.71	7.02];
% 风速和load功率限位
V_wind_serial = V_wind_serial(1:N_V_wind); % 截取数列长度

%% Part 2-2 网侧指令 load demand
LoadScale = 1.3; 
P_load_serial= xlsread('controlSTransR1.xlsx','Sheet4','E2:E1009')'*1000; % 'H4:H171'表示250kW满负荷
P_load_serial = LoadScale*P_load_serial(1:N_V_wind); % 截取数列长度
% V_wind_serial=min(max(V_wind_serial,vin),voff); % trim V wind speed 截去峰值
P_load_serial = min(max(P_load_serial,0), Pgen_rated); % trim Pload 截去峰值


%% Part 3 给定初始气压, 风速, H值, 得到 P_vdm need 设定数据
pres_gama_Ratio = 0.8*press_Max; % 设定系统开机前初始气压, 同时pres_gama_Ratio*p0在仿真循环中也会不断更新bar
E_tank = pres_gama_Ratio * p0_c2 * VtankL * log(pres_gama_Ratio)-E_tankLower; % 计算当前气压存量, Etank表征净能量
T_step = 1/6;

Hindx_ref = 0;
VDM_gentleness_coef = 0.25; % 系数用于控制 VDM 在接近两侧极限气压时的动作缓和程度,
                            % 快起快落有助于最快速捕捉能量, 弊端是功率变化太大且不够缓和。
% 开始记录额定值
CapRecords = [vin v1_rated v2_rated voff radius_Balde Pblade_rated/1000 Pvdmin_rated/1000 Pgen_rated/1000 press_Max press_Min  E_tankL/1000 VtankL/1000 E_tank/1000];

%% Part 4 运行仿真, 获取各时刻运行参数
wind_indx = 0;

for V_wind = V_wind_serial
    wind_indx = wind_indx + 1;
    
    P_load_ref = min(Pgen_rated, P_load_serial(wind_indx)); % 导入power load dispatch的预测值, 已经trim过不超过gen容量
    Pblade_cal = min(P_speed_power(V_wind), Pblade_rated); % 已添加限位的blade power curve

    % Pvdm_gap=Pblade_cal-Pgen_rated/etaG; % 原始发电目标是Pgen_rated
    Pvdm_gap = Pblade_cal - P_load_ref/etaG; % 当前发电目标为 Pgen_rated, P_load_ref
    % etaT=1.0;  % Tank效率恒定
    etaT = (2000 - pres_gama_Ratio*T_step)/2000;  % Tank效率为气压的函数; 若气压为7, 步长为0.2, 损失为千分之0.7, 每小时损失为千分之3.5, 24小时损失为8.4%
    % Pvdmneed=(Pvdm_gap>=0)*min([Pvdm_gap,E_tankL-etaT*E_tank,Pvdmin_rated])+(Pvdm_gap<0)*max([Pvdm_gap,-etaT*E_tank,-Pvdmin_rated]); 
    % Pvdmneed=(Pvdm_gap>=0)*min([Pvdm_gap,((E_tankL+E_Tank_tolerance)-etaT*E_tank),Pvdmin_rated])+(Pvdm_gap<0)*max([Pvdm_gap,-etaT*E_tank,-Pvdmin_rated]); 
    
    % 添加效率系数, 将功率全部转化到齿箱侧
    Pvdmneed = (Pvdm_gap>=0)*min([Pvdm_gap, (E_tankL-etaT*E_tank)/etaVc/T_step*VDM_gentleness_coef, Pvdmin_rated]) +...
               (Pvdm_gap<0)*max([Pvdm_gap, -etaT*E_tank*etaVp/T_step*min(1,VDM_gentleness_coef+0.5), -Pvdmin_rated]); 
    % 若P_vdm大于0, 受限于气罐剩余容量; 若P_vdm小于0, 受限于气罐已存在容量; 若不碰到这两个边界 (气罐容量边界, vdm功率边界), gap和need一样
    % T_step是为了将单位统一到功率侧
    % 若风速进入不工作区间, 发电量全由VDM来承担, P_vdm_need 实际上全部重新计算, load power全部由VDM补足
    Pvdmneed = -(1-((V_wind >= vin) & (V_wind <= voff)))* P_load_ref/etaG + ((V_wind >= vin) & (V_wind <= voff))* Pvdmneed;

    % 开始通过Scall.m, 循坏使用Surfs.m文件
    run vdmScall.m
    % 返回VDM实际能够实现的数据及VDM运行参数
    run bladeScall.m
    % 返回blade, tank, CVT, gen的运行参数
    run vdmRescall.m

    % 记录运行值
    OprRecords(wind_indx,:) = [V_wind, Pblade_cal/1000, Pvdm_gap/1000, Pvdmneed/1000, Pblade_ref/1000, Pvdm_ref/1000,  Pgenin_ref*etaG/1000, P_load_ref/1000, E_tank/1000, ...
                               CVT_Ratio_ref, gama_ref, pres_gama_Ratio,  ...
                               Cp_ref, beta_ref, numda_ref, H_ref,dlt_ref, ...
                               omg_shaft1_ref,Tq_gen_ref,VDMspd_ref, Tqd_ref, P_spin_Resv/1000 ];
end

% 将结果写入excel待分析
xlswrite('controlSTransR1.xlsx','   ','sheet2','B4:B1011')
xlswrite('controlSTransR1.xlsx', CapRecords,'sheet2','B2')
xlswrite('controlSTransR1.xlsx', OprRecords,'sheet2','B4')
disp('Done writing')
winopen('controlSTransR1.xlsx')

