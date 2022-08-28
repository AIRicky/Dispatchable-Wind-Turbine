% 本文取代vdmSurf和vdmcall
% 用求解方程的方式来得到delta和H
% 参数和变量全部来源于vdmSurf以及各交叉接口

%% 环境参数
% clc
% clear all
% close all
% gamaVc=pres_gama_Ratio; % 7.0 Bar
for i=1:15 % 此loop是为了计算在各个气压下，计算call功率的返回值
gamaVc=0.5*i+0.5; % 给定气压7.0 Bar% 原来是 gamaVc=7.0来测量单气压点的返回功率
Pvdm_need=-250*1000 ; % 给定功率 125kW
delta_ref0=pi/8 ; % 专门用于计算Tq 因为流量造成discount的预设角

%% VDM机器相关参数

%定义效率
etaVc=0.65;  % Vc效率	65%
etaVp=etaVc;  % Vp效率=Vc效率  在此案例中，只有等效，才能保证转速的连续可调性

% syms k % 定义一个系统符号变量，等会计算symsum做数列累加的时候要用到
RD=2.0;% wobble plate 盘片直径
Hmax=1.8;% H（NPS）最大长度
S_c1=2.35*23;% 单一活塞头面积pistion,表示已经discount/offset了单位转换引起的scale up/down，此数字可以设置在2.0-2.6之间
% 活塞缸总数和高压推送缸，低压吸气缸，封闭压缩缸
Npiston=23;
N_p_low=floor(Npiston/2); % 低压吸气缸数目,总数的一半，向下求整数
N_p_high=max(floor(N_p_low/2)-1,2); % 恒压平推缸/高压推送缸数目，是低压气缸的1/3，总数的1/2*1/3，但不少于两个，向下求整数
N_p_retain=Npiston-N_p_low-N_p_high; % 封闭压缩缸数目，剩余数目。

% 分别计算压缩推送角度
piedeg=2*pi()/Npiston; % 单一缸占pie饼的角度/弧度
ang1=piedeg*(N_p_retain+1); % 计算压缩角ang1的角度
ang2=piedeg*(N_p_high-1); % 计算推送角ang2的角度
TqdL=5000;% 力矩计算和绘图上限
VDMspdL=1200*2*pi()/60/(1-etaVc);% 设置VDM额定转速1200rpm对应125rad/s

%% 气罐相关参数
p0_c2=1;% 相对气压为1bar，已经在S_c1=1的时候，offset了单位转换引起的误解，因为p0.*S_c1总是同时出现的
etaT=.9999;  % Tank效率是参数99.99%,在control里面设置了：Tank效率为气压的函数
press_Min=1.5;% operational pressure,至少要保证的安全操作气压。安全低压
press_Max=8.0; % 安全高压  % gamaL=25;% 压缩率计算和绘图上限，貌似vdm最大压缩比6.195，因此不能大于6，否则无交点。已经提高到8.9
VtankL=100000*2.5;% 单位是立方米
press_Min=max(press_Min,1); % 限位最低气压的设定不会小于1

E_tankUpper=press_Max*VtankL*log(press_Max/p0_c2);% 气罐额定容量计算
E_tankLower=press_Min*VtankL*log(press_Max/p0_c2);% 气罐额定容量计算
E_tankL=E_tankUpper-E_tankLower;% 气罐额定容量计算，EtankL表征净能量

%% 正文
k1=RD.*(cos(ang1)*gamaVc-1)/(1-gamaVc);% 用于gama计算的参数k1，H=k1*sin（delta）
k2=p0_c2.*S_c1.*RD;% 用于tq计算的参数k2，常系数

Tqd1_coef = -k2*(k1+RD)/RD*(log(abs(k1+RD*cos(ang1)))-log(abs(k1+RD*cos(0)))); %close compression 部分积分结果
%Tq1 积分结果ln|k1+RD*cos(ang1)|
Tqd2_coef = -gamaVc*k2*(cos(ang1+ang2)-cos(ang1)); %open ejection部分积分结果
%Tq2 积分结果cos(ang1+ang2)
Tqd_sum_ideal_coef = (Tqd1_coef+Tqd2_coef)/2/pi*Npiston; % 盘面上的理想力矩，合计值
% 积分后在2pi平均，然后乘以汽缸数

%air motor时候，轴力矩是气压力矩打折扣，compressor时候，气压力矩是轴力矩打折扣。
%最后结果是算到CVT齿轮箱端口的，所以有时候乘以yita，有时候除以yita
TqdVc_coef =  min(Tqd_sum_ideal_coef/etaVc,TqdL);  % 定义compr为正力矩，此力矩为VDM主轴从齿箱侧拿走的力矩
TqdVp_coef = -min(Tqd_sum_ideal_coef*etaVp,TqdL);  % 定义exp为负力矩，此力矩为VDM主轴递送到齿箱侧的力矩

% 因为delta_ref1也会影响到流量，所以会影响speed的取值，但是是一个常系数方程。
% preslossdiscount=@(xdelta)  0.4*sin(xdelta*0+delta_ref0)/sin(pi()/4);% 压力减损的折扣系数，最前面的0.4是自己定义
preslossdiscount1=@(xdelta)  0.4*sin(xdelta)/sin(pi()/4);% 压力减损的折扣系数，最前面的0.4是自己定义
%空压机转速在1200rpm附近，1200round/min=1200round*6.28rad/round/60s=125rad/s
gama_pu1=gamaVc/(press_Max/p0_c2); %计算当前气压的标幺值per unit，当前气压占最大气压的多少
% 功率最优值，定义为etaVc=65%最优功率处，即(1-etaVc)=35%转速处，最大功率转速点实际上是1/2点，此点*2*35%就是35%点
VDMspdVc_ref=@(xdelta)  VDMspdL*(1-(1-gama_pu1)*preslossdiscount1(xdelta))/2  *2*(1-etaVc);
VDMspdVp_ref=@(xdelta)  VDMspdL*(1-(1-gama_pu1)*preslossdiscount1(xdelta))/2  *2*(1-etaVp);

PvdmVc1=@(xdelta)  VDMspdVc_ref(xdelta)*TqdVc_coef*tan(xdelta);   % 计算comp压缩功率，主齿箱输出正功率,最后单位转换为kW
PvdmVp1=@(xdelta)  VDMspdVp_ref(xdelta)*TqdVp_coef*tan(xdelta);   % 计算exp膨胀功率，主齿箱输出负功率,最后单位转换为kW

if Pvdm_need>=0
    power_Eq=@(xdelta) PvdmVc1(xdelta)-Pvdm_need;
else
    power_Eq=@(xdelta) PvdmVp1(xdelta)-Pvdm_need;
end

dlt_ref1=fzero(power_Eq,delta_ref0); % fzero 无法通过传递给方程赋值。

dlt_ref1    =  max(min(dlt_ref1,pi/4),-pi/4); % 对delta限位
H_ref1      =  k1* sin(dlt_ref1); %反向计算 H
Pvdm_ref1   = (Pvdm_need>=0)*PvdmVc1(dlt_ref1)+(Pvdm_need<0)*PvdmVp1(dlt_ref1); % 计算本轮Pvdm_ref
VDMspd_ref1 = (Pvdm_need>=0)*VDMspdVc_ref(dlt_ref1)+(Pvdm_need<0)*VDMspdVc_ref(dlt_ref1); % 选择转速
Tqd_ref1    =  Pvdm_ref1/VDMspd_ref1 ; % 计算力矩

% 不光记录返回功率，还要记录返回功率对应的转速和力矩
Vrecord_MaxP_Tq(i,:) = [gamaVc,Pvdm_ref1,VDMspd_ref1,Tqd_ref1,dlt_ref1,H_ref1];
end

%% 以数值积分的方式做对比试验(已验证成功，积分公式是无误的)
% % 计算本方法中的力矩
%     Tqd_sum_ideal1=Tqd_sum_ideal_coef*tan(dlt_ref1);
% % 认为转速是事先定好的，返回计算tq的时候，不care角度
% % 实际上，delta角度也决定流量，所以其实是相关的（但是不好积分）
%     Tqfun1 = @(z) (H_ref1+RD.*sin(dlt_ref1))./(H_ref1+RD.*sin(dlt_ref1).*cos(z)).*k2.*tan(dlt_ref1).*sin(z);
%     Tqfun2 = @(z) gamaVc                                                         *k2 *tan(dlt_ref1) *sin(z);
%     Tqd1_integral = integral(Tqfun1,0,ang1); %compression力矩积分
%     Tqd2_integral = integral(Tqfun2,ang1,ang1+ang2); %exhaust力矩积分
%     Tqd_sum_ideal_integral = (Tqd1_integral+Tqd2_integral)/2/pi*Npiston; % 盘面上的理想力矩，合计值
