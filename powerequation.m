function power_Eq = powerequation(xdelta) 



gamaVc=5.4; % 7.0 Bar
Pvdm_need=250*1000 ; % 125kW
delta_ref0=pi/8 ; % 专门用于计算discount的

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





            %-------------------------------------------
            % k2=etaVc考虑到动态过程，在管壁上折损一半的时候，出现最大功率
            gama_pu=gamaVc/(press_Max/p0_c2); %计算当前气压的标幺值per unit，当前气压占最大气压的多少
%             preslossdiscount=0.4*sin(dlt)/sin(pi()/4);% 压力减损的折扣系数，最前面的0.4是自己定义            

            preslossdiscount=0.4*sin(xdelta)/sin(pi()/4);% 压力减损的折扣系数，最前面的0.4是自己定义
                %空压机转速在1200rpm附近，1200round/min=1200round*6.28rad/round/60s=125rad/s
                VDMspdVc_ref=VDMspdL*(1-(1-gama_pu)*preslossdiscount)/2  *2*(1-etaVc);% 功率最优值，定义为etaVc=65%最优功率处，即(1-etaVc)=35%转速处，最大功率转速点实际上是1/2点，此点*2*35%就是35%点
                VDMspdVp_ref=VDMspdL*(1-(1-gama_pu)*preslossdiscount)/2  *2*(1-etaVp);% 功率最优值，定义为etaVp=65%最优功率处，即(1-etaVc)=35%转速处
                %-------------------------------------------
                %air motor时候，轴力矩是气压力矩打折扣，compressor时候，气压力矩是轴力矩打折扣。
                %最后结果是算到CVT齿轮箱端口的，所以有时候乘以yita，有时候除以yita

                PvdmVc=VDMspdVc_ref*TqdVc_coef*tan(xdelta);            % 计算comp压缩功率，主齿箱输出正功率,最后单位转换为kW
                PvdmVp=VDMspdVp_ref*TqdVp_coef*tan(xdelta);            % 计算exp膨胀功率，主齿箱输出负功率,最后单位转换为kW
                
                if Pvdm_need>=0
                    power_Eq=PvdmVc-Pvdm_need;
                else
                    power_Eq=PvdmVp-Pvdm_need;
                end
    end  