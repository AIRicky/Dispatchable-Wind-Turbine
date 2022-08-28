%% 此函数用于生成VDM的各种工作曲面
% 力矩和功率已经转换到gearbox侧, 等待vdmcall.m的调用
% 此模型和谐了效率和最优转速的统一, 画出了功率曲面，以及对应力矩转速气压曲面


%% 环境参数
% clc
% clear all
% close all
%% VDM机器相关参数

% %定义效率
% etaVc = 0.65;  % Vc效率	65%
% etaVp = etaVc;  % Vp效率=Vc效率, 在此案例中只有等效才能保证转速的连续可调性

% syms k % 定义一个系统符号变量, 计算symsum做数列累加的时候要用到

N_p_low = floor(Npiston/2); % 低压吸气缸数目, 总数的一半, 向下求整
N_p_high = max(floor(N_p_low/2)-1, 2); % 恒压平推缸/高压推送缸数目, 是低压气缸的1/3, 总数的1/2*1/3, 但不少于两个, 向下求整
N_p_retain = Npiston - N_p_low - N_p_high; % 封闭压缩缸数目, 剩余数目

% 分别计算压缩推送角度
piedeg = 2*pi()/Npiston; % 单一缸占pie饼的角度/弧度
ang1 = piedeg*(N_p_retain+1); % 计算压缩角ang1的角度
ang2 = piedeg*(N_p_high-1); % 计算推送角ang2的角度


%% 气罐相关参数
press_Min = max(press_Min,1); % 限位最低气压的设定不会小于1
E_tankUpper = press_Max*VtankL*log(press_Max/p0_c2); % 气罐额定容量计算
E_tankLower = press_Min*VtankL*log(press_Min/p0_c2); % 气罐额定容量计算
E_tankL = E_tankUpper - E_tankLower; % 气罐额定容量计算, EtankL表征净能量

%% 对缸体组合进行运算
dltrange = 0:pi/4/200:pi/4;   % 设置x轴delta的扫描范围 Wobble angle
Hrange   = 0:Hmax/400:Hmax;   % 设置y轴 H  的扫描范围 Netrol piston shift

dltindx = 0; % wobble angle 初始编号置零
for dlt = dltrange  % wobble angle 0度-45度
    dltindx = dltindx+1;  % wobble angle 编号累加
    Hindx = 0;% NPS 初始编号置零
    
    % 补全扫面点
    srangeIndx = find(Hrange <= RD*sin(dlt));% 通过搜索条件, 找到不可行格子的编号并进行赋值
    for  H = Hrange(srangeIndx)
    Hindx = Hindx + 1; % NPS 编号累加
            gama(Hindx,dltindx) = NaN;
            % Vc torque speed power赋值
            TqdVc(Hindx,dltindx) = NaN;
            VDMspdVc(Hindx,dltindx) = NaN;
            PvdmVc(Hindx,dltindx) = NaN;
            % Vp torque speed power赋值
            TqdVp(Hindx,dltindx) = NaN;
            VDMspdVp(Hindx,dltindx) = NaN;
            PvdmVp(Hindx,dltindx) = NaN;
    end
    
    srangeInd = find(Hrange > RD*sin(dlt)) ; % find 的输出结果是向量里面的位置号
    for  H = Hrange(srangeInd) % 偏移量0-最大值20
    Hindx = Hindx + 1; % NPS 编号累加
            V1 = H + RD*sin(dlt); % 初始体积, 初始角度为0, cos（0）=1
            V2 = H + RD*sin(dlt)*cos(ang1); % 终态体积
            % gama(Hindx,dltindx)=min((V1/V2),(press_Max/p0_c2)); % 计算压缩比, 并且加限位
            % 去除压缩比限位, 否则gama的计算值与H, dlt函数不对应
            gama(Hindx,dltindx) = (V1/V2); % 计算压缩比, 并且加限位    
            
            %-------------------------------------------
            % k2=etaVc考虑到动态过程, 在管壁上折损一半的时候, 出现最大功率
            gama_pu = gama(Hindx,dltindx)/(press_Max/p0_c2); %计算当前气压的标幺值per unit
            preslossdiscount = 0.4*sin(dlt)/sin(pi()/4);% 压力减损的折扣系数, 最前面的0.4是自己定义
            
            % 空压机转速在1200rpm附近，1200round/min=1200round*6.28rad/round/60s=125rad/s
            VDMspdVc(Hindx,dltindx) = VDMspdL*(1-(1-gama_pu)*preslossdiscount)/2*2*(1-etaVc); % 功率最优值, 定义为etaVc=65%最优功率处, 即(1-etaVc)=35%转速处
            VDMspdVp(Hindx,dltindx) = VDMspdL*(1-(1-gama_pu)*preslossdiscount)/2*2*(1-etaVp); % 功率最优值, 定义为etaVp=65%最优功率处, 即(1-etaVc)=35%转速处
            
            %-------------------------------------------
            Tqfun1 = @(z) (H + RD.*sin(dlt))./(H + RD.*sin(dlt).*cos(z)).*p0_c2.*S_c1.*RD.*tan(dlt).*sin(z);
            Tqfun2 = @(z) gama(Hindx,dltindx)                        *p0_c2 *S_c1 *RD *tan(dlt) *sin(z);
            Tqd1 = integral(Tqfun1,0,ang1); % compression力矩积分
            Tqd2 = integral(Tqfun2,ang1,ang1+ang2); % exhaust力矩积分
            Tqd_sum_ideal = (Tqd1+Tqd2)/2/pi*Npiston; % 盘面上的理想力矩, 合计值
            
            %air motor时轴力矩是气压力矩打折扣; compressor时气压力矩是轴力矩打折扣, 所以有时乘以yita, 有时除以yita
            TqdVc(Hindx, dltindx) =  min(Tqd_sum_ideal/etaVc,TqdL); % 定义compr为正力矩, 此力矩为VDM主轴从齿箱侧拿走的力矩
            PvdmVc(Hindx, dltindx) = VDMspdVc(Hindx, dltindx)*TqdVc(Hindx,dltindx); % 计算comp压缩功率，主齿箱输出正功率, 最后单位转换为kW
            TqdVp(Hindx, dltindx) = -min(Tqd_sum_ideal*etaVp,TqdL); % 定义exp为负力矩, 此力矩为VDM主轴递送到齿箱侧的力矩
            PvdmVp(Hindx, dltindx) = VDMspdVp(Hindx,dltindx)*TqdVp(Hindx,dltindx); % 计算exp膨胀功率, 主齿箱输出负功率, 最后单位转换为kW
    end

end

        % max(max(TqdVc,[],1),[],2)
        % TqdL
            Pvdm_suply_negMax = 1.2*min(min(PvdmVp,[],1),[],2); % 计算PvdmVp里面的最大功率, 也就是这一气压条件下最大expander能力, 是负数
            
if VDM_curve_option %% VDM 工作曲面图
    Pvdm_unit= max(max(PvdmVc,[],1),[],2);
    gama_unit= max(max(gama,[],1),[],2);
    
    figure % 绘制三维曲面
    subplot(2,1,1)  % 绘制 VDM power曲面
    surf(dltrange, Hrange, 10*PvdmVc/1e3,'edgecolor','none') % 10 & 1e3 is just for Science Paper (2019/04/09) 
    xlabel('\delta (rad)')
    ylabel('H (m)')
    zlabel('P^{Vc} (kW)')
    view([-60 30])
    title('VDM compression power surface')
   
    subplot(2,1,2) %绘制 VDM power等高线
    contour(dltrange,Hrange,10*PvdmVc/1e3,'ShowText','on') % 10 & 1e3 is just for Science Paper (2019/04/09) 
    title('VDM compression power contour')
    colorbar
    xlabel('\delta (rad)')
    ylabel('H (m)')

    figure 
    subplot(2,1,1) % 绘制gama曲面
    surf(dltrange, Hrange, gama,'edgecolor','none') 
    xlabel('\delta (rad)')
    ylabel('H (m)')
    zlabel('\gamma^{Vc}')
    view([-60 30])
    title('Compression ratio surface')

    subplot(2,1,2) % 绘制gama 等高线             
    contour(dltrange, Hrange, gama,'ShowText','on') 
    title('Compression ratio contour')
    colorbar
    xlabel('\delta (rad)')
    ylabel('H (m)')

    figure   % 绘制meshgrid
    contour(dltrange, Hrange, PvdmVc/Pvdm_unit,'r--','ShowText','on')
    set(gcf,'renderer','zbuffer');
    hold on
    contour(dltrange, Hrange, gama/gama_unit,'linewidth',1,'ShowText','on')
    title('Torque contour')
    hold off
    xlabel('\delta (rad)')
    ylabel('H (m)')
end
