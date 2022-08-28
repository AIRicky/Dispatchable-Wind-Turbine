%% Function: 调用blade函数, 通过输入风力, 计算出 blade 设定功率，
% blade 功率 和最大可获得功率比值, 得到对应的 prim shaft转速和 blade angle

%% 假设已获得Cp-curve
% clc % CLean Command windows
% close all

artificial_gearR = 100; % 增速齿轮箱                                        
%   V_wind=12;
%   Pblade_cal=min(P_speed_power(V_wind),Pblade_rated); % 已经添加限位的blade power curve
%   Pvdmneed=min(Pblade_cal-Pgen_rated/etaG,Pvdmin_rated); % 送出给vdm做需求, 单位W
%   Pvdm_ref=.6*Pvdmneed; % 从vdm返回, 得到证实需求, 单位W
% % 如何保证Pvdm_ref的绝对值一定小于等于Pvdmneed

if  (V_wind >= vin) && (V_wind <= voff) 
    Pblade_ref = min(Pblade_cal, P_load_ref/etaG + Pvdm_ref);
    Pgenin_ref = Pblade_ref-Pvdm_ref; 
     %% begein  再来一个二次调整，减少计算误差
        if (Pgenin_ref - P_load_ref/etaG < 0) && (Pvdm_ref > 0) % 判定发电量没有满足load同时又给power到compressor
            vdm_err_capacity = min(Pvdm_ref, P_load_ref/etaG - Pgenin_ref); % 计算这一部分misplaced的能量
            Pgenin_ref = Pgenin_ref + vdm_err_capacity ; %还回给发电
            Pvdm_ref = Pvdm_ref - vdm_err_capacity; %/etaVp 从comprssor功率中扣除
        else
        end

        if (Pblade_cal - Pblade_ref > 0) && (Pvdm_ref < 0) % 避免弃风之后, 反而用 expander
            vdm_err_capacity = min(Pblade_cal-Pblade_ref,-Pvdm_ref); % 计算这一部分 misplaced 的能量
            Pblade_ref = Pblade_ref + vdm_err_capacity; % 还回给发电, 修正后的 blade_ref
            Pvdm_ref = Pvdm_ref + vdm_err_capacity; %/etaVp 从comprssor功率中扣除
        else
        end                
             %% end     再来一个二次调整，减少计算误差
    Cp_cal = Cp_max*Pblade_ref/P_speed_power(V_wind);
    [Cp_err,Cp_indx] = min(abs(optCp-Cp_cal)); %  R0011-B，找最接近的结果。          
             
else %  小于cut-in spd 或者 大于cut-out
    Pblade_ref = 0;
    Cp_cal = NaN; % 或者写最大值24度degree
%     display('High/Low wind:shutdown');
    Pgenin_ref = -Pvdm_ref; % 用 VDM 发电
    Cp_indx = (V_wind < vin)*1 + (V_wind > voff)*length(optCp); % beta角度, 要么最大, 要么最小
    % Cp_indx=Cp_indx; %之前做的是和上次角度一致
    % 一种选择是blade pitch和保持最近一次可用的角度, Cp_indx=1 or 226, 另一种方式是低于cut-in对应最小pitch, 高于cut-off对应最大pitch
end
% Pblade_cal
% Cp_cal
% Pblade_ref=Pblade_ref/1000 % 调整显示值w-kw
% Pvdm_ref=-Pvdm_ref/1000  % 调整显示值w-kw, 正负值定义
% Pgenin_ref=Pgenin_ref/1000  % 调整显示值w-kw

%% 计算叶片控制的参数，Cp值，以及对应的beta，numda 和 prim shaft rotation speed
% Cp_err=min(abs(optCp-Cp_cal)) %  R0011-A
% Cp_indx=find(abs(optCp-Cp_cal)==Cp_err)%  R0011-A
Cp_ref = optCp(Cp_indx);   
beta_ref = beta_range(Cp_indx);
numda_ref = optnumda(Cp_indx); % Cp值计算不准确会影响numbda的取值, 从而影响叶片和电机转速的设定
omg_shaft1_ref = numda_ref*min(max(V_wind,vin),voff)/radius_Balde*artificial_gearR; % 单位可能是 rad/s, 取决于曲线拟合时候定义的 numda


% figure  % Cp 一维展开，证明单调性
% plot(optCp)
% hold on
% scatter(Cp_indx,Cp_ref,'r')
% hold off


%% 计算电机力矩（即对应电流）
Tq_gen_ref = Pgenin_ref/omg_shaft1_ref;

%% 计算CVT变速比
CVT_Ratio_ref = omg_shaft1_ref/VDMspd_ref;

%% 计算气压更新
E_tank = pres_gama_Ratio*p0_c2*VtankL*log(pres_gama_Ratio)-E_tankLower; % 已知气压比, 求气罐能量, Etank表征净能量
E_tank = etaT*E_tank+(Pvdm_ref>=0)*Pvdm_ref*etaVc*T_step+(Pvdm_ref<0)*Pvdm_ref/etaVp*T_step; % 气罐能量update
E_tank = max(0,min(E_tank,E_tankL));
pres2nergy=@(xgama)  xgama*p0_c2*VtankL*log(xgama)-(E_tank+E_tankLower); % 建立气压能量函数, 用于计算气压更新, 在bladecall里面调用
% (E_tank+E_tankLower)表示绝对能量
pres_gama_Ratio = fzero(pres2nergy,pres_gama_Ratio);
