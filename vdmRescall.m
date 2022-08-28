%% 计算vdm在当前气压条件下的剩余能力

% 风能富余时, rev=富余能量减去已用电机容量
P_blade_surplus = max(0,min(Pvdm_gap, (Pgen_rated - P_load_ref)/etaG)); % 富余风能和电机剩余容量, 取小
dltindx = 0; % wobble angle 初始编号置零
for dlt = dltrange  % wobble angle 0度-45度
    dltindx = dltindx+1;  % wobble angle 编号累加
    Hindx = 0;% NPS 初始编号置零

    % 把扫面点补全了
    srangeIndx = find(Hrange<=RD*sin(dlt)); % 通过搜索条件，找到不可行格子编号, 进行赋值
    for  H = Hrange(srangeIndx)
        Hindx = Hindx + 1; % NPS 编号累加
        eval_reserves(Hindx,dltindx) = NaN;
    end
    srangeInd = find(Hrange>RD*sin(dlt)) ; % find 的输出结果是向量里面的位置号
    for  H = Hrange(srangeInd) % 偏移量0-最大值20
        Hindx = Hindx + 1; % NPS 编号累加
        eval_reserves(Hindx,dltindx) = err_amplifier*(abs((gama(Hindx,dltindx)-pres_gama_Ratio)))...
            + 1/err_amplifier*(abs((PvdmVp(Hindx,dltindx)-Pvdm_suply_negMax)))+1/10000*(abs((Hindx-Hindx_ref))); % 添加了微小系数消除evl_abs在0附近的多值
    end

end
% 循环赋值eval_abs完毕, 进行判定
min_eval_resv = min(min(eval_reserves,[],1),[],2);
[Hindx_Mref,dltindx_Mref] = find(eval_reserves == min_eval_resv);
% 风能欠缺时, rev=最大能力减去已用能力, 然后与电机空余容量取小
P_vdm_surplus = -(PvdmVp(Hindx_Mref, dltindx_Mref) - min(0,Pvdm_ref)); % 富余能力等于最大能力减去已经使用的能力, 加负号切换成正数
P_spin_Resv = min((P_vdm_surplus + P_blade_surplus)*etaG,(Pgen_rated - P_load_ref)); % 然后用电机空容量限位
