%% Function: 已知VDM全部工作曲面条件下, 找最近工作交点
% Note: 1) Pvdm_need 均为 Gearbox 侧功率;
%       2) 对比 eval_abs 偏差时用到了三次方, 来扩大1以上的误差, 让误差的计算非线性
%       3) 不 run 曲面生成的函数, 而是调用这一数据库的已知数据, 确定交点或者工作点
%       4) 作图说明：截取所需的等高线, 重叠放置, 目测交汇点; 另一路计算交汇点, 用圆圈标出, 已验证一致
%       5) if-else-end 模式下, 两个分支完全对称, 对应 EITHER compressor OR expander
%       6) 这种布置主要是减少运算量, 每次只执行一个分支

%% 不要轻易清零, 因为要调用其它函数
% clc
% close all

%             pres_gama_Ratio=6.7999; % 给定气压值
%             Pvdmneed=360.62*1000;   % 给定 VDM 功率需求值
%             Hindx_ref = 0;          % 设定一个初始值, 让选项在平行值之间, 选择距离上次 H 最近的地方

% 开始一个判断语句, 让每次进入EITHER comprssor OR expander
err_amplifier = 1000;

% gama_bar_tolerence=0.25;

if Pvdmneed >= 0
% 第一分支：compression mode
        dltindx = 0; % wobble angle 初始编号置零
        for dlt = dltrange  % wobble angle 0度-45度
            dltindx = dltindx + 1;  % wobble angle 编号累加
            Hindx = 0;% NPS 初始编号置零

            % 补全扫面点
            srangeIndx = find(Hrange <= RD*sin(dlt)); % 通过搜索条件, 找到不可行格子编号并进行赋值
            for  H = Hrange(srangeIndx)
                Hindx = Hindx + 1; % NPS 编号累加
                eval_abs(Hindx,dltindx) = NaN; % wobble plate 撞击缸底, 不可用区间, 赋值为NaN
            end
            srangeInd = find(Hrange > RD*sin(dlt)); % find 的输出结果是向量里面的位置号
            for  H = Hrange(srangeInd) % 偏移量0-最大值20
                Hindx = Hindx+1; % NPS 编号累加
            % 此处添加了 if 语句, 让 VDM不要超能力僭越, 剥夺 Pgen 地盘
            % 对比 eval_abs 偏差时用到了三次方, 来扩大 1 以上的误差, 让误差的计算非线性, 但此操作带来的结果是会增加1以内的误差容忍度
            % 还需要补充【airtificial 高压力, 零功率】的许可
                    if 0 %(PvdmVc(Hindx,dltindx)-Pvdmneed)>0 % || abs(gama(Hindx,dltindx)-pres_gama_Ratio)>=gama_bar_tolerence
                        eval_abs(Hindx,dltindx) = NaN;% 不允许超能力, 即shaft侧送来500kW, 不允许压缩机工作在501kW中; 但是膨胀机允许这种超能力
                    else
                        eval_abs(Hindx,dltindx) = err_amplifier*(abs((gama(Hindx,dltindx)-pres_gama_Ratio)))...
                            + 1/err_amplifier * (abs((PvdmVc(Hindx,dltindx)-Pvdmneed))) + 1/10000*(abs((Hindx-Hindx_ref))); % 添加了微小系数，消除 evl_abs 在0附近的多值
%                         eval_abs(Hindx,dltindx)=err_amplifier*(abs((gama(Hindx,dltindx)-pres_gama_Ratio)))^3+1/err_amplifier*(abs((PvdmVc(Hindx,dltindx)-Pvdmneed))); % 
                    end

            end

        end
        % 循环赋值 eval_abs 完毕, 进行判定
        min_eval_abs = min(min(eval_abs,[],1),[],2);
        [Hindx_ref, dltindx_ref] = find(eval_abs == min_eval_abs);% 之所以定义为 buffer 是因为计算值可能是一个vector
        H_ref = Hrange(Hindx_ref);
        dlt_ref = dltrange(dltindx_ref);% 也可把弧度转换成角度 radtodeg(dlt_ref)
        gama_ref = gama(Hindx_ref,dltindx_ref);
        Tqd_ref = TqdVc(Hindx_ref,dltindx_ref);
        VDMspd_ref = VDMspdVc(Hindx_ref,dltindx_ref);
        Pvdm_ref = PvdmVc(Hindx_ref,dltindx_ref);     
%                 gama_ref_box=gama(min(Hindx,max(1,Hindx_ref+(-1:1))),min(dltindx,max(1,dltindx_ref+(-1:1))))% 看看局部微观
%                 Pvdm_ref_box=PvdmVc(min(Hindx,max(1,Hindx_ref+(-1:1))),min(dltindx,max(1,dltindx_ref+(-1:1))))%% 看看局部微观

       %% 作图验证运算结果。
        % figure %绘制evaluation曲面
        % surf(dltrange,Hrange,eval_abs,'edgecolor','none') 

%         figure %绘制两个等高线，以及认定的交点
%         hold on
%         contour(dltrange,Hrange,gama,[pres_gama_Ratio,pres_gama_Ratio],'b--')
%         %压缩比和力矩等高线 scale down 到 p.u.值之后重叠找交点
%         contour(dltrange,Hrange,PvdmVc,[Pvdmneed,Pvdmneed],'b-','linewidth',1)
%         scatter(dltrange(dltindx_ref),Hrange(Hindx_ref),'r')%功率最大点对应转速
%         % title('Torque contour')
%         set(gcf,'renderer','zbuffer');
%         hold off

else % elseif Pvdmneed<0
% 第二分支：expansion mode
        dltindx = 0; % wobble angle 初始编号置零
        for dlt = dltrange  % wobble angle 0度-45度
            dltindx = dltindx+1;  % wobble angle 编号累加
            Hindx = 0;% NPS 初始编号置零

            % 把扫面点补全了
            srangeIndx = find(Hrange<=RD*sin(dlt)); % 通过搜索条件, 找到不可行格子编号, 进行赋值
            for  H = Hrange(srangeIndx)
                Hindx = Hindx+1; % NPS 编号累加
                eval_abs(Hindx,dltindx) = NaN;
            end
            srangeInd = find(Hrange>RD*sin(dlt)) ; % find 的输出结果是向量里面的位置号
            for  H = Hrange(srangeInd) % 偏移量0-最大值20
                Hindx = Hindx + 1; % NPS 编号累加
            % 在此处添加了if语句, 让vdm不要超能力僭越, 剥夺Pgen地盘, 对比eval_abs偏差的时候用到了三次方来扩大1以上的误差, 让误差的计算非线性
            % 但此操作会增加1以内的误差容忍度。还需要补充【airtificial高压力，零功率】的许可            
                    if 0 % abs(gama(Hindx,dltindx)-pres_gama_Ratio)>=gama_bar_tolerence
                        eval_abs(Hindx,dltindx) = NaN; % 不允许超能力，即shaft侧送来500kW, 不允许压缩机工作在501kW; 但是膨胀机允许这种超能力
                    else            
                        eval_abs(Hindx,dltindx) = err_amplifier*(abs((gama(Hindx,dltindx) - pres_gama_Ratio)))...
                            +1/err_amplifier*(abs((PvdmVp(Hindx,dltindx)-Pvdmneed)))+1/10000*(abs((Hindx-Hindx_ref))); % 添加了微小系数消除evl_abs在0附近的多值
                    end
            end

        end
        % 循环赋值 eval_abs完毕, 可以进行判定
        min_eval_abs = min(min(eval_abs,[],1),[],2);
        [Hindx_ref, dltindx_ref] = find(eval_abs == min_eval_abs);
        H_ref = - Hrange(Hindx_ref);  % 膨胀机的delta和H设为负值, 容易区分压缩和膨胀过程
        dlt_ref = - dltrange(dltindx_ref); % 也可以把弧度转换成角度radtodeg(dlt_ref)
        gama_ref = gama(Hindx_ref, dltindx_ref);
        Tqd_ref = TqdVp(Hindx_ref, dltindx_ref);
        VDMspd_ref = VDMspdVp(Hindx_ref, dltindx_ref);
        Pvdm_ref = PvdmVp(Hindx_ref, dltindx_ref);

%                 gama_ref_box=gama(min(Hindx,max(1,Hindx_ref+(-1:1))),min(dltindx,max(1,dltindx_ref+(-1:1))))% 看看局部微观
%                 Pvdm_ref_box=PvdmVp(min(Hindx,max(1,Hindx_ref+(-1:1))),min(dltindx,max(1,dltindx_ref+(-1:1))))%% 看看局部微观

        % figure %绘制evaluation曲面
        % surf(dltrange,Hrange,eval_abs,'edgecolor','none') 

%         figure %绘制两个等高线，以及认定的交点
%         hold on
%         contour(dltrange,Hrange,gama,[pres_gama_Ratio,pres_gama_Ratio],'b--')  %压缩比和力矩等高线 scale down 到 p.u.值之后重叠找焦点。
%         contour(dltrange,Hrange,PvdmVp,[Pvdmneed,Pvdmneed],'b-','linewidth',1)
%         scatter(dltrange(dltindx_ref),Hrange(Hindx_ref),'r')%功率最大点对应转速
%         % title('Torque contour')
%         set(gcf,'renderer','zbuffer');
%         hold off


end




%--------------------------------------------------------------------------------------------后面是冗余重复部分，准备删除--------------------------------------------------------

                    % % % % % else
                    % % % % % %第三分支：VDM stand still mode
                    % % % % %                 Hindx_ref=Hindx_ref;
                    % % % % %                 dltindx_ref= 1;
                    % % % % %                 H_ref=Hrange(Hindx_ref);
                    % % % % %                 dlt_ref=dltrange(dltindx_ref) ;% 也可以把弧度转换成角度radtodeg(dlt_ref)
                    % % % % %                 gama_ref=gama(Hindx_ref,dltindx_ref);
                    % % % % %                 Tqd_ref=TqdVp(Hindx_ref,dltindx_ref);
                    % % % % %                 VDMspd_ref=VDMspdVp(Hindx_ref,dltindx_ref);
                    % % % % %                 Pvdm_ref=PvdmVp(Hindx_ref,dltindx_ref);