%% Function: ��֪VDMȫ����������������, �������������
% Note: 1) Pvdm_need ��Ϊ Gearbox �๦��;
%       2) �Ա� eval_abs ƫ��ʱ�õ������η�, ������1���ϵ����, �����ļ��������
%       3) �� run �������ɵĺ���, ���ǵ�����һ���ݿ����֪����, ȷ��������߹�����
%       4) ��ͼ˵������ȡ����ĵȸ���, �ص�����, Ŀ�⽻���; ��һ·���㽻���, ��ԲȦ���, ����֤һ��
%       5) if-else-end ģʽ��, ������֧��ȫ�Գ�, ��Ӧ EITHER compressor OR expander
%       6) ���ֲ�����Ҫ�Ǽ���������, ÿ��ִֻ��һ����֧

%% ��Ҫ��������, ��ΪҪ������������
% clc
% close all

%             pres_gama_Ratio=6.7999; % ������ѹֵ
%             Pvdmneed=360.62*1000;   % ���� VDM ��������ֵ
%             Hindx_ref = 0;          % �趨һ����ʼֵ, ��ѡ����ƽ��ֵ֮��, ѡ������ϴ� H ����ĵط�

% ��ʼһ���ж����, ��ÿ�ν���EITHER comprssor OR expander
err_amplifier = 1000;

% gama_bar_tolerence=0.25;

if Pvdmneed >= 0
% ��һ��֧��compression mode
        dltindx = 0; % wobble angle ��ʼ�������
        for dlt = dltrange  % wobble angle 0��-45��
            dltindx = dltindx + 1;  % wobble angle ����ۼ�
            Hindx = 0;% NPS ��ʼ�������

            % ��ȫɨ���
            srangeIndx = find(Hrange <= RD*sin(dlt)); % ͨ����������, �ҵ������и��ӱ�Ų����и�ֵ
            for  H = Hrange(srangeIndx)
                Hindx = Hindx + 1; % NPS ����ۼ�
                eval_abs(Hindx,dltindx) = NaN; % wobble plate ײ���׵�, ����������, ��ֵΪNaN
            end
            srangeInd = find(Hrange > RD*sin(dlt)); % find �������������������λ�ú�
            for  H = Hrange(srangeInd) % ƫ����0-���ֵ20
                Hindx = Hindx+1; % NPS ����ۼ�
            % �˴������ if ���, �� VDM��Ҫ��������Խ, ���� Pgen ����
            % �Ա� eval_abs ƫ��ʱ�õ������η�, ������ 1 ���ϵ����, �����ļ��������, ���˲��������Ľ���ǻ�����1���ڵ�������̶�
            % ����Ҫ���䡾airtificial ��ѹ��, �㹦�ʡ������
                    if 0 %(PvdmVc(Hindx,dltindx)-Pvdmneed)>0 % || abs(gama(Hindx,dltindx)-pres_gama_Ratio)>=gama_bar_tolerence
                        eval_abs(Hindx,dltindx) = NaN;% ����������, ��shaft������500kW, ������ѹ����������501kW��; �������ͻ��������ֳ�����
                    else
                        eval_abs(Hindx,dltindx) = err_amplifier*(abs((gama(Hindx,dltindx)-pres_gama_Ratio)))...
                            + 1/err_amplifier * (abs((PvdmVc(Hindx,dltindx)-Pvdmneed))) + 1/10000*(abs((Hindx-Hindx_ref))); % �����΢Сϵ�������� evl_abs ��0�����Ķ�ֵ
%                         eval_abs(Hindx,dltindx)=err_amplifier*(abs((gama(Hindx,dltindx)-pres_gama_Ratio)))^3+1/err_amplifier*(abs((PvdmVc(Hindx,dltindx)-Pvdmneed))); % 
                    end

            end

        end
        % ѭ����ֵ eval_abs ���, �����ж�
        min_eval_abs = min(min(eval_abs,[],1),[],2);
        [Hindx_ref, dltindx_ref] = find(eval_abs == min_eval_abs);% ֮���Զ���Ϊ buffer ����Ϊ����ֵ������һ��vector
        H_ref = Hrange(Hindx_ref);
        dlt_ref = dltrange(dltindx_ref);% Ҳ�ɰѻ���ת���ɽǶ� radtodeg(dlt_ref)
        gama_ref = gama(Hindx_ref,dltindx_ref);
        Tqd_ref = TqdVc(Hindx_ref,dltindx_ref);
        VDMspd_ref = VDMspdVc(Hindx_ref,dltindx_ref);
        Pvdm_ref = PvdmVc(Hindx_ref,dltindx_ref);     
%                 gama_ref_box=gama(min(Hindx,max(1,Hindx_ref+(-1:1))),min(dltindx,max(1,dltindx_ref+(-1:1))))% �����ֲ�΢��
%                 Pvdm_ref_box=PvdmVc(min(Hindx,max(1,Hindx_ref+(-1:1))),min(dltindx,max(1,dltindx_ref+(-1:1))))%% �����ֲ�΢��

       %% ��ͼ��֤��������
        % figure %����evaluation����
        % surf(dltrange,Hrange,eval_abs,'edgecolor','none') 

%         figure %���������ȸ��ߣ��Լ��϶��Ľ���
%         hold on
%         contour(dltrange,Hrange,gama,[pres_gama_Ratio,pres_gama_Ratio],'b--')
%         %ѹ���Ⱥ����صȸ��� scale down �� p.u.ֵ֮���ص��ҽ���
%         contour(dltrange,Hrange,PvdmVc,[Pvdmneed,Pvdmneed],'b-','linewidth',1)
%         scatter(dltrange(dltindx_ref),Hrange(Hindx_ref),'r')%���������Ӧת��
%         % title('Torque contour')
%         set(gcf,'renderer','zbuffer');
%         hold off

else % elseif Pvdmneed<0
% �ڶ���֧��expansion mode
        dltindx = 0; % wobble angle ��ʼ�������
        for dlt = dltrange  % wobble angle 0��-45��
            dltindx = dltindx+1;  % wobble angle ����ۼ�
            Hindx = 0;% NPS ��ʼ�������

            % ��ɨ��㲹ȫ��
            srangeIndx = find(Hrange<=RD*sin(dlt)); % ͨ����������, �ҵ������и��ӱ��, ���и�ֵ
            for  H = Hrange(srangeIndx)
                Hindx = Hindx+1; % NPS ����ۼ�
                eval_abs(Hindx,dltindx) = NaN;
            end
            srangeInd = find(Hrange>RD*sin(dlt)) ; % find �������������������λ�ú�
            for  H = Hrange(srangeInd) % ƫ����0-���ֵ20
                Hindx = Hindx + 1; % NPS ����ۼ�
            % �ڴ˴������if���, ��vdm��Ҫ��������Խ, ����Pgen����, �Ա�eval_absƫ���ʱ���õ������η�������1���ϵ����, �����ļ��������
            % ���˲���������1���ڵ�������̶ȡ�����Ҫ���䡾airtificial��ѹ�����㹦�ʡ������            
                    if 0 % abs(gama(Hindx,dltindx)-pres_gama_Ratio)>=gama_bar_tolerence
                        eval_abs(Hindx,dltindx) = NaN; % ��������������shaft������500kW, ������ѹ����������501kW; �������ͻ��������ֳ�����
                    else            
                        eval_abs(Hindx,dltindx) = err_amplifier*(abs((gama(Hindx,dltindx) - pres_gama_Ratio)))...
                            +1/err_amplifier*(abs((PvdmVp(Hindx,dltindx)-Pvdmneed)))+1/10000*(abs((Hindx-Hindx_ref))); % �����΢Сϵ������evl_abs��0�����Ķ�ֵ
                    end
            end

        end
        % ѭ����ֵ eval_abs���, ���Խ����ж�
        min_eval_abs = min(min(eval_abs,[],1),[],2);
        [Hindx_ref, dltindx_ref] = find(eval_abs == min_eval_abs);
        H_ref = - Hrange(Hindx_ref);  % ���ͻ���delta��H��Ϊ��ֵ, ��������ѹ�������͹���
        dlt_ref = - dltrange(dltindx_ref); % Ҳ���԰ѻ���ת���ɽǶ�radtodeg(dlt_ref)
        gama_ref = gama(Hindx_ref, dltindx_ref);
        Tqd_ref = TqdVp(Hindx_ref, dltindx_ref);
        VDMspd_ref = VDMspdVp(Hindx_ref, dltindx_ref);
        Pvdm_ref = PvdmVp(Hindx_ref, dltindx_ref);

%                 gama_ref_box=gama(min(Hindx,max(1,Hindx_ref+(-1:1))),min(dltindx,max(1,dltindx_ref+(-1:1))))% �����ֲ�΢��
%                 Pvdm_ref_box=PvdmVp(min(Hindx,max(1,Hindx_ref+(-1:1))),min(dltindx,max(1,dltindx_ref+(-1:1))))%% �����ֲ�΢��

        % figure %����evaluation����
        % surf(dltrange,Hrange,eval_abs,'edgecolor','none') 

%         figure %���������ȸ��ߣ��Լ��϶��Ľ���
%         hold on
%         contour(dltrange,Hrange,gama,[pres_gama_Ratio,pres_gama_Ratio],'b--')  %ѹ���Ⱥ����صȸ��� scale down �� p.u.ֵ֮���ص��ҽ��㡣
%         contour(dltrange,Hrange,PvdmVp,[Pvdmneed,Pvdmneed],'b-','linewidth',1)
%         scatter(dltrange(dltindx_ref),Hrange(Hindx_ref),'r')%���������Ӧת��
%         % title('Torque contour')
%         set(gcf,'renderer','zbuffer');
%         hold off


end




%--------------------------------------------------------------------------------------------�����������ظ����֣�׼��ɾ��--------------------------------------------------------

                    % % % % % else
                    % % % % % %������֧��VDM stand still mode
                    % % % % %                 Hindx_ref=Hindx_ref;
                    % % % % %                 dltindx_ref= 1;
                    % % % % %                 H_ref=Hrange(Hindx_ref);
                    % % % % %                 dlt_ref=dltrange(dltindx_ref) ;% Ҳ���԰ѻ���ת���ɽǶ�radtodeg(dlt_ref)
                    % % % % %                 gama_ref=gama(Hindx_ref,dltindx_ref);
                    % % % % %                 Tqd_ref=TqdVp(Hindx_ref,dltindx_ref);
                    % % % % %                 VDMspd_ref=VDMspdVp(Hindx_ref,dltindx_ref);
                    % % % % %                 Pvdm_ref=PvdmVp(Hindx_ref,dltindx_ref);