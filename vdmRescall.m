%% ����vdm�ڵ�ǰ��ѹ�����µ�ʣ������

% ���ܸ���ʱ, rev=����������ȥ���õ������
P_blade_surplus = max(0,min(Pvdm_gap, (Pgen_rated - P_load_ref)/etaG)); % ������ܺ͵��ʣ������, ȡС
dltindx = 0; % wobble angle ��ʼ�������
for dlt = dltrange  % wobble angle 0��-45��
    dltindx = dltindx+1;  % wobble angle ����ۼ�
    Hindx = 0;% NPS ��ʼ�������

    % ��ɨ��㲹ȫ��
    srangeIndx = find(Hrange<=RD*sin(dlt)); % ͨ�������������ҵ������и��ӱ��, ���и�ֵ
    for  H = Hrange(srangeIndx)
        Hindx = Hindx + 1; % NPS ����ۼ�
        eval_reserves(Hindx,dltindx) = NaN;
    end
    srangeInd = find(Hrange>RD*sin(dlt)) ; % find �������������������λ�ú�
    for  H = Hrange(srangeInd) % ƫ����0-���ֵ20
        Hindx = Hindx + 1; % NPS ����ۼ�
        eval_reserves(Hindx,dltindx) = err_amplifier*(abs((gama(Hindx,dltindx)-pres_gama_Ratio)))...
            + 1/err_amplifier*(abs((PvdmVp(Hindx,dltindx)-Pvdm_suply_negMax)))+1/10000*(abs((Hindx-Hindx_ref))); % �����΢Сϵ������evl_abs��0�����Ķ�ֵ
    end

end
% ѭ����ֵeval_abs���, �����ж�
min_eval_resv = min(min(eval_reserves,[],1),[],2);
[Hindx_Mref,dltindx_Mref] = find(eval_reserves == min_eval_resv);
% ����Ƿȱʱ, rev=���������ȥ��������, Ȼ��������������ȡС
P_vdm_surplus = -(PvdmVp(Hindx_Mref, dltindx_Mref) - min(0,Pvdm_ref)); % ���������������������ȥ�Ѿ�ʹ�õ�����, �Ӹ����л�������
P_spin_Resv = min((P_vdm_surplus + P_blade_surplus)*etaG,(Pgen_rated - P_load_ref)); % Ȼ���õ����������λ
