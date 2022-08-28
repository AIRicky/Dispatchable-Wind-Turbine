%% Function: ����blade����, ͨ���������, ����� blade �趨���ʣ�
% blade ���� �����ɻ�ù��ʱ�ֵ, �õ���Ӧ�� prim shaftת�ٺ� blade angle

%% �����ѻ��Cp-curve
% clc % CLean Command windows
% close all

artificial_gearR = 100; % ���ٳ�����                                        
%   V_wind=12;
%   Pblade_cal=min(P_speed_power(V_wind),Pblade_rated); % �Ѿ������λ��blade power curve
%   Pvdmneed=min(Pblade_cal-Pgen_rated/etaG,Pvdmin_rated); % �ͳ���vdm������, ��λW
%   Pvdm_ref=.6*Pvdmneed; % ��vdm����, �õ�֤ʵ����, ��λW
% % ��α�֤Pvdm_ref�ľ���ֵһ��С�ڵ���Pvdmneed

if  (V_wind >= vin) && (V_wind <= voff) 
    Pblade_ref = min(Pblade_cal, P_load_ref/etaG + Pvdm_ref);
    Pgenin_ref = Pblade_ref-Pvdm_ref; 
     %% begein  ����һ�����ε��������ټ������
        if (Pgenin_ref - P_load_ref/etaG < 0) && (Pvdm_ref > 0) % �ж�������û������loadͬʱ�ָ�power��compressor
            vdm_err_capacity = min(Pvdm_ref, P_load_ref/etaG - Pgenin_ref); % ������һ����misplaced������
            Pgenin_ref = Pgenin_ref + vdm_err_capacity ; %���ظ�����
            Pvdm_ref = Pvdm_ref - vdm_err_capacity; %/etaVp ��comprssor�����п۳�
        else
        end

        if (Pblade_cal - Pblade_ref > 0) && (Pvdm_ref < 0) % ��������֮��, ������ expander
            vdm_err_capacity = min(Pblade_cal-Pblade_ref,-Pvdm_ref); % ������һ���� misplaced ������
            Pblade_ref = Pblade_ref + vdm_err_capacity; % ���ظ�����, ������� blade_ref
            Pvdm_ref = Pvdm_ref + vdm_err_capacity; %/etaVp ��comprssor�����п۳�
        else
        end                
             %% end     ����һ�����ε��������ټ������
    Cp_cal = Cp_max*Pblade_ref/P_speed_power(V_wind);
    [Cp_err,Cp_indx] = min(abs(optCp-Cp_cal)); %  R0011-B������ӽ��Ľ����          
             
else %  С��cut-in spd ���� ����cut-out
    Pblade_ref = 0;
    Cp_cal = NaN; % ����д���ֵ24��degree
%     display('High/Low wind:shutdown');
    Pgenin_ref = -Pvdm_ref; % �� VDM ����
    Cp_indx = (V_wind < vin)*1 + (V_wind > voff)*length(optCp); % beta�Ƕ�, Ҫô���, Ҫô��С
    % Cp_indx=Cp_indx; %֮ǰ�����Ǻ��ϴνǶ�һ��
    % һ��ѡ����blade pitch�ͱ������һ�ο��õĽǶ�, Cp_indx=1 or 226, ��һ�ַ�ʽ�ǵ���cut-in��Ӧ��Сpitch, ����cut-off��Ӧ���pitch
end
% Pblade_cal
% Cp_cal
% Pblade_ref=Pblade_ref/1000 % ������ʾֵw-kw
% Pvdm_ref=-Pvdm_ref/1000  % ������ʾֵw-kw, ����ֵ����
% Pgenin_ref=Pgenin_ref/1000  % ������ʾֵw-kw

%% ����ҶƬ���ƵĲ�����Cpֵ���Լ���Ӧ��beta��numda �� prim shaft rotation speed
% Cp_err=min(abs(optCp-Cp_cal)) %  R0011-A
% Cp_indx=find(abs(optCp-Cp_cal)==Cp_err)%  R0011-A
Cp_ref = optCp(Cp_indx);   
beta_ref = beta_range(Cp_indx);
numda_ref = optnumda(Cp_indx); % Cpֵ���㲻׼ȷ��Ӱ��numbda��ȡֵ, �Ӷ�Ӱ��ҶƬ�͵��ת�ٵ��趨
omg_shaft1_ref = numda_ref*min(max(V_wind,vin),voff)/radius_Balde*artificial_gearR; % ��λ������ rad/s, ȡ�����������ʱ����� numda


% figure  % Cp һάչ����֤��������
% plot(optCp)
% hold on
% scatter(Cp_indx,Cp_ref,'r')
% hold off


%% ���������أ�����Ӧ������
Tq_gen_ref = Pgenin_ref/omg_shaft1_ref;

%% ����CVT���ٱ�
CVT_Ratio_ref = omg_shaft1_ref/VDMspd_ref;

%% ������ѹ����
E_tank = pres_gama_Ratio*p0_c2*VtankL*log(pres_gama_Ratio)-E_tankLower; % ��֪��ѹ��, ����������, Etank����������
E_tank = etaT*E_tank+(Pvdm_ref>=0)*Pvdm_ref*etaVc*T_step+(Pvdm_ref<0)*Pvdm_ref/etaVp*T_step; % ��������update
E_tank = max(0,min(E_tank,E_tankL));
pres2nergy=@(xgama)  xgama*p0_c2*VtankL*log(xgama)-(E_tank+E_tankLower); % ������ѹ��������, ���ڼ�����ѹ����, ��bladecall�������
% (E_tank+E_tankLower)��ʾ��������
pres_gama_Ratio = fzero(pres2nergy,pres_gama_Ratio);
