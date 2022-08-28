%% bladeSurfs

% ����Cp curve����beta�仯��surface
% WT�ڸ���beta�����µ�MPPT����, x/y/z�ֱ���omg/beta/Cp
Cp_curve_option = 1;
% clc
% clear all
% close all
[~, ~, raw1] = xlsread('controlTransR1.xlsx','Sheet1','B2:D26'); % �������ϵ��(�����в���)
        cpfitcoef = reshape([raw1{:}],size(raw1));
        ni = transpose(cpfitcoef(1:25,1));
        nj = transpose(cpfitcoef(1:25,2));
        naij = transpose(cpfitcoef(1:25,3));
n_beta = 0;

beta_range = 0:.1:22.5; % ����24��degree, Cp���ǵ���������, ͬһ��Ч�ʵ��Ӧ����ת��
numda_range_GE = .5:.1:15; % Ҷ���ٱ�
numda_scale_coeff = 1;
numda_range_CA = numda_range_GE * numda_scale_coeff; 

for beta = beta_range % y��beta����, n-beta������λ��, beta�ǵ�ֵ
    % �������ֵ beta ��ϵı���ȡ�Ƕ�
    n_beta = n_beta + 1; % ����betaָ����
    n_numda = 0;
    
    for numda = numda_range_GE % x��numda����, n-numda������λ��, numda�ǵ�ֵ
        n_numda = n_numda + 1;
        
        sumaij = 0;
        for nn = 1:5*5
           sumaij = sumaij + naij(nn)*beta^ni(nn)*numda^nj(nn); % ����Cpֵ, ���ϵ���Ӻ�
        end
        Cpsurface(n_beta,n_numda) = max(sumaij,0); % ��������ֵ
    end 
    [optCp(n_beta),n_optnumda] = max(Cpsurface(n_beta,:)); % ��ÿһ��beta������, �����һ��
    optnumda(n_beta) = numda_range_CA(n_optnumda);
end

%% ��ͼ��֤Cp�Ķ�ά����ά����
 if Cp_curve_option 
    figure
    hold on
    subplot(2,1,1) % ��Cp surfaceͼ ��optimal Cp���ߵ���ͼ
    % ���Գ���ʹ�� [X,Y] = meshgrid(-2:3,-5:0) ��������, ������
    surf(numda_range_CA, beta_range, Cpsurface) % Cp surface
    plot3(optnumda, beta_range, optCp) % Cp surface��ɽ��
    shading interp
    xlabel('\omega (m/s)')
    ylabel('\beta (^{\circ}C)')
    zlabel('C_p')
    hold off
    
    subplot(2,1,2) % Cp surface���еȸ���
    hold on
    %contour(numda_range_CA,beta_range,Cpsurface) % �� Cp surface�ĵȸ���
    contour(numda_range_CA, beta_range, Cpsurface,'ShowText','on') % �� Cp surface�ĵȸ���
    plot(optnumda, beta_range,'r--') % ��Cp�ȸ��߶�άƽ������㻰ɽ��
    xlabel('\omega (m/s)')
    ylabel('\beta (^{\circ}C)')
    hold off
    
%         figure
%         % Cp һάչ����֤��������
%         plot(optCp) % ��������Ҫ����һ��
%         % ���������ķ�Χ�����0.5173--0.0888
 end
