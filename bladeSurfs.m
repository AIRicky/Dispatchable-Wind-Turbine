%% bladeSurfs

% 画出Cp curve随着beta变化的surface
% WT在各种beta条件下的MPPT控制, x/y/z分别是omg/beta/Cp
Cp_curve_option = 1;
% clc
% clear all
% close all
[~, ~, raw1] = xlsread('controlTransR1.xlsx','Sheet1','B2:D26'); % 导入拟合系数(文献中参数)
        cpfitcoef = reshape([raw1{:}],size(raw1));
        ni = transpose(cpfitcoef(1:25,1));
        nj = transpose(cpfitcoef(1:25,2));
        naij = transpose(cpfitcoef(1:25,3));
n_beta = 0;

beta_range = 0:.1:22.5; % 超过24度degree, Cp不是单调函数了, 同一个效率点对应两个转速
numda_range_GE = .5:.1:15; % 叶尖速比
numda_scale_coeff = 1;
numda_range_CA = numda_range_GE * numda_scale_coeff; 

for beta = beta_range % y是beta向量, n-beta是数据位标, beta是单值
    % 假设这个值 beta 拟合的变量取角度
    n_beta = n_beta + 1; % 建立beta指针编号
    n_numda = 0;
    
    for numda = numda_range_GE % x是numda向量, n-numda是数据位标, numda是单值
        n_numda = n_numda + 1;
        
        sumaij = 0;
        for nn = 1:5*5
           sumaij = sumaij + naij(nn)*beta^ni(nn)*numda^nj(nn); % 计算Cp值, 拟合系数加和
        end
        Cpsurface(n_beta,n_numda) = max(sumaij,0); % 消除负数值
    end 
    [optCp(n_beta),n_optnumda] = max(Cpsurface(n_beta,:)); % 在每一个beta条件下, 都算出一个
    optnumda(n_beta) = numda_range_CA(n_optnumda);
end

%% 作图验证Cp的二维和三维曲面
 if Cp_curve_option 
    figure
    hold on
    subplot(2,1,1) % 画Cp surface图 和optimal Cp曲线叠加图
    % 可以尝试使用 [X,Y] = meshgrid(-2:3,-5:0) 建立网格, 简化运算
    surf(numda_range_CA, beta_range, Cpsurface) % Cp surface
    plot3(optnumda, beta_range, optCp) % Cp surface的山脊
    shading interp
    xlabel('\omega (m/s)')
    ylabel('\beta (^{\circ}C)')
    zlabel('C_p')
    hold off
    
    subplot(2,1,2) % Cp surface横切等高线
    hold on
    %contour(numda_range_CA,beta_range,Cpsurface) % 画 Cp surface的等高线
    contour(numda_range_CA, beta_range, Cpsurface,'ShowText','on') % 画 Cp surface的等高线
    plot(optnumda, beta_range,'r--') % 在Cp等高线二维平面上描点话山脊
    xlabel('\omega (m/s)')
    ylabel('\beta (^{\circ}C)')
    hold off
    
%         figure
%         % Cp 一维展开，证明单调性
%         plot(optCp) % 横坐标需要调整一下
%         % 最后算出来的范围大概是0.5173--0.0888
 end
