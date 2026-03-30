clear; clc; close all;

%% 1. 기본 파라미터 설정
V = 20.0;             % 속도 (m/s)
k_gain = 3.0;         % 유도 이득
g = 9.81;             % 중력 가속도
acc_limit = 1 * g;    % 가속도 제한 (1g)
psi_t = pi / 2;       % Target Heading (90도, +y 방향)

% 기준 명령 각도 (Command Lead Angle) - 이 값들에 따라 6개 플롯 생성
sigma_pc_deg_list = [0 15 30 45 60 75];  
sigma_pc_rad_list = deg2rad(sigma_pc_deg_list);

%% 2. 격자(Meshgrid) 생성 (공통)
% R: 1m ~ 200m, Theta: 90도 ~ 270도
r_range = linspace(1, 200, 500); 
theta_range = linspace(deg2rad(90), deg2rad(270), 400);
[R_grid, Theta_grid] = meshgrid(r_range, theta_range);

% 좌표 변환
Px_grid = R_grid .* cos(Theta_grid);
Py_grid = R_grid .* sin(Theta_grid);

% 기하학적 각도 계산
Lam_grid = Theta_grid + pi;       % 시선각
Sigma_t_grid = psi_t - Lam_grid;  % 타겟 리드각

%% 3. Custom Colormap 생성 (색상 꼬임 완벽 방지)
% -g 영역(보라색), Safe 영역(파란색), +g 영역(빨간색)이 정확하게 칠해지도록 설정
cmap = zeros(256, 3);
cmap(1:50, :)   = repmat([0.8 0.6 1.0], 50, 1);     % Purple (-g 영역)
cmap(51:180, :) = repmat([0.7 0.8 1.0], 130, 1);    % Blue (Safe 영역)
cmap(181:256, :) = repmat([1.0 0.7 0.7], 76, 1);    % Red (+g 영역)

%% 4. 6개 서브플롯 생성 루프
figure('Name', 'Saturation Map Comparison', 'Theme', 'light', 'Position', [100, 50, 1000, 850]);

for i = 1:6
    % 현재 케이스의 sigma_pc 및 sigma_p 설정
    current_sigma_pc = sigma_pc_rad_list(i);
    current_sigma_p = current_sigma_pc; 
    
    % --- 가속도 계산 ---
    Term1 = (V^2 ./ R_grid) .* (sin(Sigma_t_grid) - sin(current_sigma_p));
    Term2 = V * k_gain * (current_sigma_p - current_sigma_pc);
    Acc_cmd_grid = Term1 - Term2;
    % ---------------------------------------
    
    % 서브플롯 배치 (2행 3열)
    subplot(2, 3, i); hold on; grid on; axis equal;
    
    % 제목: 현재 sigma_pc 값 표시
    title(sprintf('Case %d: \\sigma_{pc} = %.0f^\\circ', i, sigma_pc_deg_list(i)), 'FontSize', 11);
    
    % 영역 그리기 (Contourf) - 원본 데이터를 그대로 사용해서 경계 오차 제거
    % 범위를 [-100g, -1g, 1g, 100g]로 설정해서 구간을 딱 3개로만 나눔
    contourf(Px_grid, Py_grid, Acc_cmd_grid, [-100*acc_limit, -acc_limit, acc_limit, 100*acc_limit], 'LineColor', 'none');
    
    % 수제 색상맵 및 축 설정 적용
    colormap(gca, cmap);
    clim(gca, [-2*acc_limit, 2*acc_limit]); 
    
    % 경계선 강조 (색칠 경계와 완벽하게 일치하는지 확인 가능)
    contour(Px_grid, Py_grid, Acc_cmd_grid, [acc_limit acc_limit], 'r', 'LineWidth', 1.5);
    contour(Px_grid, Py_grid, Acc_cmd_grid, [-acc_limit -acc_limit], 'm', 'LineWidth', 1.5);
    
    % Target 화살표
    quiver(0, 0, 0, 15, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    
    % 축 범위
    xlim([-50, 5]); ylim([-5, 50]);
    if i > 3, xlabel('x (m)'); end
    if mod(i, 3) == 1, ylabel('y (m)'); end
end

% 전체 타이틀
sgtitle('Acceleration Saturation Map(Mathemetical) (\sigma_{p} = \sigma_{pc})', 'FontSize', 14, 'FontWeight', 'bold');

% 범례 설명용 더미 축
axes('Position', [0.85 0.1 0.1 0.1], 'Visible', 'off');
text(0, 0.9, 'Red: Unsafe (+g)', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);
text(0, 0.6, 'Blue: Safe', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 10);
text(0, 0.3, 'Purple: Unsafe (-g)', 'Color', 'm', 'FontWeight', 'bold', 'FontSize', 10);