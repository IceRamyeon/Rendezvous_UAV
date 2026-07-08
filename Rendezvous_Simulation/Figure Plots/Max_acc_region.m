clear; clc; close all;

%% 1. 기본 파라미터 설정
V = 20.0;             % 속도 (m/s)
k_gain = 3.0;         % 유도 이득
g = 9.81;             % 중력 가속도
acc_limit = 1 * g;    % 가속도 제한 (1g)
psi_t = pi / 2;       % Target Heading (90도, +y 방향)

% 기준 명령 각도 (Command Lead Angle) - 0도, 20도, 40도로 변경
sigma_pc_deg_list = [0 20 40];  
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

%% 3. 개별 Figure 생성 루프 (빨간 영역만 도출)
for i = 1:length(sigma_pc_deg_list)
    % 현재 케이스의 sigma_pc 및 sigma_p 설정
    current_sigma_pc = sigma_pc_rad_list(i);
    current_sigma_p = current_sigma_pc; 
    
    % --- 가속도 계산 ---
    Term1 = (V^2 ./ R_grid) .* (sin(Sigma_t_grid) - sin(current_sigma_p));
    Term2 = V * k_gain * (current_sigma_p - current_sigma_pc);
    Acc_cmd_grid = Term1 - Term2;
    % ---------------------------------------
    
    % 파란색(Safe) 및 보라색(-g) 영역 제거 
    % (가속도 제한치인 acc_limit 미만의 값들을 모두 NaN 처리하여 그리지 않음)
    Acc_cmd_grid_red = Acc_cmd_grid;
    Acc_cmd_grid_red(Acc_cmd_grid_red < acc_limit) = NaN;
    
    % 각각 독립된 Figure 생성
    figure('Name', sprintf('Saturation Map (sigma_p = %d)', sigma_pc_deg_list(i)), ...
           'Theme', 'light', 'Position', [100+i*50, 100+i*50, 600, 500]);
    hold on; grid on; axis equal;
    
    % 제목
    % title(sprintf('Acceleration Saturation Map (+g Only)\n\\sigma_{p} = \\sigma_{pc} = %.0f^\\circ', sigma_pc_deg_list(i)), 'FontSize', 12, 'FontWeight', 'bold');
    
    % 빨간 영역(Unsafe +g)만 그리기 (Contourf)
    % 색상은 FaceColor로 옅은 빨간색 지정
    contourf(Px_grid, Py_grid, Acc_cmd_grid_red, [acc_limit, 100*acc_limit], 'FaceColor', [1.0 0.7 0.7], 'LineColor', 'none');
    
    % 경계선 강조
    contour(Px_grid, Py_grid, Acc_cmd_grid, [acc_limit acc_limit], 'r', 'LineWidth', 1.5);
    
    % 화살촉 모양의 Target 배치 (y = +-1m 높이, +y 방향)
    target_x = [0, 1, 0, -1];
    target_y = [1, -1, -0.2, -1];
    fill(target_x, target_y, 'k', 'EdgeColor', 'none');
    
    % 축 범위 및 라벨
    xlim([-50, 1]); ylim([-25, 25]);
    xlabel('x (m)'); ylabel('y (m)');
    
    % 텍스트 설명 추가
    % text(-48, 45, 'Red: Unsafe (+g)', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 11);
end