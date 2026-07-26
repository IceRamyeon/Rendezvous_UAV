clear; clc; close all;

%% 1. 기본 파라미터 설정
V = 20.0;             % 속도 (m/s)
k_gain = 3.0;         % 유도 이득
dt = 0.01;           % Time step (s)
g = 9.81;             % 중력 가속도
acc_limit = 1 * g;    % 가속도 제한 (1g)
psi_t = pi / 2;       % Target Heading (90도, +y 방향)

% 기준 명령 각도 (Command Lead Angle) - 6개 케이스
sigma_pc_deg_list = [90 105 120 135 150 165]; 
sigma_pc_rad_list = deg2rad(sigma_pc_deg_list);

%% 2. 6개 서브플롯 생성 루프
figure('Name', 'Acceleration Saturation Map(Simulation)', 'Theme', 'light', 'Position', [100, 100, 1200, 800]);

for i = 1:6
    % 현재 케이스의 sigma_pc 및 초기값 sigma_p0 설정
    current_sigma_pc = sigma_pc_rad_list(i);
    sigma_p0 = current_sigma_pc; % sigma_p0 = sigma_pc 가정
    
    % --- [Region Map 계산] ---
    % r 범위를 1m ~ 200m
    r_range = linspace(1, 200, 200); 
    theta_range = linspace(deg2rad(90), deg2rad(270), 200); 
    [R_grid, Theta_grid] = meshgrid(r_range, theta_range);
    
    % 1) 전체 격자에 대해 초기 위치 및 각도 계산
    Px_grid = R_grid .* cos(Theta_grid);
    Py_grid = R_grid .* sin(Theta_grid);
    Lam_grid = Theta_grid + pi;
    Psi_p_grid = sigma_p0 + Lam_grid;
    
    % 2) Kinematics (t=0)
    % Lam_dot 계산
    Lam_dot_grid = (V ./ R_grid) .* (-sin(Lam_grid - psi_t) - sin(Psi_p_grid - Lam_grid));
    % 현재 입력 U 계산 (sigma_p = sigma_pc 이므로 k_gain 항은 사실상 0)
    U_curr_grid = Lam_dot_grid - k_gain * (sigma_p0 - current_sigma_pc);
    
    % 3) t = dt 업데이트 (Euler Integration) -> 시뮬레이션 예측값
    R_new_grid = R_grid + (V * (cos(Lam_grid - psi_t) - cos(Psi_p_grid - Lam_grid))) * dt;
    Lam_new_grid = Lam_grid + Lam_dot_grid * dt;
    Psi_p_new_grid = Psi_p_grid + U_curr_grid * dt;
    
    % 4) 가속도 재계산 (예측된 상태 기반)
    Sigma_p_new_grid = Psi_p_new_grid - Lam_new_grid;
    Lam_dot_new_grid = (V ./ R_new_grid) .* (-sin(Lam_new_grid - psi_t) - sin(Psi_p_new_grid - Lam_new_grid));
    
    % 최종 가속도 명령 (V * sigma_p_dot와 유사한 형태)
    Acc_cmd_grid = V * (Lam_dot_new_grid - k_gain * (Sigma_p_new_grid - current_sigma_pc));
    % ---------------------------------------
    
    % 서브플롯 배치 (2행 3열)
    subplot(2, 3, i); hold on; grid on; axis equal;
    
    % 제목 수정
    title(sprintf('Case %d: \\sigma_{p0} = %.0f^\\circ', i, sigma_pc_deg_list(i)), 'FontSize', 11);
    
    % 영역 그리기
    contourf(Px_grid, Py_grid, abs(Acc_cmd_grid), [0, acc_limit, 1e5], 'LineColor', 'none');
    
    % 색상맵 설정
    colormap([0.7 0.8 1.0; 1.0 0.7 0.7]); 
    clim([0, 2*acc_limit]); 
    
    % 경계선
    contour(Px_grid, Py_grid, abs(Acc_cmd_grid), [acc_limit acc_limit], 'r', 'LineWidth', 1.5);
    
    % Target 화살표
    quiver(0, 0, 0, 15, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    
    xlim([-50, 5]); ylim([-5, 50]);
    if i > 3, xlabel('x (m)'); end
    if mod(i, 3) == 1, ylabel('y (m)'); end
end

sgtitle(['Simulation Saturation Maps (\sigma_{p0} = \sigma_{pc})'], 'FontSize', 14, 'FontWeight', 'bold');

% 범례
axes('Position', [0.85 0.1 0.1 0.1], 'Visible', 'off');
text(0, 0.6, 'Blue: Safe', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 10);
text(0, 0.3, 'Red: Unsafe', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);