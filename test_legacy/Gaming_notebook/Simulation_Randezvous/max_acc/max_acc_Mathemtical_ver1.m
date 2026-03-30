clear; clc; close all;

%% 1. 기본 파라미터 설정
V = 20.0;             % 속도 (m/s)
k_gain = 3.0;         % 유도 이득
g = 9.81;             % 중력 가속도
acc_limit = 1 * g;    % 가속도 제한 (2g)
psi_t = pi / 2;       % Target Heading (90도, +y 방향)

% 기준 명령 각도 (Command Lead Angle)
sigma_pc_deg = 60.0;  
sigma_pc = deg2rad(sigma_pc_deg);

% % 선생이 요청한 6가지 케이스 (Offset)
% offsets = [-25, -20, -15, -10, -5, 0]; 

%% 2. 격자(Meshgrid) 생성 (공통)
% R: 1m ~ 100m, Theta: 90도 ~ 180도
r_range = linspace(1, 200, 500); 
theta_range = linspace(deg2rad(90), deg2rad(180), 400);
[R_grid, Theta_grid] = meshgrid(r_range, theta_range);

% 좌표 변환
Px_grid = R_grid .* cos(Theta_grid);
Py_grid = R_grid .* sin(Theta_grid);

% 기하학적 각도 계산 (Sigma_t는 타겟/위치에 따라 고정)
Lam_grid = Theta_grid + pi;       % 시선각
Sigma_t_grid = psi_t - Lam_grid;  % 타겟 리드각 (Lead Angle of Target)

%% 3. 6개 서브플롯 생성 루프
figure('Name', 'Saturation Map Comparison', 'Theme', 'light', 'Position', [100, 100, 1200, 800]);

for i = 1:6
    % 현재 케이스의 sigma_p 계산
    current_offset = offsets(i);
    sigma_p_deg = sigma_pc_deg + current_offset;
    sigma_p = deg2rad(sigma_p_deg);
    
    % --- 가속도 계산 (Algebraic Formula) ---
    % Term 1: 기하학적 회전 성분 (V^2/r * (sin(sigma_t) - sin(sigma_p)))
    Term1 = (V^2 ./ R_grid) .* (sin(Sigma_t_grid) - sin(sigma_p));
    
    % Term 2: 피드백 제어 성분 (V * k * (sigma_p - sigma_pc))
    Term2 = V * k_gain * (sigma_p - sigma_pc);
    
    % 최종 가속도 명령
    Acc_cmd_grid = Term1 - Term2;
    % ---------------------------------------
    
    % 서브플롯 배치 (2행 3열)
    subplot(2, 3, i); hold on; grid on; axis equal;
    
    % 제목: 현재 sigma_p와 오프셋 표시
    title(sprintf('Case %d: \\sigma_p = %.0f^\\circ (Offset %+d)', i, sigma_p_deg, current_offset), 'FontSize', 11);
    
    % 영역 그리기 (Contourf)
    % 0~2g(안전) -> 파랑, 2g 이상(위험) -> 빨강
    contourf(Px_grid, Py_grid, abs(Acc_cmd_grid), [0, acc_limit, 1e5], 'LineColor', 'none');
    
    % 색상맵 및 축 설정
    colormap([0.7 0.8 1.0; 1.0 0.7 0.7]); % 연한 파랑 / 연한 빨강
    caxis([0, 2*acc_limit]); 
    
    % 경계선 강조
    contour(Px_grid, Py_grid, abs(Acc_cmd_grid), [acc_limit acc_limit], 'r', 'LineWidth', 1.5);
    
    % Target 화살표
    quiver(0, 0, 0, 15, 'k', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    
    % 축 범위 (보기 좋게 조절)
    xlim([-200, 10]); ylim([-10, 200]);
    if i > 3, xlabel('x (m)'); end
    if mod(i, 3) == 1, ylabel('y (m)'); end
end

% 전체 타이틀 (Suptitle 대용)
sgtitle(['Acceleration Saturation Maps based on \sigma_p variations (\sigma_{pc} = ' num2str(sigma_pc_deg) '^\circ)'], 'FontSize', 14, 'FontWeight', 'bold');

% 범례 설명용 더미 축
% (오른쪽 아래 빈 공간에 설명 추가)
axes('Position', [0.85 0.1 0.1 0.1], 'Visible', 'off');
text(0, 0.6, 'Blue: Safe', 'Color', 'b', 'FontWeight', 'bold', 'FontSize', 10);
text(0, 0.3, 'Red: Unsafe', 'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);