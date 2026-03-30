clear; clc; close all;

%% 1. 파라미터 설정
V = 20.0;             % 속도 (m/s)
sigma_p0_deg = 60;  % 초기 Lead Angle (deg)
sigma_pc_deg = 60;  % 명령 Lead Angle (deg)
k_gain = 3.0;         % 유도 이득
dt = 0.001;            % Time step (s)
g = 9.81;             % 중력 가속도
acc_limit = 1 * g;    % 가속도 제한 (2g)

% 라디안 변환
sigma_p0 = deg2rad(sigma_p0_deg);
sigma_pc = deg2rad(sigma_pc_deg);
psi_t = pi / 2;       % Target Heading (90도, +y 방향)

%% 2. 각도 범위 설정 (2사분면, 불가능 영역 제외)
% Target 기준 90도 ~ 150도 (pi/2 ~ pi - sigma_p0)
theta_start = deg2rad(90);
theta_end = pi - sigma_p0;

%% [Region Map 계산]
% r 범위를 1m ~ 20m (필요하면 더 넓게)로 설정해서 격자 생성
r_range = linspace(1, 200, 1000); % r 범위를 1m ~ 20m로 설정
theta_range = linspace(theta_start, theta_end, 1000); % 각도 범위 설정
[R_grid, Theta_grid] = meshgrid(r_range, theta_range); % 극좌표계 방식으로 격자 생성

% 1) 전체 격자에 대해 초기 위치 및 각도 계산
Px_grid = R_grid .* cos(Theta_grid);
Py_grid = R_grid .* sin(Theta_grid);
Lam_grid = Theta_grid + pi;
Psi_p_grid = sigma_p0 + Lam_grid;

% 2) Kinematics (t=0)
% Lam_dot 계산 (행렬 연산 .* ./ 사용)
Lam_dot_grid = (V ./ R_grid) .* (-sin(Lam_grid - psi_t) - sin(Psi_p_grid - Lam_grid));
U_curr_grid = Lam_dot_grid - k_gain * (sigma_p0 - sigma_pc);

% 3) t = 0.01s 업데이트 (Euler Integration)
R_new_grid = R_grid + (V * (cos(Lam_grid - psi_t) - cos(Psi_p_grid - Lam_grid))) * dt;
Lam_new_grid = Lam_grid + Lam_dot_grid * dt;
Psi_p_new_grid = Psi_p_grid + U_curr_grid * dt;

% 4) 가속도 재계산 (전체 영역)
Sigma_p_new_grid = Psi_p_new_grid - Lam_new_grid;
Lam_dot_new_grid = (V ./ R_new_grid) .* (-sin(Lam_new_grid - psi_t) - sin(Psi_p_new_grid - Lam_new_grid));
Acc_cmd_grid = V * (Lam_dot_new_grid - k_gain * (Sigma_p_new_grid - sigma_pc));

%% [Visualization] 그래프 그리기
figure('Name', 'Saturation Map Comparison', 'Theme', 'light', 'Position', [100, 100, 1200, 800]);
hold on; grid on; axis equal;
title('Acceleration Saturation Region Map');

% 등고선 그리기 (Contourf)
% 레벨: 2g 경계를 기준으로 나눔
% contourf로 영역 채우기 (절댓값 사용)
[C, h] = contourf(Px_grid, Py_grid, abs(Acc_cmd_grid), [acc_limit acc_limit], 'LineColor', 'none');

% 색상 매핑 (커스텀: 안전=하늘색, 위험=빨간색)
% 0(안전) ~ 2g(경계) ~ 그 이상(위험)
colormap([0.6 0.8 1.0; 1.0 0.6 0.6]); 
caxis([0, 2*acc_limit]); % 색상 축 고정

% Target 위치 표시
quiver(0, 0, 0, 5, 'k', 'LineWidth', 2);
text(1, 5, 'Target', 'FontSize', 12, 'FontWeight', 'bold');

% 축 및 범례 설정
xlabel('x (m)'); ylabel('y (m)');
% txt = text(-18, 15, 'Red: Unsafe (>2g)\nBlue: Safe');
% set(txt, 'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', 'w');

% 보기 좋게 범위 조절 (원하는 대로 바꿔도 돼 선생)
xlim([-40, 5]); ylim([-5, 40]);

% 경계선만 따로 빨갛게 그려서 강조하고 싶으면 주석 풀어서 써~
% contour(Px_grid, Py_grid, abs(Acc_cmd_grid), [acc_limit acc_limit], 'r', 'LineWidth', 2);