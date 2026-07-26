clear; clc; close all;

%% 1. 파라미터 설정 (으헤~ 기본 설정이야)
V = 20.0;             % 속도 (m/s)
r0_check = 10.0;      % 검증할 거리 (m)
sigma_p0_deg = 45.0;  % 초기 Lead Angle (deg)
sigma_pc_deg = 45.0;  % 명령 Lead Angle (deg)
k_gain = 3.0;         % 유도 이득
dt = 0.01;            % Time step (s)
g = 9.81;             % 중력 가속도
acc_limit = 2 * g;    % 가속도 제한 (2g)

% 라디안 변환
sigma_p0 = deg2rad(sigma_p0_deg);
sigma_pc = deg2rad(sigma_pc_deg);
psi_t = pi / 2;       % Target Heading (90도, +y 방향)

%% 2. 각도 범위 설정 (2사분면, 불가능 영역 제외)
% Target 기준 90도 ~ 150도 (pi/2 ~ pi - sigma_p0)
theta_start = deg2rad(90);
theta_end = pi - sigma_p0;

%% [Part 1] r = 10m 에서의 Arc Check (선생이 요청한 거)
% 벡터화 연산 (for 루프 대신 linspace 사용)
theta_arc = linspace(theta_start, theta_end, 200);

% 1) 초기 상태 계산
Px = r0_check * cos(theta_arc);
Py = r0_check * sin(theta_arc);
r = r0_check;
lam = theta_arc + pi; % LOS 각도
psi_p = sigma_p0 + lam;

% 2) t=0 Kinematics & Dynamics
r_dot = V * (cos(lam - psi_t) - cos(psi_p - lam));
lam_dot = (V / r) * (-sin(lam - psi_t) - sin(psi_p - lam));
u_curr = lam_dot - k_gain * (sigma_p0 - sigma_pc);

% 3) t = 0.01s 업데이트 (Euler Integration)
r_new = r + r_dot * dt;
lam_new = lam + lam_dot * dt;
psi_p_new = psi_p + u_curr * dt;

% 4) 가속도 재계산
sigma_p_new = psi_p_new - lam_new;
lam_dot_new = (V ./ r_new) .* (-sin(lam_new - psi_t) - sin(psi_p_new - lam_new));
acc_cmd = V * (lam_dot_new - k_gain * (sigma_p_new - sigma_pc));

% 5) 시각화 (Figure 1)
figure(1); hold on; grid on; axis equal;
title(['Validation at r = ', num2str(r0_check), ' m']);

% Target 화살표
quiver(0, 0, 0, 2, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(0.5, 2, 'Target (+y)', 'FontSize', 12);

% 안전/위험 색상 구분 Plot
unsafe_idx = abs(acc_cmd) > acc_limit;
safe_idx = ~unsafe_idx;

% 점 찍기
scatter(Px(safe_idx), Py(safe_idx), 30, 'b', 'filled', 'DisplayName', 'Safe (<=2g)');
scatter(Px(unsafe_idx), Py(unsafe_idx), 30, 'r', 'filled', 'DisplayName', 'Unsafe (>2g)');

legend('Location', 'SouthWest');
xlabel('x (m)'); ylabel('y (m)');
xlim([-12, 2]); ylim([-2, 12]);


%% [Part 2] Region Map 그리기 (이게 선생이 궁금해한 확장판)
% r 범위를 1m ~ 20m로 확장해서 격자(Meshgrid) 생성
r_range = linspace(1, 10, 100);
theta_range = linspace(theta_start, theta_end, 100);
[R_grid, Theta_grid] = meshgrid(r_range, theta_range);

% 1) 전체 격자에 대해 상태 계산
Px_grid = R_grid .* cos(Theta_grid);
Py_grid = R_grid .* sin(Theta_grid);
Lam_grid = Theta_grid + pi;
Psi_p_grid = sigma_p0 + Lam_grid;

% 2) Kinematics (t=0)
% r_dot은 가속도 계산에 직접 안 쓰이니 생략 가능하지만 로직상 포함
% Lam_dot 계산 (행렬 연산 .* ./ 사용 주의)
Lam_dot_grid = (V ./ R_grid) .* (-sin(Lam_grid - psi_t) - sin(Psi_p_grid - Lam_grid));
U_curr_grid = Lam_dot_grid - k_gain * (sigma_p0 - sigma_pc);

% 3) t = 0.01s 업데이트
R_new_grid = R_grid + (V * (cos(Lam_grid - psi_t) - cos(Psi_p_grid - Lam_grid))) * dt;
Lam_new_grid = Lam_grid + Lam_dot_grid * dt;
Psi_p_new_grid = Psi_p_grid + U_curr_grid * dt;

% 4) 가속도 재계산 (전체 영역)
Sigma_p_new_grid = Psi_p_new_grid - Lam_new_grid;
Lam_dot_new_grid = (V ./ R_new_grid) .* (-sin(Lam_new_grid - psi_t) - sin(Psi_p_new_grid - Lam_new_grid));
Acc_cmd_grid = V * (Lam_dot_new_grid - k_gain * (Sigma_p_new_grid - sigma_pc));

% 5) Region 시각화 (Figure 2)
figure(2); hold on; grid on; axis equal;
title('Acceleration Saturation Region Map');

% 등고선 그리기 (Contourf)
% 레벨: 0부터 2g까지는 파란색 계열, 2g 이상은 빨간색으로 경계 표시
levels = [0, acc_limit, 100]; % 0~2g(안전), 2g~100(위험)
% contourf로 영역 채우기
[C, h] = contourf(Px_grid, Py_grid, abs(Acc_cmd_grid), [acc_limit acc_limit], 'LineColor', 'none');

% 색상 매핑 (커스텀: 안전=하늘색, 위험=빨간색)
colormap([0.8 0.8 1; 1 0.6 0.6]); 
caxis([0, 2*acc_limit]); % 색상 축 고정

% 경계선 강조
contour(Px_grid, Py_grid, abs(Acc_cmd_grid), [acc_limit acc_limit], 'r', 'LineWidth', 2);

% Target
quiver(0, 0, 0, 5, 'k', 'LineWidth', 2);
text(1, 5, 'Target', 'FontSize', 12);

xlabel('x (m)'); ylabel('y (m)');
legend('Saturation Boundary (2g)', 'Location', 'SouthWest');
txt = text(-15, 10, 'Red Area: Unsafe (>2g)\nBlue Area: Safe');
set(txt, 'FontSize', 11, 'FontWeight', 'bold');

xlim([-20, 2]); ylim([-2, 20]);