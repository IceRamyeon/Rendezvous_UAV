clear; clc; close all;

%% 1. 기본 파라미터 설정 (으헤~ 변하지 않는 것들이야)
V = 20.0;             % 속도 (m/s)
r_check = 10.0;       % 가속도 체크 거리 (m)
sigma_p0_ref = 60;    % Threshold 계산용 기준 초기 리드각 (deg)
k_gain = 3.0;         % 유도 이득
dt = 0.01;            % Time step (s)
g = 9.81;             % 중력 가속도
acc_limit = 2 * g;    % 가속도 제한 (2g)
psi_t = pi / 2;       % Target Heading (+y 방향)
scan_th_max = 180 - sigma_p0_ref;

%% 2. [Step 1] r=10m에서 가속도 포화가 일어나는 한계 각도(theta_th) 찾기
% "어느 각도부터 위험해지는지" 기준을 잡는 단계야.

% 검색 범위: 90도(측면) ~ 180도(정면)
theta_scan_deg = linspace(90, scan_th_max, 1000); 
theta_scan = deg2rad(theta_scan_deg);

% 상태 변수 설정
r = r_check;
lam = theta_scan + pi;           % LOS angle
sigma_ref_rad = deg2rad(sigma_p0_ref);
psi_p = lam + sigma_ref_rad;     % Pursuer Heading

% Kinematics (t=0)
r_dot = V * (cos(lam - psi_t) - cos(psi_p - lam));
lam_dot = (V / r) * (-sin(lam - psi_t) - sin(psi_p - lam));

% 0.01초 뒤 미래 예측 (Euler Step) - 기존 코드 로직 유지
u_curr = lam_dot; % t=0에서 제어 입력 (단순화)
r_new = r + r_dot * dt;
lam_new = lam + lam_dot * dt;
psi_p_new = psi_p + u_curr * dt;

% 가속도 명령 계산
sigma_p_new = psi_p_new - lam_new;
lam_dot_new = (V ./ r_new) .* (-sin(lam_new - psi_t) - sin(psi_p_new - lam_new));
acc_cmd = V * (lam_dot_new - k_gain * (sigma_p_new - sigma_ref_rad));

% 한계 각도 판별 (Safe -> Unsafe 넘어가는 지점)
unsafe_idx = find(abs(acc_cmd) > acc_limit, 1);

if isempty(unsafe_idx)
    theta_th = pi; % 전부 안전함 (이럴 리 없지만)
    theta_th_deg = 180;
else
    % 위험해지기 직전 각도를 한계값으로 설정
    theta_th = theta_scan(unsafe_idx); 
    theta_th_deg = theta_scan_deg(unsafe_idx);
end

fprintf('Calculated Threshold Bearing (theta_th): %.2f deg\n', theta_th_deg);


%% 3. [Step 2] 기하학적 영역 맵 생성 (Region Map)
% 선생이 말한 수식: a = V*tf*sin(2sigma), b = 2*V*tf*cos^2(sigma)
% 조건: theta_th < 2 * sigma_p0 이면 성공(Blue), 아니면 실패(Red)

% 격자 생성
tf_range = linspace(0.1, 30.0, 200);   % 비행 시간
sigma_range_deg = linspace(1, 89, 200); % 초기 리드각
[TF, Sigma_Deg] = meshgrid(tf_range, sigma_range_deg);
Sigma_Rad = deg2rad(Sigma_Deg);

% 좌표 변환 (-a, b)
% a = V * tf * sin(2*sigma)
% b = 2 * V * tf * cos^2(sigma)
A_grid = V .* TF .* sin(2 .* Sigma_Rad);
B_grid = 2 * V .* TF .* (cos(Sigma_Rad).^2);

X_grid = -A_grid; % 왼쪽 영역 검토
Y_grid = B_grid;

% 성공/실패 마스크 생성 (기하학적 조건)
% 조건: theta_th < 2 * sigma_p0
Success_Mask = theta_th < (2 .* Sigma_Rad);


%% 4. 그래프 그리기 (수정됨)
% 으헤~ 여기가 문제였던 것 같아. 색칠하는 방법을 좀 더 확실하게 바꿨어.

figure('Name', 'Saturation Map Comparison', 'Theme', 'light', 'Position', [100, 100, 1200, 800]); clf; hold on; grid on; axis equal;
title(sprintf('Geometric Feasibility Map (Threshold \\theta_{th} = %.1f^\\circ)', theta_th_deg));

% [디버깅] 계산된 한계 각도 확인
fprintf('DEBUG: Calculated theta_th = %.2f deg\n', theta_th_deg);
if theta_th_deg >= 179.9
    fprintf('DEBUG: 으헤? 모든 각도에서 안전하다고 나왔어. 가속도 제한(2g)이 너무 널널한가 봐.\n');
end

% 1) 영역 표시 (Contourf 수정)
% 마스크(0 or 1)를 그릴 때, 레벨을 명확히 지정하거나 기본값 사용
% 0~1 사이를 채우도록 설정
[C, h] = contourf(X_grid, Y_grid, double(Success_Mask), linspace(0, 1, 3), 'LineColor', 'none');

% 2) 색상 매핑 강제 고정
% 0(Fail) = Red, 1(Success) = Blue
colormap([1 0.7 0.7;   % 0: 연한 빨강 (Fail)
          0.7 0.7 1]); % 1: 연한 파랑 (Success)
clim([0 1]); % (구버전 MATLAB은 caxis([0 1]) 사용)

% 3) 경계선 그리기 (Boundary Line)
% 2 * sigma = theta_th 인 지점 => sigma = theta_th / 2
boundary_sigma = theta_th / 2;
if rad2deg(boundary_sigma) < 90
    boundary_tf = linspace(0, 10, 100);
    boundary_a = V * boundary_tf * sin(2 * boundary_sigma);
    boundary_b = 2 * V * boundary_tf * (cos(boundary_sigma)^2);
    plot(-boundary_a, boundary_b, 'w-', 'LineWidth', 2, 'DisplayName', 'Boundary'); % 흰색 실선
end

% 4) 선생이 궁금해한 sigma_p0 = 30도 궤적 그리기
target_sigma_deg = 30;
target_sigma = deg2rad(target_sigma_deg);
traj_tf = linspace(0, 10, 100);

traj_a = V * traj_tf * sin(2 * target_sigma);
traj_b = 2 * V * traj_tf * (cos(target_sigma)^2);

plot(-traj_a, traj_b, 'k--', 'LineWidth', 2, 'DisplayName', ['Trajectory (\sigma_{p0}=' num2str(target_sigma_deg) '^\circ)']);

% 5) 타겟 및 꾸미기
quiver(0, 0, 0, 10, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');
text(2, 5, 'Target', 'FontSize', 12, 'FontWeight', 'bold');

xlabel('x (m) [-a]');
ylabel('y (m) [b]');
xlim([-1500, 10]); ylim([-10, 1500]);

% 범례 생성 (꼼수 안 쓰고 텍스트로 설명하거나, 위에서 DisplayName 쓴 것 활용)
legend('Location', 'NorthWest');

% 상태 텍스트
msg = sprintf('Condition: \\theta_{th} < 2\\sigma_{p0}\n(Current: %.1f < %.1f)', theta_th_deg, 2*target_sigma_deg);
text(-150, 100, msg, 'BackgroundColor', 'w', 'EdgeColor', 'k');