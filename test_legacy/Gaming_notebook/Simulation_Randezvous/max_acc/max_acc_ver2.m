clear; clc; close all;

%% 1. 파라미터 설정 (으헤~ 기본 설정이야)
V = 20.0;             % 속도 (m/s)
r_check = 10.0;       % 가속도 포화 여부를 판별할 기준 거리 (m)
sigma_raw = 30.0;     % 명령과 초기 Lead angle
sigma_p0_deg = sigma_raw;  % 초기 Lead Angle (deg)
sigma_pc_deg = sigma_raw;  % 명령 Lead Angle (deg)
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

%% [Step 1 & 2] r = 10m 에서 가속도 포화 각도 판별
% 으헤~ 여기서 먼저 '어떤 각도가 위험한지' 딱 정하는 거야.

theta_arc = linspace(theta_start, theta_end, 500); % 정밀도를 위해 점 개수 늘림

% 1) 초기 상태 계산 (r = r_check)
r = r_check;
lam = theta_arc + pi; 
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

% 5) 안전/위험 인덱스 구분
unsafe_idx = abs(acc_cmd) > acc_limit;
safe_idx = ~unsafe_idx;


%% [Step 3] 기하학적 확장 (Geometric Expansion) 반영
% 선생의 아이디어: "랑데부 시간 t_f가 지났을 때, Pursuer들의 집합은 원을 그린다!"
% 중심: (0, V*t_f), 반지름: V*t_f (Target이 V*t_f만큼 전진했으니까)

figure(1); clf; hold on; grid on; axis equal;
title(['Geometric Safe/Unsafe Region (Step 3 Logic)']);

% Target 화살표
quiver(0, 0, 0, 5, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
text(1, 5, 'Target Velocity', 'FontSize', 12, 'FontWeight', 'bold');

% 확장할 최대 시간 (초) -> 영역 크기 결정
tf_max = 5.0; 
tf_steps = linspace(0.1, tf_max, 50); % 0.1초부터 5초까지 확장

% 루프를 돌면서 원들을 그린다 (마치 파문이 퍼지듯이~)
for tf = tf_steps
    % 1. 기하학적 원의 중심과 반지름 (Step 3 내용)
    % Target은 (0, 0)에서 +y 방향으로 V*tf 만큼 이동
    Center_Y = V * tf; 
    Radius = V * tf;   % 랑데부하려면 Pursuer도 V*tf 만큼 이동해야 함
    
    % 2. 각도(theta_arc)에 따른 Pursuer 위치 매핑
    % 논리: r_check에서 확인한 '위험한 상대 각도(theta)'가 유지된다고 가정하고 확장
    % 원의 방정식: (x - 0)^2 + (y - Center_Y)^2 = Radius^2
    % 여기서 theta_arc는 Target에 대한 상대 각도이므로 좌표 변환 적용
    
    % Pursuer 위치 (기하학적)
    % P_x = Radius * cos(theta_arc)
    % P_y = Center_Y + Radius * sin(theta_arc)
    % (참고: theta는 2사분면이므로 cos은 음수, sin은 양수)
    
    Curve_X = Radius .* cos(theta_arc);
    Curve_Y = Center_Y + Radius .* sin(theta_arc);
    
    % 3. 색상 구분하여 그리기 (Plotting)
    % 안전한 구간 (Blue)
    if any(safe_idx)
        plot(Curve_X(safe_idx), Curve_Y(safe_idx), '-', 'Color', [0.6 0.8 1.0], 'LineWidth', 1.5); 
    end
    
    % 위험한 구간 (Red)
    if any(unsafe_idx)
        plot(Curve_X(unsafe_idx), Curve_Y(unsafe_idx), '-', 'Color', [1.0 0.6 0.6], 'LineWidth', 1.5);
    end
end

% 외곽선(마지막 tf) 진하게 그리기
Final_Center_Y = V * tf_max;
Final_Radius = V * tf_max;
Final_X = Final_Radius .* cos(theta_arc);
Final_Y = Final_Center_Y + Final_Radius .* sin(theta_arc);

plot(Final_X(safe_idx), Final_Y(safe_idx), 'b-', 'LineWidth', 2);
plot(Final_X(unsafe_idx), Final_Y(unsafe_idx), 'r-', 'LineWidth', 2);

% r_check (10m) 위치도 표시 (검증 지점)
% 기하학적 모델에서는 tf = r_check / V 일 때의 원에 해당함
tf_check = r_check / V;
Check_Center_Y = V * tf_check;
Check_Radius = V * tf_check;
Check_X = Check_Radius .* cos(theta_arc);
Check_Y = Check_Center_Y + Check_Radius .* sin(theta_arc);
plot(Check_X, Check_Y, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Validation Arc (r=10m)');


%% 그래프 꾸미기
xlabel('x (m)'); ylabel('y (m)');
xlim([-100, 10]); ylim([-10, 200]); % 범위는 적당히 조절

% 범례 생성 (더미 플롯 이용)
h_safe = plot(NaN, NaN, 's', 'MarkerFaceColor', [0.6 0.8 1.0], 'MarkerEdgeColor', 'b', 'MarkerSize', 10);
h_unsafe = plot(NaN, NaN, 's', 'MarkerFaceColor', [1.0 0.6 0.6], 'MarkerEdgeColor', 'r', 'MarkerSize', 10);
legend([h_safe, h_unsafe], {'Safe Region (Acc \le 2g)', 'Unsafe Region (Acc > 2g)'}, 'Location', 'NorthWest');

text(-80, 150, {'Geometric BRS Method', 'Expanding Circles based on', 'Immediate Acc Check'}, 'FontSize', 10, 'BackgroundColor', 'w');


%% [Part 2] Region Map 그리기 (이게 선생이 궁금해한 확장판)
% r 범위를 1m ~ 20m로 확장해서 격자(Meshgrid) 생성
r_range = linspace(1, 100, 10000);
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
% contour(Px_grid, Py_grid, abs(Acc_cmd_grid), [acc_limit acc_limit], 'r', 'LineWidth', 2);

% Target
quiver(0, 0, 0, 5, 'k', 'LineWidth', 2);
text(1, 5, 'Target', 'FontSize', 12);

xlabel('x (m)'); ylabel('y (m)');
legend('Saturation Boundary (2g)', 'Location', 'SouthWest');
txt = text(-15, 10, 'Red Area: Unsafe (>2g)\nBlue Area: Safe');
set(txt, 'FontSize', 11, 'FontWeight', 'bold');

xlim([-20, 2]); ylim([-2, 20]);