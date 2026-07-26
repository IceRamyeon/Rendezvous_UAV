% DPG (Deviated Pursuit Guidance) Trajectory in Target Reference Frame
clc; clear; close all;

% --- 파라미터 설정 ---
Vp = 20;           % Pursuer의 속도
Vt = 20;           % Target의 속도
gamma_t_deg = 90;       % Target의 이동 방향 (라디안, x축 기준)
gamma_t = deg2rad(gamma_t_deg);
T = 3;             % 시뮬레이션 할 일정 시간 (초)

% Pursuer의 초기 위치 (Target 기준 프레임)
x0 = -300;
y0 = 400;

% 시그마_pc (Look angle / Deviation angle) 범위: 0도 ~ 90도
sigma_pc_deg = -170:10:180; 
final_points = zeros(length(sigma_pc_deg), 2);

figure('Theme','light'); hold on; grid on;

% --- 각 sigma_pc에 대해 궤적 계산 ---
for i = 1:length(sigma_pc_deg)
    sigma_pc = deg2rad(sigma_pc_deg(i));
    
    % 상대 운동 방정식 (ODE)
    % Y(1) = x, Y(2) = y
    % Target이 원점(0,0)이므로 Pursuer에서 Target을 바라보는 LOS 각도 lambda는 atan2(-y, -x)
    dydt = @(t, Y) [
        Vp * cos(atan2(-Y(2), -Y(1)) + sigma_pc) - Vt * cos(gamma_t);
        Vp * sin(atan2(-Y(2), -Y(1)) + sigma_pc) - Vt * sin(gamma_t)
    ];
    
    % ode45로 미분방정식 풀이
    [t, Y] = ode45(dydt, [0 T], [x0; y0]);
    
    % 궤적 그리기
    plot(Y(:,1), Y(:,2), 'DisplayName', ['\sigma_{pc} = ' num2str(sigma_pc_deg(i)) '^\circ']);
    
    % T초 후의 final point 저장
    final_points(i, :) = Y(end, :);
end

% --- Final points 및 마커 표시 ---
% Final points들을 이어주는 선
plot(final_points(:,1), final_points(:,2), 'r-o', 'LineWidth', 0.5, 'MarkerFaceColor', 'r', 'DisplayName', 'Final Points Set');

% 타겟 위치 (원점) 표시
plot(0, 0, 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'DisplayName', 'Target (0,0)');

xlabel('X Position'); 
ylabel('Y Position');
axis equal;

r = sqrt(x0^2 + y0^2);
xlim([-r 10]);
ylim([-0.1*r 0.9*r+10]);