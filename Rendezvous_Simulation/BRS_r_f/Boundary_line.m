clear; clc;

%% =========================================================
%  1. 기본 파라미터 및 스케일 팩터
% =========================================================
scale = 1;                 
r_f = 2.0 * scale;         % r_f를 딱 2m로 고정
r_target_radius = 2.0 * scale;     
max_r_plot_limit = 20000;  

%% =========================================================
%  2. Figure 준비
% =========================================================
ax = gca;
hold(ax, 'on');
grid(ax, 'off');           
axis(ax, 'equal');

% 보고 싶은 단일 각도 설정 (선생이 말한 63도)
sigma_pc_deg = 62.6;
sigma_pc_rad = deg2rad(sigma_pc_deg);
dark_green = [0.0, 0.6, 0.0];

%% =========================================================
%  3. 단일 궤적 계산 및 선 긋기 (Plot)
% =========================================================
if abs(cos(sigma_pc_rad)) >= 1e-10
    
    % 궤적을 그릴 sigma_t 범위
    sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 2000);

    % r_f = 2.0 에 대한 단일 r 궤적 계산
    denominator = cos((sigma_t_traj + sigma_pc_rad) / 2).^2;
    r_traj = r_f * cos(sigma_pc_rad)^2 ./ denominator;

    % 화면 밖으로 너무 날아가는 값은 자르기
    r_traj = min(r_traj, max_r_plot_limit);

    % 유효한 값들만 추려내기 (타겟 반경 밖이면서 무한대가 아닌 값)
    valid_idx = isfinite(r_traj) & (r_traj >= r_target_radius);
    
    plot_sigma_t = sigma_t_traj(valid_idx);
    plot_r = r_traj(valid_idx);
    
    if ~isempty(plot_r)
        % 직교좌표계(Cartesian)로 변환
        plot_angle = -pi/2 - plot_sigma_t;
        x_traj = plot_r .* cos(plot_angle);
        y_traj = plot_r .* sin(plot_angle);
        
        % 면적을 채우는 fill 대신, 선을 그리는 plot 사용
        plot(ax, x_traj, y_traj, 'Color', dark_green, 'LineWidth', 3);
    end
end

%% =========================================================
%  4. Target 및 후처리
% =========================================================
target_x = [0, -1, 0, 1] * scale;
target_y = [1, -1, -0.3, -1] * scale;

fill(ax, target_x, target_y, 'r', 'EdgeColor', 'k', 'LineWidth', 1.5);

theta_circle = linspace(0, 2*pi, 300);
plot(ax, r_target_radius * cos(theta_circle), r_target_radius * sin(theta_circle), 'k--', 'LineWidth', 1.2);

x_lb = 100 * scale * -1;
x_up = 10 * scale * 1;
y_lb = 50 * scale * -1;
y_up = 50 * scale * 1;

xlim(ax, [x_lb, x_up]);
ylim(ax, [y_lb, y_up]);

xlabel(ax, 'x (m)', 'FontSize', 12);
ylabel(ax, 'y (m)', 'FontSize', 12);
title(ax, sprintf('Trajectory Line ($r_f = %.1f$m, $\\sigma_{pc} = %.1f^\\circ$)', r_f, sigma_pc_deg), ...
    'FontSize', 15, 'FontWeight', 'bold', 'Interpreter', 'latex');