clear; clc; close all;

%% =========================================================
%  1. 기본 파라미터 및 스케일 팩터
% =========================================================
scale = 100;                 % 스케일 팩터 (1이면 기본 스케일, 100이면 100배 확대)

V = 20.0;                  
g = 9.81;
acc_limit = 1 * g;         

% 거리/반경 파라미터에 스케일 팩터를 곱해서 전체적인 크기 동기화
r_f_max = 2.0 * scale;             
r_target_radius = 2.0 * scale;     
max_r_plot_limit = 20000 * scale;  % 화면 밖으로 충분히 뻗어나가도록 한계치 대폭 상향

epsilon = 1e-7;

sigma_pc_deg_list = 0 : 0.1 : 90;
num_sigma = length(sigma_pc_deg_list);

%% =========================================================
%  2. Figure 준비
% =========================================================
figure( ...
    'Name', 'Accumulated Intersect Region', ...
    'Theme', 'light', ...
    'Position', [100, 100, 1000, 750]);

ax = axes;
hold(ax, 'on');
grid(ax, 'off');           
axis(ax, 'equal');

fprintf('\n====================================================\n');
fprintf(' Intersect Region 덧칠 시작 (스케일 팩터: %d)\n', scale);
fprintf(' 잘림 현상 디버깅 완료\n');
fprintf('====================================================\n');

%% =========================================================
%  3. sigma_pc = 0도 ~ 90도 Sweep (어두운 초록색 덧칠)
% =========================================================
dark_green = [0, 0.5, 0];

for i_sigma = 1:num_sigma

    sigma_pc_deg = sigma_pc_deg_list(i_sigma);
    sigma_pc_rad = deg2rad(sigma_pc_deg);

    if abs(cos(sigma_pc_rad)) < 1e-10
        continue;
    end

    % r_f_min 계산 (이 값도 스케일에 맞춰 비례하도록 수정)
    sigma_t_calc = linspace(0, pi, 50000);
    
    y_calc = ...
        (sin(sigma_t_calc) - sin(sigma_pc_rad)) ...
        .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ...
        ./ (cos(sigma_pc_rad)^2 + epsilon);

    y_max_value = max(y_calc);
    r_f_min = ((y_max_value * V^2) / (2 * acc_limit)) * scale;

    if r_f_min > r_f_max || r_f_min < 0
        continue;
    end

    % FRS/BRS 경계 곡선 계산
    sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 2000);

    denominator = cos((sigma_t_traj + sigma_pc_rad) / 2).^2;
    r_min_traj = r_f_min * cos(sigma_pc_rad)^2 ./ denominator;
    r_max_traj = r_f_max * cos(sigma_pc_rad)^2 ./ denominator;

    % [디버깅 핵심] 발산하는 영역의 인덱스를 날리지 않고 최대치로 고정(cap)
    r_max_traj = min(r_max_traj, max_r_plot_limit);

    % r_max_traj 한계 초과 필터링 제거
    valid_idx = ...
        isfinite(r_min_traj) ...
        & isfinite(r_max_traj) ...
        & (r_min_traj >= r_target_radius) ...
        & (r_max_traj >= r_min_traj);

    valid_number = find(valid_idx);

    if isempty(valid_number)
        continue;
    end

    separate_point = [1, find(diff(valid_number) > 1) + 1, length(valid_number) + 1];

    for i_segment = 1:length(separate_point) - 1

        segment_idx = valid_number(separate_point(i_segment) : separate_point(i_segment + 1) - 1);

        if length(segment_idx) < 2
            continue;
        end

        segment_sigma_t = sigma_t_traj(segment_idx);
        segment_r_min = r_min_traj(segment_idx);
        segment_r_max = r_max_traj(segment_idx);

        plot_angle = -pi/2 - segment_sigma_t;

        x_min = segment_r_min .* cos(plot_angle);
        y_min = segment_r_min .* sin(plot_angle);
        x_max = segment_r_max .* cos(plot_angle);
        y_max = segment_r_max .* sin(plot_angle);

        x_polygon = [x_min, fliplr(x_max)];
        y_polygon = [y_min, fliplr(y_max)];

        finite_polygon = isfinite(x_polygon) & isfinite(y_polygon);
        x_polygon = x_polygon(finite_polygon);
        y_polygon = y_polygon(finite_polygon);

        if length(x_polygon) < 3
            continue;
        end

        fill(ax, x_polygon, y_polygon, dark_green, 'EdgeColor', 'none');
    end
end

%% =========================================================
%  4. Target 및 후처리
% =========================================================
target_x = [0, -1, 0, 1] * scale;
target_y = [1, -1, -0.3, -1] * scale;

fill( ...
    ax, target_x, target_y, 'r', ...
    'EdgeColor', 'k', 'LineWidth', 1.5);

theta_circle = linspace(0, 2*pi, 300);
plot( ...
    ax, ...
    r_target_radius * cos(theta_circle), ...
    r_target_radius * sin(theta_circle), ...
    'k--', 'LineWidth', 1.2);

xlim(ax, [-70, 10] * scale);
ylim(ax, [-40, 40] * scale);

xlabel(ax, 'x (m)', 'FontSize', 12);
ylabel(ax, 'y (m)', 'FontSize', 12);
title(ax, ...
    sprintf('Intersect Region (Scale = %d)\n\\sigma_{pc} = 0^\\circ:0.5^\\circ:90^\\circ', scale), ...
    'FontSize', 15, 'FontWeight', 'bold');

fprintf(' 덧칠 완료~\n');
fprintf('====================================================\n');