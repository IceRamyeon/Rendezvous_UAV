%%  IsPointRegion.m
% 입력한 점이 Reachable Region 안에 있는지 판별하는 함수

clear; clc; close all;

% =========================================================
% 0. Input Point (Bearing Angle)
% Target heading (+y축) 기준: CW (+), CCW (-)
% =========================================================
r_pt = 400;
theta_pt_deg = -80;

% =========================================================
%  1. 기본 파라미터 및 스케일 팩터
% =========================================================

V = 20.0;                  
g = 9.81;
acc_limit = 1 * g;         

% 거리/반경 파라미터에 스케일 팩터를 곱해서 전체적인 크기 동기화
r_f_max = 2.0;             
r_target_radius = 2.0;     
max_r_plot_limit = 20000;  % 화면 밖으로 충분히 뻗어나가도록 한계치 대폭 상향

epsilon = 1e-7;

sigma_pc_deg_list = 0 : 0.05 : 90;
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
fprintf(' Intersect Region 덧칠 시작\n');
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

    % r_f_min 계산 
    sigma_t_calc = linspace(0, pi, 50000);
    
    y_calc = ...
        (sin(sigma_t_calc) - sin(sigma_pc_rad)) ...
        .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ...
        ./ (cos(sigma_pc_rad)^2 + epsilon);

    y_max_value = max(y_calc);
    r_f_min = ((y_max_value * V^2) / (2 * acc_limit));

    if r_f_min > r_f_max || r_f_min < 0
        continue;
    end

    % FRS/BRS 경계 곡선 계산
    sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 2000);

    denominator = cos((sigma_t_traj + sigma_pc_rad) / 2).^2;
    r_min_traj = r_f_min * cos(sigma_pc_rad)^2 ./ denominator;
    r_max_traj = r_f_max * cos(sigma_pc_rad)^2 ./ denominator;

    r_max_traj = min(r_max_traj, max_r_plot_limit);

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

% =========================================================
%  4. Target 및 후처리
% =========================================================
target_x = [0, -1, 0, 1];
target_y = [1, -1, -0.3, -1];

fill( ...
    ax, target_x, target_y, 'r', ...
    'EdgeColor', 'k', 'LineWidth', 1.5);

theta_circle = linspace(0, 2*pi, 300);
plot( ...
    ax, ...
    r_target_radius * cos(theta_circle), ...
    r_target_radius * sin(theta_circle), ...
    'k--', 'LineWidth', 1.2);

% =========================================================
%  5. Input Point 영역 포함 여부 검사 및 표시 (Bearing Angle 기준)
% =========================================================
theta_pt_rad = deg2rad(theta_pt_deg);

% Target heading(+y축)을 기준으로 CW(+), CCW(-)
x_pt = r_pt * sin(theta_pt_rad);
y_pt = r_pt * cos(theta_pt_rad);

% 플롯 각도 정의(plot_angle = -pi/2 - sigma_t)에 따른 역산
% cos(plot_angle) = x/r, sin(plot_angle) = y/r
% sigma_t_pt = -pi/2 - plot_angle 
% => cos(sigma_t_pt) = -y/r, sin(sigma_t_pt) = -x/r
% => sigma_t_pt = atan2(-x_pt, -y_pt)
sigma_t_pt = atan2(-x_pt, -y_pt);
is_inside = false;

for i_sigma = 1:num_sigma
    sigma_pc_deg = sigma_pc_deg_list(i_sigma);
    sigma_pc_rad = deg2rad(sigma_pc_deg);

    if abs(cos(sigma_pc_rad)) < 1e-10
        continue;
    end

    % r_f_min 동일하게 계산
    sigma_t_calc = linspace(0, pi, 50000);
    y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ./ (cos(sigma_pc_rad)^2 + epsilon);
    r_f_min = ((max(y_calc) * V^2) / (2 * acc_limit));

    if r_f_min > r_f_max || r_f_min < 0
        continue;
    end

    % 해당 sigma_pc에서 점의 각도가 유효한지 확인
    if sigma_t_pt >= -sigma_pc_rad && sigma_t_pt <= (pi - sigma_pc_rad)
        
        % 해당 각도에서의 r_min과 r_max 경계 계산
        denominator = cos((sigma_t_pt + sigma_pc_rad) / 2)^2;
        r_min_val = r_f_min * cos(sigma_pc_rad)^2 / denominator;
        r_max_val = r_f_max * cos(sigma_pc_rad)^2 / denominator;
        r_max_val = min(r_max_val, max_r_plot_limit);

        % 점의 거리(r)가 해당 각도에서의 경계 안쪽에 있는지 판별
        if r_pt >= r_min_val && r_pt <= r_max_val && r_min_val >= r_target_radius
            is_inside = true;
            break; 
        end
    end
end

% 결과 출력
fprintf(' 덧칠 완료\n');
fprintf('====================================================\n');
if is_inside
    fprintf(' [결과] 입력한 점 (r=%.1f, theta=%.1f도)은 영역 안에 있음.\n', r_pt, theta_pt_deg);
else
    fprintf(' [결과] 입력한 점 (r=%.1f, theta=%.1f도)은 영역 밖에 있음.\n', r_pt, theta_pt_deg);
end
fprintf('====================================================\n');

% ---------------------------------------------------------
% 플롯에 입력한 점 그리기 및 축 한계 조절
% ---------------------------------------------------------
plot(ax, x_pt, y_pt, 'bp', 'MarkerFaceColor', 'b', 'MarkerSize', 12);
text(ax, x_pt, y_pt, sprintf('  Input Point'), 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');

% 점이 잘리지 않도록 기존 축 범위와 점의 위치를 비교해서 유동적으로 축 설정
x_lb = min([100 * -1, x_pt - 50]);
x_up = max([10,  x_pt + 50]);
y_lb = min([50 * -1, y_pt - 50]);
y_up = max([50,  y_pt + 50]);

xlim(ax, [x_lb, x_up]);
ylim(ax, [y_lb, y_up]);

xlabel(ax, 'x (m)', 'FontSize', 12);
ylabel(ax, 'y (m)', 'FontSize', 12);
title(ax, ...
    sprintf('Intersect Region ($\\sigma_{pc} \\in [0^\\circ, 90^\\circ]$)'), ...
    'FontSize', 15, ...
    'FontWeight', 'bold', ...
    'Interpreter', 'latex');