% DPG Final Points 집합과 특정 궤적 간의 최단 거리 찾기
clc; clear; close all;

%% 1. DPG Final Points (Test Point Set) 계산
Vp = 20;           
Vt = 20;           
gamma_t_deg = 90;       
gamma_t = deg2rad(gamma_t_deg);
T = 3;             

% Pursuer의 초기 위치 (Target 기준 프레임)
x0 = -800;
y0 = 500;

sigma_pc_deg_set = -170:10:180; 
final_points = zeros(length(sigma_pc_deg_set), 2);

for i = 1:length(sigma_pc_deg_set)
    sigma_pc_val = deg2rad(sigma_pc_deg_set(i));
    
    % 상대 운동 방정식
    dydt = @(t, Y) [
        Vp * cos(atan2(-Y(2), -Y(1)) + sigma_pc_val) - Vt * cos(gamma_t);
        Vp * sin(atan2(-Y(2), -Y(1)) + sigma_pc_val) - Vt * sin(gamma_t)
    ];
    
    [~, Y] = ode45(dydt, [0 T], [x0; y0]);
    final_points(i, :) = Y(end, :);
end

%% 2. Boundary Line (sigma_pc = 62.6도) 계산
r_f = 2.0;         
r_target_radius = 2.0;     
max_r_plot_limit = 20000;  

sigma_pc_deg_traj = 62.6;
sigma_pc_rad_traj = deg2rad(sigma_pc_deg_traj);

% 해석적 경계선 계산
sigma_t_traj = linspace(-sigma_pc_rad_traj, pi - sigma_pc_rad_traj - 1e-5, 20000);
denominator = cos((sigma_t_traj + sigma_pc_rad_traj) / 2).^2;
r_traj = r_f * cos(sigma_pc_rad_traj)^2 ./ denominator;
r_traj = min(r_traj, max_r_plot_limit);

valid_idx = isfinite(r_traj) & (r_traj >= r_target_radius);
plot_sigma_t = sigma_t_traj(valid_idx);
plot_r = r_traj(valid_idx);

x_traj = [];
y_traj = [];
if ~isempty(plot_r)
    plot_angle = -pi/2 - plot_sigma_t;
    x_traj = plot_r .* cos(plot_angle);
    y_traj = plot_r .* sin(plot_angle);
end

%% 3. 두 Set(Final Points vs 62.6도 경계선) 사이의 최단 거리 탐색 및 조건 판별
min_dist = inf;
best_fp_idx = 1;
best_traj_idx = 1;

if ~isempty(x_traj)
    % interp1을 사용하기 위해 x_traj의 중복 값을 제거하고 오름차순 정렬
    [x_unique, unique_idx] = unique(x_traj);
    y_unique = y_traj(unique_idx);
    
    % Final Point 중 하나라도 Boundary Line 아래에 있는지 판별
    is_any_below = false;
    for i = 1:size(final_points, 1)
        x_fp = final_points(i, 1);
        y_fp = final_points(i, 2);
        
        % 'extrap' 제거: 경계선 x 데이터 범위를 벗어나면 NaN 반환
        y_bound = interp1(x_unique, y_unique, x_fp, 'linear');
        
        % 계산된 y_bound가 존재하고(NaN이 아님), 점이 선보다 아래에 있을 때만 true
        if ~isnan(y_bound) && (y_fp < y_bound)
            is_any_below = true;
            break;
        end
    end
    
    if is_any_below
        % 조건 1: Boundary line 밑에 점이 있을 경우
        [~, best_fp_idx] = min(abs(sigma_pc_deg_set - sigma_pc_deg_traj));
        distances = sqrt((x_traj - final_points(best_fp_idx, 1)).^2 + (y_traj - final_points(best_fp_idx, 2)).^2);
        [min_dist, best_traj_idx] = min(distances);
    else
        % 조건 2: 모든 점이 Boundary line 위에 있을 경우 (지금 사진의 상황)
        for i = 1:size(final_points, 1)
            distances = sqrt((x_traj - final_points(i, 1)).^2 + (y_traj - final_points(i, 2)).^2);
            [local_min_dist, local_min_idx] = min(distances);
            
            if local_min_dist < min_dist
                min_dist = local_min_dist;
                best_fp_idx = i;
                best_traj_idx = local_min_idx;
            end
        end
    end
end

%% 4. 결과 시각화
figure('Name', 'Shortest Point to Final Points', 'Theme', 'light', 'Position', [100, 100, 1000, 750]);
hold on; grid on; axis equal;

% 1) 궤적 그리기 (Dark Green)
dark_green = [0.0, 0.6, 0.0];
if ~isempty(x_traj)
    plot(x_traj, y_traj, 'Color', dark_green, 'LineWidth', 2, 'DisplayName', sprintf('\\sigma_{pc} = %.1f^\\circ Trajectory', sigma_pc_deg_traj));
end

% 2) Final Points 집합 그리기 (Red line & markers)
plot(final_points(:,1), final_points(:,2), 'r-o', 'LineWidth', 1.5, 'MarkerFaceColor', 'r', 'DisplayName', 'Final Points Set');

% 3) 가장 가까운 점들 마킹 및 선 연결
if ~isempty(x_traj)
    closest_fp = final_points(best_fp_idx, :);
    closest_traj_pt = [x_traj(best_traj_idx), y_traj(best_traj_idx)];
    
    % 궤적 상의 최단 거리 점 (Purple Star)
    purple = [0.5, 0.0, 0.5];
    plot(closest_traj_pt(1), closest_traj_pt(2), 'p', 'MarkerSize', 15, 'MarkerFaceColor', purple, 'MarkerEdgeColor', 'k', 'DisplayName', 'Closest Traj Point');

    % Final Point 중 최단 거리인 점 (Blue Circle)
    plot(closest_fp(1), closest_fp(2), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'Closest Final Point');
    
    % 두 점을 잇는 점선
    plot([closest_fp(1), closest_traj_pt(1)], [closest_fp(2), closest_traj_pt(2)], 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    fprintf('--- 계산 결과 ---\n');
    fprintf('최단 거리: %.3f m\n', min_dist);
    fprintf('해당 Final Point의 sigma_pc: %d도\n', sigma_pc_deg_set(best_fp_idx));
end

% 4) Target 표시
target_x = [0, -1, 0, 1];
target_y = [1, -1, -0.3, -1];
fill(target_x, target_y, 'k', 'EdgeColor', 'k', 'LineWidth', 1.5, 'DisplayName', 'Target');
theta_circle = linspace(0, 2*pi, 300);
plot(r_target_radius * cos(theta_circle), r_target_radius * sin(theta_circle), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');

xlabel('x (m)', 'FontSize', 12);
ylabel('y (m)', 'FontSize', 12);
title(sprintf('Minimum Distance between Final Points and Trajectory (\\sigma_{pc} = %.1f^\\circ)', sigma_pc_deg_traj), 'FontSize', 15, 'FontWeight', 'bold');
legend('show', 'Location', 'bestoutside');

xlim([-1000 0]);
ylim([-100 900])