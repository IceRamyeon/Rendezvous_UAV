clear; clc; close all;

%% =========================================================
%  1. 기본 파라미터 및 스케일 팩터
% =========================================================
scale = 1;               

V = 20.0;                  
g = 9.81;
acc_limit = 1 * g;         

r_f_max = 2.0;             
r_target_radius = 2.0;     
max_r_plot_limit = 20000;  

epsilon = 1e-7;

sigma_pc_deg_list = 0 : 0.5 : 90;
num_sigma = length(sigma_pc_deg_list);

%% =========================================================
%  2. Figure 준비 (3차원 뷰어 및 조명 설정)
% =========================================================
figure( ...
    'Name', '3D Continuous Intersect Region', ...
    'Theme', 'light', ...
    'Position', [100, 100, 1000, 750]);

ax = axes;
hold(ax, 'on');
grid(ax, 'on');            
axis(ax, 'equal');

% 이전 시점(-40, 30)으로 복구
view(ax, -40, 30); 

% 이전 컬러맵(parula)으로 복구
colormap(ax, 'parula');     

fprintf('\n====================================================\n');
fprintf(' 3D Intersect Region 곡면 렌더링 시작 (스케일: %d)\n', scale);
fprintf('====================================================\n');

%% =========================================================
%  3. 보간용 3D Mesh 행렬 사전 할당
% =========================================================
M_side = 200; 
num_cols = M_side * 2 + 1; 

X_mesh = nan(num_sigma, num_cols);
Y_mesh = nan(num_sigma, num_cols);
Z_mesh = nan(num_sigma, num_cols);
C_mesh = nan(num_sigma, num_cols);

%% =========================================================
%  4. sigma_pc = 0도 ~ 90도 Sweep (경계선 추출 및 보간)
% =========================================================
for i_sigma = 1:num_sigma

    sigma_pc_deg = sigma_pc_deg_list(i_sigma);
    sigma_pc_rad = deg2rad(sigma_pc_deg);

    if abs(cos(sigma_pc_rad)) < 1e-10
        continue;
    end

    sigma_t_calc = linspace(0, pi, 50000);
    y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) ...
        .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ...
        ./ (cos(sigma_pc_rad)^2 + epsilon);

    y_max_value = max(y_calc);
    r_f_min = ((y_max_value * V^2) / (2 * acc_limit));

    if r_f_min > r_f_max || r_f_min < 0
        continue;
    end

    sigma_t_traj = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 2000);
    denominator = cos((sigma_t_traj + sigma_pc_rad) / 2).^2;
    r_min_traj = r_f_min * cos(sigma_pc_rad)^2 ./ denominator;
    r_max_traj = r_f_max * cos(sigma_pc_rad)^2 ./ denominator;
    r_max_traj = min(r_max_traj, max_r_plot_limit);

    valid_idx = isfinite(r_min_traj) & isfinite(r_max_traj) ...
        & (r_min_traj >= r_target_radius) & (r_max_traj >= r_min_traj);
    valid_number = find(valid_idx);

    if isempty(valid_number)
        continue;
    end

    separate_point = [1, find(diff(valid_number) > 1) + 1, length(valid_number) + 1];
    max_len = 0;
    best_segment_idx = [];
    for i_seg = 1:length(separate_point)-1
        seg_idx = valid_number(separate_point(i_seg) : separate_point(i_seg+1)-1);
        if length(seg_idx) > max_len
            max_len = length(seg_idx);
            best_segment_idx = seg_idx;
        end
    end

    if max_len < 2
        continue;
    end

    t_start = sigma_t_traj(best_segment_idx(1));
    t_end   = sigma_t_traj(best_segment_idx(end));
    t_resampled = linspace(t_start, t_end, M_side);

    denom_res = cos((t_resampled + sigma_pc_rad) / 2).^2;
    r_min_res = r_f_min * cos(sigma_pc_rad)^2 ./ denom_res;
    r_max_res = r_f_max * cos(sigma_pc_rad)^2 ./ denom_res;
    r_max_res = min(r_max_res, max_r_plot_limit);

    plot_angle_res = -pi/2 - t_resampled;

    x_min = r_min_res .* cos(plot_angle_res);
    y_min = r_min_res .* sin(plot_angle_res);
    x_max = r_max_res .* cos(plot_angle_res);
    y_max = r_max_res .* sin(plot_angle_res);

    X_row = [x_min, fliplr(x_max), x_min(1)];
    Y_row = [y_min, fliplr(y_max), y_min(1)];
    Z_row = (sigma_pc_deg) * ones(size(X_row));
    C_row = sigma_pc_deg * ones(size(X_row));

    X_mesh(i_sigma, :) = X_row;
    Y_mesh(i_sigma, :) = Y_row;
    Z_mesh(i_sigma, :) = Z_row;
    C_mesh(i_sigma, :) = C_row;
end

%% =========================================================
%  5. 3D 단일 곡면(Surface) 생성 및 조명 효과
% =========================================================
surf(ax, X_mesh, Y_mesh, Z_mesh, C_mesh, ...
    'EdgeColor', 'none', ...
    'FaceColor', 'interp', ...
    'FaceAlpha', 0.85, ... % 이전 투명도로 복구
    'AmbientStrength', 0.4, ...
    'DiffuseStrength', 0.8, ...
    'SpecularStrength', 0.1);

camlight(ax, 'headlight');
lighting(ax, 'gouraud');

%% =========================================================
%  6. Target 및 후처리
% =========================================================
% target_x = [0, -1, 0, 1] * scale;
% target_y = [1, -1, -0.3, -1] * scale;
% target_z = [0, 0, 0, 0]; 

% fill3( ...
%     ax, target_x, target_y, target_z, 'r', ...
%     'EdgeColor', 'k', 'LineWidth', 1.5);

% theta_circle = linspace(0, 2*pi, 300);
% plot3( ...
%     ax, ...
%     (r_target_radius * scale) * cos(theta_circle), ...
%     (r_target_radius * scale) * sin(theta_circle), ...
%     zeros(size(theta_circle)), ...
%     'k--', 'LineWidth', 1.2);

% 이전 x, y축 스케일 범위로 복구
xlim(ax, [-100 * scale, 0]);
ylim(ax, [-50 * scale, 50 * scale]);
zlim(ax, [0, 90]);

xlabel(ax, 'x (m)', 'FontSize', 12);
ylabel(ax, 'y (m)', 'FontSize', 12);

if scale == 1
    zlabel(ax, '\sigma_{pc} (deg)', 'FontSize', 12);
else
    zlabel(ax, sprintf('\\sigma_{pc} \\times %d', scale), 'FontSize', 12);
end

% title(ax, ...
%     sprintf('3D Continuous Intersect Region (Scale = %d)', scale), ...
%     'FontSize', 15, 'FontWeight', 'bold');

cb = colorbar(ax);
cb.Label.String = '\sigma_{p} (deg)';
cb.Label.FontSize = 12;

fprintf(' plot 완료\n');
fprintf('====================================================\n');