clear; clc; close all;

%% =========================================================
%  1. 기본 파라미터 및 스케일 팩터
% =========================================================
scale = 1;                 

V = 20.0;                  
g = 9.81;
acc_limit = 1 * g;         

r_f_max = 2.0;             
r_target_radius = 2.0 * scale;     
max_r_plot_limit = 500;    

epsilon = 1e-7;

sigma_pc_deg_list = 0:1:90; 
num_sigma = length(sigma_pc_deg_list);

num_s = 300; % 곡면을 구성할 궤적(t) 해상도

% 격자 사전 할당
X_surf = NaN(num_sigma, num_s);
Y_surf = NaN(num_sigma, num_s);
Z_surf = NaN(num_sigma, num_s);

%% =========================================================
%  2. 데이터 계산 (NaN 클리핑 제거 & 유효 구간 동적 할당)
% =========================================================
for i_sigma = 1:num_sigma

    sigma_pc_deg = sigma_pc_deg_list(i_sigma);
    sigma_pc_rad = deg2rad(sigma_pc_deg);

    if abs(cos(sigma_pc_rad)) < 1e-10
        continue;
    end

    % r_f_min 계산
    sigma_t_calc = linspace(0, pi, 50000);
    y_calc = (sin(sigma_t_calc) - sin(sigma_pc_rad)) ...
        .* (1 + cos(sigma_t_calc + sigma_pc_rad)) ...
        ./ (cos(sigma_pc_rad)^2 + epsilon);

    y_max_value = max(y_calc);
    r_f_min = ((y_max_value * V^2) / (2 * acc_limit));

    if r_f_min > r_f_max || r_f_min < 0
        continue;
    end

    % [1단계] 유효한 sigma_t 범위를 찾기 위한 테스트
    t_test = linspace(-sigma_pc_rad, pi - sigma_pc_rad - 1e-5, 2000);
    denom_test = cos((t_test + sigma_pc_rad) / 2).^2;
    r_min_test = r_f_min * cos(sigma_pc_rad)^2 ./ denom_test;
    r_max_test = r_f_max * cos(sigma_pc_rad)^2 ./ denom_test;
    
    r_max_test = min(r_max_test, max_r_plot_limit);
    
    valid_idx = isfinite(r_min_test) & isfinite(r_max_test) ...
              & (r_min_test >= r_target_radius) & (r_max_test >= r_min_test);
              
    if ~any(valid_idx)
        continue; % 유효 영역이 전혀 없으면 이 층은 패스 (NaN으로 남음)
    end
    
    % [핵심] 억지로 잘라내는 대신, 딱 맞는 양 끝점(Boundary)을 추출
    t_start = t_test(find(valid_idx, 1, 'first'));
    t_end   = t_test(find(valid_idx, 1, 'last'));
    
    % [2단계] 추출한 [t_start, t_end] 구간만 num_s 개로 균등하게 쪼개기
    sig_t_actual = linspace(t_start, t_end, num_s);
    
    denom_actual = cos((sig_t_actual + sigma_pc_rad) / 2).^2;
    r_min_actual = r_f_min * cos(sigma_pc_rad)^2 ./ denom_actual;
    r_max_actual = r_f_max * cos(sigma_pc_rad)^2 ./ denom_actual;
    r_max_actual = min(r_max_actual, max_r_plot_limit);

    for i_s = 1:num_s
        theta = -pi/2 - sig_t_actual(i_s);
        
        % y값을 최대로 만들기 위한 반경 선택 로직 (이전과 동일)
        if sin(theta) >= 0
            r_selected = r_max_actual(i_s);
        else
            r_selected = r_min_actual(i_s);
        end
        
        X_surf(i_sigma, i_s) = r_selected * cos(theta);
        Y_surf(i_sigma, i_s) = r_selected * sin(theta);
        Z_surf(i_sigma, i_s) = sigma_pc_deg;
    end
end

%% =========================================================
%  3. 3D Figure 렌더링
% =========================================================
figure('Theme', 'light', 'Position', [150, 150, 800, 600]);
ax = axes;
hold(ax, 'on');
grid(ax, 'on');           

% 이제 NaN으로 깨진 부분 없이 꽉 찬 행렬이 들어가서 면이 부드러워져
h_surf = surf(ax, X_surf, Y_surf, Z_surf);

set(h_surf, ...
    'EdgeColor', 'none', ...
    'FaceColor', 'interp', ...
    'FaceAlpha', 0.85);

colormap(ax, 'parula');
c = colorbar(ax);
c.Label.String = '\sigma_{pc} (deg)';
c.Label.FontSize = 11;

% 시점 및 축 설정
view(ax, -40, 30);

x_lb = 100 * scale * -1;
x_up = 0 * scale * 1;
y_lb = 50 * scale * -1;
y_up = 50 * scale * 1;

xlim(ax, [x_lb, x_up]);
ylim(ax, [y_lb, y_up]);
zlim(ax, [0, 90]);

xlabel(ax, 'x (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax, 'y (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel(ax, '\sigma_{pc} (deg)', 'FontSize', 12, 'FontWeight', 'bold');
% title(ax, ...
%    sprintf('3D Intersect Region Max-Y Surface ($\\sigma_{pc} \\in [0^\\circ, 90^\\circ]$)'), ...
%    'FontSize', 15, ...
%    'FontWeight', 'bold', ..
%    'Interpreter', 'latex')