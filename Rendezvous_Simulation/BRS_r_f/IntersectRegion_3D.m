clear; clc; close all;

%% =========================================================
%  1. 기본 파라미터 
% =========================================================
V = 20.0;                  
g = 9.81;
acc_limit = 1 * g;         
epsilon = 1e-7;

% 논문용 깔끔한 플롯을 위한 표시 한계치 (너무 멀리 가는 값 잘라내기)
r_plot_limit = 1000.0; 

% Z축(각도) 해상도
sigma_pc_deg_list = 0 : 1 : 90; 
num_sigma = length(sigma_pc_deg_list);

% 곡면을 구성할 궤적(t) 해상도
num_t = 300; 

%% =========================================================
%  2. 3D Mesh 격자 사전 할당
% =========================================================
X_mesh = nan(num_sigma, num_t);
Y_mesh = nan(num_sigma, num_t);
Z_mesh = nan(num_sigma, num_t);

%% =========================================================
%  3. 표면(Surface) 경계선 계산
% =========================================================
for i_sigma = 1:num_sigma
    sig_pc_deg = sigma_pc_deg_list(i_sigma);
    sig_pc_rad = deg2rad(sig_pc_deg);

    if abs(cos(sig_pc_rad)) < 1e-10
        continue;
    end

    % r_f_min 계산
    t_calc = linspace(0, pi, 10000);
    y_calc = (sin(t_calc) - sin(sig_pc_rad)) ...
        .* (1 + cos(t_calc + sig_pc_rad)) ...
        ./ (cos(sig_pc_rad)^2 + epsilon);
    
    r_f_min = (max(y_calc) * V^2) / (2 * acc_limit);

    % 곡면의 궤적(t) 범위: -sig_pc 부터 pi - sig_pc 까지
    % 특이점을 피하기 위해 양끝에 1e-3 정도 여백을 줌
    sig_t = linspace(-sig_pc_rad + 1e-3, pi - sig_pc_rad - 1e-3, num_t);

    % 최소 반경(Boundary) 수식
    denom = cos((sig_t + sig_pc_rad) / 2).^2;
    r_min_traj = r_f_min * cos(sig_pc_rad)^2 ./ denom;

    % 값이 너무 커져서 그래프가 망가지는 것을 방지 (NaN 처리하면 깔끔하게 잘림)
    % r_min_traj(r_min_traj > r_plot_limit) = NaN;

    % X, Y 좌표 변환
    plot_angle = -pi/2 - sig_t;
    X_mesh(i_sigma, :) = r_min_traj .* cos(plot_angle);
    Y_mesh(i_sigma, :) = r_min_traj .* sin(plot_angle);
    
    % Z축은 해당 층의 sigma_pc 각도
    Z_mesh(i_sigma, :) = sig_pc_deg;
end

%% =========================================================
%  4. 3D 렌더링
% =========================================================
figure('Theme', 'light', 'Position', [150, 150, 800, 600]);
ax = axes;
hold(ax, 'on'); 
grid(ax, 'on');

% [핵심] surf 함수로 단일 곡면 렌더링
h_surf = surf(ax, X_mesh, Y_mesh, Z_mesh);

% 디자인 스타일링: 반투명하고 깔끔한 메쉬
set(h_surf, ...
    'FaceAlpha', 0.6, ...           % 투명도
    'EdgeColor', [0.6 0.6 0.6], ... % 격자선 색상(회색)
    'EdgeAlpha', 0.4, ...           % 격자선 투명도
    'FaceColor', 'interp');         % 색상을 부드럽게 보간

% 색상 테마: 파스텔톤 느낌을 주려면 parula가 무난해
colormap(ax, parula); 
clim(ax, [0, 90]);

% 축 및 시점 설정
view(ax, -40, 30);
xlim(ax, [-1000, 0]);
ylim(ax, [0, 1000]);
zlim(ax, [0, 90]);

% 축 라벨 
xlabel(ax, 'x (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel(ax, 'y (m)', 'FontSize', 12, 'FontWeight', 'bold');
zlabel(ax, '\sigma_{pc} (deg)', 'FontSize', 12, 'FontWeight', 'bold');

% 축 비율 고정 (x, y는 동일하게, z는 시각적 밸런스에 맞게)
daspect(ax, [10, 10, 1]);

% 테두리 박스 켜서 공간감 부여
ax.Box = 'on';
ax.LineWidth = 1.0;