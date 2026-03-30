    clear; clc; close all;

%% 1. Inputs & Parameters
r_f = 2;              % [m]
a_max_input = 1;      % [g]
sigma_p_deg_array = [60 62.5 65];  % [degree] (선생이 마음대로 바꿀 수 있는 배열!)
epsilon = 1e-7;         % f(sigma_p;simga_t) 발산 예방
V = 20;               % [m/s] 
n = 50000;            % 점 개수
ub = pi;              % 어디까지 돌릴거예요?
ub_deg = rad2deg(ub);

% 단위 변환 및 계산
a_max = a_max_input * 9.81;       % [m/s^2]
sigma_t = linspace(0, ub, n);     % sigma_t 범위를 0부터 pi까지 생성

%% 2. Function Calculation & 3. Plots
% 그림 창은 하나만 띄우고 그 안에 3개의 서브플롯을 넣을 거야~
figure('Name', 'Guidance Reachable Set Analysis', 'Theme', 'light');

% sigma_p_deg 배열의 개수만큼 반복!
for k = 1:length(sigma_p_deg_array)
    sigma_p_deg = sigma_p_deg_array(k);
    sigma_p = deg2rad(sigma_p_deg);   % [rad]
    
    % 우변 상수 C 계산
    C = (2 * r_f * a_max) / (V^2);
    
    % 좌변 함수 계산 
    y = (sin(sigma_t) - sin(sigma_p)) .* (1 + cos(sigma_t + sigma_p)) / (cos(sigma_p)^2 + epsilon);
    
    % 3x1 서브플롯 중 k번째 위치에 그리기
    subplot(3, 1, k);
    hold on; grid on;
    
    % 좌변 그래프 (곡선: 산 모양)
    plot(rad2deg(sigma_t), y, 'b-', 'LineWidth', 2, 'DisplayName', 'LHS: f(\sigma_t)');
    
    % 우변 그래프 (상수 C: 수평선)
    yline(C, 'r--', 'LineWidth', 2, 'DisplayName', sprintf('RHS: C = %.4f', C));
    
    % -----------------------------------------------------------
    % [추가된 부분] 최댓값 찾기 및 r_f 역산
    % -----------------------------------------------------------
    [y_max, max_idx] = max(y);  %% asdasdasd
    sigma_t_max = sigma_t(max_idx);
    
    % y_max = C 로 두고 r_f 역산: r_f = (y_max * V^2) / (2 * a_max * cos(sigma_p)^2)
    r_f_max = (y_max * V^2) / (2 * a_max);
    
    % 최댓값 지점에 빨간색 별 모양 마커 표시
    plot(rad2deg(sigma_t_max), y_max, 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'HandleVisibility', 'off');
    
    % 최댓값 텍스트 표시 (가독성을 위해 위로 살짝 띄움)
    txt_max = sprintf(' Max: %.1f\\circ\n r_{f,max} = %.2f m', rad2deg(sigma_t_max), r_f_max);
    text(rad2deg(sigma_t_max), y_max + 0.1, txt_max, 'FontSize', 10, 'Color', 'r', 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    % -----------------------------------------------------------
    
    % 교점 찾기 로직
    diff_y = y - C;
    cross_idx = find(diff_y(1:end-1) .* diff_y(2:end) <= 0);
    
    for i = 1:length(cross_idx)
        idx = cross_idx(i);
        x_cross = rad2deg(sigma_t(idx)); % 교점의 x좌표 (도 단위)
        
        % 해당 교차각에서의 실제 도달 거리 r 계산!
        r_val = (V^2 / a_max) * (sin(sigma_t(idx)) - sin(sigma_p));
        
        % 교점에 노란색 마커 표시
        plot(x_cross, C, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
        
        % 텍스트에 각도와 거리를 같이 띄워주기
        txt = sprintf(' %.1f\\circ\n r = %.2f m', x_cross, r_val);
        text(x_cross, C + 0.05, txt, 'FontSize', 10, 'FontWeight', 'bold', 'VerticalAlignment', 'bottom');
    end
    
    % 그래프 꾸미기
    xlabel('\sigma_t [degree]', 'FontSize', 11, 'FontWeight', 'bold');
    ylabel('Value', 'FontSize', 11, 'FontWeight', 'bold');
    title(sprintf('\\sigma_p = %.1f\\circ 일 때의 교차각 및 거리 분석', sigma_p_deg), 'FontSize', 12);
    xlim([0 ub_deg]);
    
    % y축 범위를 텍스트가 안 잘리도록 여유 있게 조정 (최댓값 텍스트 공간 확보)
    ylim([min(-0.2, min(y)-0.1) max(max(y)+0.5, C+0.5)]); 
    
    legend('Location', 'best');
    hold off;
end