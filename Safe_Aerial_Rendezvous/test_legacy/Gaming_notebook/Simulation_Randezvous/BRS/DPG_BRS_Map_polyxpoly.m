function DPG_BRS_Map_polyxpoly()
    % 으헤~ 거리와 방위각을 촘촘히 훑으면서 궤적이 가속도 영역을 관통하는지 검사하는 Map이야!
    % polyxpoly를 사용해 관통 거리를 계산하고, 2m 이하면 Semi-안전(초록색)으로 분류해.
    
    % close all;
    
    %% 1. 파라미터 세팅
    r_f = 2.0; % [m] 허용 오차 거리
    V_p = 20.0;
    V_t = 20.0;
    gain_k = 5.0;     
    limit_acc = 1.0;  
    a_max = limit_acc * 9.81; % G -> m/s^2
    epsilon = 2; % 허용거리
    
    % Sweep 조건
    r_vec = 100:20:1000;     % [m] 100m ~ 1000m (20m 간격)
    b_vec = 0:2:90;          % [deg] 방위각 0도 ~ 90도 (2도 간격)
    n_traj = 150;            % 궤적 점 개수
    
    %% 2. 피규어 준비
    figure('Name', 'Reachability Map (polyxpoly)', 'Theme', 'light', 'Position', [150, 150, 700, 600]);
    hold on; grid on; axis equal;
    
    % Target 위치 (중앙, +y 방향)
    Xt = 0; Yt = 0;
    quiver(Xt, Yt, 0, 100, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');
    plot(Xt, Yt, 'ko', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
    
    fprintf('>>> 분석 중...\n');
    
    %% 3. 점 생성 및 궤적-영역 교차 검사 루프
    for r = r_vec
        for b = b_vec
            % 방위각 설정
            sigma_t_init = deg2rad(b); 
            
            %% [Step 1] 비선형 방정식으로 sigma_ref 찾기
            eq = @(s_ref) cos(s_ref) - sqrt(r/r_f) * cos((s_ref + sigma_t_init)/2);
            opt = optimset('Display', 'off');
            sigma_ref = fzero(eq, pi - sigma_t_init, opt);
            
            %% [Step 2] 예상 Rendezvous Trajectory 생성
            sigma_path = linspace(sigma_t_init, sigma_ref, n_traj);
            rho_path = r_f * (cos(sigma_ref) ./ cos((sigma_ref + sigma_path)/2)).^2;
            lambda_path = pi/2 + sigma_path; 
            
            X_path = rho_path .* cos(lambda_path);
            Y_path = rho_path .* sin(lambda_path);
            
            % Pursuer 초기 시작 위치
            X_p = X_path(1);
            Y_p = Y_path(1);
            
            %% [Step 3] 가속도 포화 영역(Max Acc Region) 다각형 생성
            sigma_p = sigma_ref;
            sigma_t_arr = linspace(0, 2*pi, 200); 
            
            numerator = V_p * (V_t * sin(sigma_t_arr) - V_p * sin(sigma_p));
            term_feedback = V_p * gain_k * (sigma_p - sigma_ref);
            
            r_pos = max(0, numerator ./ (a_max + term_feedback));
            r_neg = max(0, numerator ./ (-a_max + term_feedback));
            
            psi_t_rad = pi/2; 
            plot_angle_vec = psi_t_rad + pi - sigma_t_arr;
            
            X_pos = r_pos .* cos(plot_angle_vec);
            Y_pos = r_pos .* sin(plot_angle_vec);
            X_neg = r_neg .* cos(plot_angle_vec);
            Y_neg = r_neg .* sin(plot_angle_vec);
            
            %% [Step 4] polyxpoly를 사용한 위험/안전 판별 및 거리 계산
            [xi_pos, yi_pos] = polyxpoly(X_path, Y_path, X_pos, Y_pos);
            [xi_neg, yi_neg] = polyxpoly(X_path, Y_path, X_neg, Y_neg);
            
            xi_all = [xi_pos; xi_neg];
            yi_all = [yi_pos; yi_neg];
            
            if isempty(xi_all)
                % 교차점이 아예 없음 -> 안전
                plot_color = 'b'; % Blue
            else
                % 교차점이 존재함 -> 거리 계산
                if length(xi_all) >= 2
                    cross_dist = sqrt((xi_all(end) - xi_all(1))^2 + (yi_all(end) - yi_all(1))^2);
                else
                    cross_dist = 0; % 점이 1개면 스친 거니까 거리는 0
                end
                
                % 관통 거리가 2m 이하인지 판별
                if cross_dist <= epsilon
                    plot_color = [0 0.8 0];   % Green (Semi-안전)
                else
                    plot_color = [1 0.4 0.4]; % Red (위험)
                end
            end
            
            %% [Step 5] 결과 점 찍기
            plot(X_p, Y_p, '.', 'Color', plot_color, 'MarkerSize', 8, 'HandleVisibility', 'off');
        end
    end
    
    %% 4. 마무리 시각화 디자인
    title('Reachability Map via polyxpoly (Safe / Semi-Safe / Danger)', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Relative X [m]');
    ylabel('Relative Y [m]');
    
    % [수정] sprintf를 사용해서 epsilon 값이 포함된 문자열을 미리 생성해!
    semi_safe_str = sprintf('Semi-Safe (Dist $\\le$ %.1f m)', epsilon);
    danger_str = sprintf('Danger (Dist $>$ %.1f m)', epsilon);

    % 범례용 투명 점 찍기
    plot(NaN, NaN, 'b.', 'MarkerSize', 15, 'DisplayName', 'Safe (No Intersection)');
    
    % [수정] 위에서 만든 문자열 변수를 DisplayName에 대입해줘~
    plot(NaN, NaN, '.', 'Color', [0 0.8 0], 'MarkerSize', 15, 'DisplayName', semi_safe_str);
    plot(NaN, NaN, '.', 'Color', [1 0.4 0.4], 'MarkerSize', 15, 'DisplayName', danger_str);
    
    % LaTeX 인터프리터를 사용해야 수식이 예쁘게 나와!
    legend('Interpreter', 'latex', 'Location', 'best');
    
    xlim([-1100, 10]);
    ylim([-10, 1100]);
    
    fprintf('>>> 분석 종료.\n');
end