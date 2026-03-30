function DPG_BRS_Map_inpolygon()
    % 으헤~ 거리와 방위각을 촘촘히 훑으면서 궤적이 가속도 영역을 침범하는지 검사하는 Map이야!
    
    close all;
    
    %% 1. 파라미터 세팅
    r_f = 2.0; % [m] 허용 오차 거리
    V_p = 20.0;
    V_t = 20.0;
    gain_k = 5.0;     
    limit_acc = 1.0;  
    a_max = limit_acc * 9.81; % G -> m/s^2
    
    % Sweep 조건 (선생 컴퓨터가 안 터질 정도로 적당히 촘촘하게 잡았어!)
    r_vec = 100:20:1000;     % [m] 100m ~ 1000m (20m 간격)
    b_vec = 0:2:90;          % [deg] 방위각 0도 ~ 90도 (2도 간격, 좌반평면 형성용)
    n_traj = 150;            % 궤적을 구성할 점의 개수 (속도를 위해 살짝 줄임)
    
    %% 2. 피규어 준비
    figure('Name', 'Reachability Map by inpolygon', 'Theme', 'light', 'Position', [150, 150, 700, 600]);
    hold on; grid on; axis equal;
    
    % Target 위치 (중앙, +y 방향)
    Xt = 0; Yt = 0;
    quiver(Xt, Yt, 0, 100, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');
    plot(Xt, Yt, 'ko', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');
    
    fprintf('>>> 으헤~ 거리랑 각도 섞어가면서 궤적 겹치는지 검사 시작할게. 잠깐만 누워있어~ Zzz...\n');
    
    %% 3. 점 생성 및 궤적-영역 교차 검사 루프
    for r = r_vec
        for b = b_vec
            % 전방 좌반 평면 형성을 위해 방위각 절댓값 사용
            sigma_t_init = deg2rad(b); 
            
            %% [Step 1] 비선형 방정식으로 sigma_ref 찾기 (둔각 탐색용 초기값 적용)
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
            sigma_t_arr = linspace(0, 2*pi, 200); % 속도를 위해 점 개수 200개로 조정
            
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
            
            %% [Step 4] inpolygon을 사용한 위험/안전 판별
            in_pos = inpolygon(X_path, Y_path, X_pos, Y_pos);
            in_neg = inpolygon(X_path, Y_path, X_neg, Y_neg);
            
            % 궤적 점이 다각형 내부에 단 하나라도 들어가면 위험!
            if any(in_pos) || any(in_neg)
                plot_color = [1 0.4 0.4]; % Red (위험 - Max Acc 침범)
            else
                plot_color = 'b';         % Blue (안전 - 침범 안 함)
            end
            
            %% [Step 5] 결과 점 찍기
            plot(X_p, Y_p, '.', 'Color', plot_color, 'MarkerSize', 8, 'HandleVisibility', 'off');
        end
    end
    
    %% 4. 마무리 시각화 디자인
    title('Reachability Map via Trajectory \& Max Acc Intersect', 'Interpreter', 'latex', 'FontSize', 14);
    xlabel('Relative X [m]');
    ylabel('Relative Y [m]');
    
    % 범례용 투명 점 찍기
    plot(NaN, NaN, 'b.', 'MarkerSize', 15, 'DisplayName', 'Safe (No Intersection)');
    plot(NaN, NaN, '.', 'Color', [1 0.4 0.4], 'MarkerSize', 15, 'DisplayName', 'Danger (Intersects Max Acc)');
    legend('Location', 'best');
    
    xlim([-1100, 10]);
    ylim([-10, 1100]);
    
    fprintf('>>> 으헤~ 맵 완성! 파란색은 안전 구역, 빨간색은 위험 구역이야~\n');
end