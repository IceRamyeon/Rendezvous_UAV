function BRS_DPG_1(r, bearing_deg, n)
    % Target은 (0,0)에 위치하며, +y 방향(빨간색 화살표)을 바라봅니다.
    % Pursuer는 입력받은 Target 기준 전방 좌반 평면에 형성됩니다.
    % 가속도 포화 영역(Max Acc Region) 계산 코드를 내장하여 한 번에 그립니다.
    % [추가] inpolygon을 사용하여 궤적의 Max Acc 영역 침범 여부를 판별합니다.
    
    close all;
    
    if nargin < 3
        n = 200;
    end
    
    %% 0. Initialization & Parameters
    r_f = 2.0; % [m] 허용 오차 거리
    
    % [Max Acc Region 전용 파라미터]
    V_p = 20.0;       % Pursuer 속도
    V_t = 20.0;       % Target 속도
    gain_k = 5.0;     % 유도 게인 K
    limit_acc = 1.0;  % 가속도 제한 (G)
    a_max = limit_acc * 9.81; % G -> m/s^2
    
    % 전방 좌반 평면 형성을 위해 방위각의 절댓값을 취함
    sigma_t_init = deg2rad(abs(bearing_deg)); 
    
    %% 1. 비선형 방정식으로 sigma_ref 찾기
    eq = @(s_ref) cos(s_ref) - sqrt(r/r_f) * cos((s_ref + sigma_t_init)/2);
    
    % 800m처럼 먼 거리에서는 둔각 해를 안정적으로 찾도록 초기값 설정
    sigma_ref = fzero(eq, pi - sigma_t_init);
    
    %% 2. 예상 Rendezvous Trajectory 생성
    sigma_path = linspace(sigma_t_init, sigma_ref, n);
    rho_path = r_f * (cos(sigma_ref) ./ cos((sigma_ref + sigma_path)/2)).^2;
    lambda_path = pi/2 + sigma_path; 
    
    X_path = rho_path .* cos(lambda_path);
    Y_path = rho_path .* sin(lambda_path);
    
    X_p = X_path(1);
    Y_p = Y_path(1);
    
    %% 3. 가속도 포화 영역(Max Acc Region) 좌표 계산
    sigma_p = sigma_ref;
    
    % 다각형 생성을 위해 0 ~ 2*pi까지 점 생성 (시작과 끝이 만나 닫힌 다각형이 됨)
    sigma_t_arr = linspace(0, 2*pi, 500); 
    
    numerator = V_p * (V_t * sin(sigma_t_arr) - V_p * sin(sigma_p));
    term_feedback = V_p * gain_k * (sigma_p - sigma_ref);
    
    r_pos = numerator ./ (a_max + term_feedback);
    r_neg = numerator ./ (-a_max + term_feedback);
    
    r_pos_clamped = max(0, r_pos);
    r_neg_clamped = max(0, r_neg);
    
    psi_t_rad = pi/2; 
    plot_angle_vec = psi_t_rad + pi - sigma_t_arr;
    
    X_pos = r_pos_clamped .* cos(plot_angle_vec);
    Y_pos = r_pos_clamped .* sin(plot_angle_vec);
    
    X_neg = r_neg_clamped .* cos(plot_angle_vec);
    Y_neg = r_neg_clamped .* sin(plot_angle_vec);
    
    %% 4. [NEW] inpolygon을 통한 위험/안전 판별
    % 궤적의 (X_path, Y_path) 점들이 +Acc 또는 -Acc 다각형 내부에 있는지 검사
    in_pos = inpolygon(X_path, Y_path, X_pos, Y_pos);
    in_neg = inpolygon(X_path, Y_path, X_neg, Y_neg);
    
    % 단 한 점이라도 내부(1)에 들어갔다면 위험 판정!
    is_danger = any(in_pos) || any(in_neg);
    
    fprintf('\n========================================\n');
    if is_danger
        fprintf('>>> 으헤~ 궤적이 Max Acc 영역을 스쳐 지나가! [위험]\n');
    else
        fprintf('>>> 으헤~ 궤적이 Max Acc 영역을 깔끔하게 피해가네~ [안전]\n');
    end
    fprintf('========================================\n\n');
    
    %% 5. 시각화 (Plotting)
    figure('Name', 'DPG Rendezvous Trajectory with Max Acc', 'Theme', 'light', 'Position', [150, 150, 700, 600]);
    ax = gca; 
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    
    % 1) Max Acc 영역 그리기 (가장 밑에 깔리도록 먼저 그림)
    if any(r_pos > 0)
        fill(ax, X_pos, Y_pos, 'c', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(ax, X_pos, Y_pos, 'c-.', 'LineWidth', 1.5, 'DisplayName', '+Acc Limit');
    end
    if any(r_neg > 0)
        fill(ax, X_neg, Y_neg, 'm', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(ax, X_neg, Y_neg, 'm-.', 'LineWidth', 1.5, 'DisplayName', '-Acc Limit');
    end
    
    % 2) BRS 예상 궤적 (파란색 실선)
    plot(ax, X_path, Y_path, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Rendezvous Trajectory');
    
    % [NEW] 위험 판정 시 영역을 침범한 포인트들을 빨간색 별표로 강조
    if is_danger
        danger_idx = in_pos | in_neg;
        plot(ax, X_path(danger_idx), Y_path(danger_idx), 'r*', 'MarkerSize', 5, 'DisplayName', 'Danger Points');
    end
    
    % 3) Pursuer 시작 위치 (파란색 사각형)
    plot(ax, X_p, Y_p, 'bs', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Pursuer Start');
    
    % 4) Target 위치 및 방향 (원점, 빨간색 화살표)
    arrow_length = min(max(r, 5) * 0.15, 120); 
    quiver(ax, 0, 0, 0, arrow_length, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'DisplayName', 'Target Heading');
    plot(ax, 0, 0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'HandleVisibility', 'off');
    
    % 축 및 레이블 디자인
    title(ax, sprintf('Rendezvous Trajectory ($r = %.1f$ m, $\\theta = %d^\\circ$)', r, abs(bearing_deg)), ...
        'Interpreter', 'latex', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel(ax, 'Relative X (m)');
    ylabel(ax, 'Relative Y (m)');
    
    % 여백을 주어 그래프가 잘리지 않도록 범위 설정
    xlim(ax, [-1000, 100]);
    ylim(ax, [-100, 1000]);
    
    legend(ax, 'Location', 'best');
    
    %% 6. 데이터 저장 (CSV 파일 추출)
    trajectory_data = [X_path(:), Y_path(:)];
    writematrix(trajectory_data, 'Trajectory_Data.csv');
    
    % 디버깅 및 확인용 출력
    fprintf('>>> 계산 완료: sigma_ref = %.2f deg\n', rad2deg(sigma_ref));
end