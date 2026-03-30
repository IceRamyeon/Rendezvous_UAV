function draw_DPG_Region_max(ax, sigma_p, r_f, r_max_plot)
    % 으헤~ 지정한 Subplot(ax) 위에 덧그리는 함수야.
    
    % r_max_plot을 따로 안 넣으면 아저씨가 알아서 400으로 잡아줄게.
    if nargin < 4
        r_max_plot = 400; 
    end
    if nargin < 3
        r_f = 2.0; % 기본 타겟 반경
    end

    % 핵심! 지정된 축(ax)의 기존 그림이 안 날아가게 hold 상태를 유지해줘.
    hold(ax, 'on');

    % --- (1) BRS 수식에 따른 상수 K 계산 ---
    K_val = r_f * cos(sigma_p)^2;

    % --- (2) 반경(r)을 기준으로 촘촘하게 쪼개기 ---
    r_valid = linspace(r_f, r_max_plot, 500);

    % --- (3) 역함수(acos)를 이용해서 각도(sigma_t) 역산 ---
    sigma_t_valid = 2 * acos(sqrt(K_val ./ r_valid)) - sigma_p;

    % --- (4) 양쪽 대칭 각도 설정 (Target이 +y방향인 BRS 좌표계 맞춤) ---
    plot_angle_R = -pi/2 - sigma_t_valid;
    plot_angle_L = -pi/2 + sigma_t_valid;

    % --- (5) 극좌표 -> 직교좌표 변환 ---
    X_R = r_valid .* cos(plot_angle_R);
    Y_R = r_valid .* sin(plot_angle_R);

    X_L = r_valid .* cos(plot_angle_L);
    Y_L = r_valid .* sin(plot_angle_L);

    % 원점 -> 오른쪽 곡선 -> 왼쪽 곡선 -> 원점 순서로 폴리곤 닫기
    Region_X = [0, X_R, fliplr(X_L), 0];
    Region_Y = [0, Y_R, fliplr(Y_L), 0];

    % --- (6) Subplot(ax)에 콕 집어서 그리기 ---
    % fill과 plot 명령어 첫 번째 인자로 무조건 'ax'를 넣어줘야 거기로 쏙 들어가!
    fill(ax, Region_X, Region_Y, 'y', 'FaceAlpha', 0.2, 'EdgeColor', 'none', 'DisplayName', 'Reachable Region');

    plot(ax, X_R, Y_R, 'y--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(ax, X_L, Y_L, 'y--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
end