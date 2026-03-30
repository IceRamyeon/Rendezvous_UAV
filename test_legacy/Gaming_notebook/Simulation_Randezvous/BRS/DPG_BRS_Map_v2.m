function DPG_BRS_Map_v2(n)
    % 으헤~ 1m 간격으로 점을 찍어서 검사하는 Reachability Map이야!
    % 
    % [사용법]
    % Draw_Analytical_Map_v2(5)  % Max Acc 초과 점이 5개 이상이면 실패
    % 입력 변수 n이 없으면 기본값으로 1을 사용해.

    close all

    if nargin < 1
        n = 1; % 기본값: 1개라도 포화 영역에 들어가면 실패
    end

    %% 1. 파라미터 세팅
    V_p = 20.0;
    V_t = 20.0;
    limit_acc = 1.0; 
    a_max = limit_acc * 9.81;
    r_f = 2.0;
    delta = 1.0; % [m] 점 간격 (delta)
    
    % Sweep 조건 (조금 촘촘하게 해서 예쁘게 찍어볼게)
    r_vec = 100:10:1000;      % [m] 0.1km ~ 1.0km
    b_vec = -90:1:0;       % [deg] Bearing -180도 ~ 180도

    %% 2. 피규어 준비
    figure('Name', 'Analytical Reachability Map (Discrete Pts)', 'Theme', 'light', 'Position', [100, 100, 700, 600]);
    hold on; grid on; axis equal;
    
    % Target 위치 (중앙)
    Xt = 0; Yt = 0;
    psi_t = pi/2; % Target은 북쪽(90도) 바라봄

    % Target 위치만 살짝 표시
    quiver(Xt, Yt, 0, 100, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');
    plot(Xt, Yt, 'ko', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');

    fprintf('>>> 으헤~ 1m 간격으로 콕콕 찌르면서 검사할게! 점이 많아서 조금 걸릴지도 몰라~\n');

    %% 3. 점 생성 및 계산 루프
    for r_idx = 1:length(r_vec)
        r_init = r_vec(r_idx);
        
        for b_idx = 1:length(b_vec)
            bearing_deg = b_vec(b_idx);
            
            % [Step 1] 초기 위치 및 시선각 계산
            global_los_rad = psi_t - deg2rad(bearing_deg);
            Xp = Xt + r_init * cos(global_los_rad);
            Yp = Yt + r_init * sin(global_los_rad);
            
            lam_init = atan2(Yt - Yp, Xt - Xp);
            sigma_t_init = abs(wrapToPi(psi_t - lam_init)); 

            % [Step 2] 논문 공식으로 sigma_ref 역산 찾기
            f_eqn = @(x) cos(x) - sqrt(r_f/r_init) * cos((x + sigma_t_init)/2);
            
            try
                % 해가 존재하는지 체크
                if f_eqn(0) * f_eqn(pi/2 - 1e-5) <= 0
                    opt = optimset('Display', 'off'); 
                    sigma_ref = fzero(f_eqn, [0, pi/2 - 1e-5], opt);
                else
                    sigma_ref = fzero(f_eqn, 0.1, optimset('Display', 'off')); 
                end
                
                % [Step 3 & 4 & 5] 도달 가능 여부 및 가속도 검사
                if sigma_t_init > pi - sigma_ref
                    plot_color = [1 0.4 0.4]; % Red (Target 뒤쪽 도달 불가)
                else
                    % [Step 3] 1m 간격 추출을 위해 먼저 촘촘한 곡선(Base Curve) 생성
                    sig_dense = linspace(sigma_t_init, sigma_ref, 2000);
                    r_dense = r_f * (cos(sigma_ref)^2) ./ (cos((sig_dense + sigma_ref)/2).^2);
                    
                    % 궤적의 누적 이동 거리(Arc Length) 계산
                    X_dense = r_dense .* cos(-pi/2 + sig_dense);
                    Y_dense = r_dense .* sin(-pi/2 + sig_dense);
                    S_cum = [0, cumsum(sqrt(diff(X_dense).^2 + diff(Y_dense).^2))];
                    
                    % delta(1m) 간격으로 점을 찍기 위한 내삽(Interpolation)
                    S_query = 0:delta:S_cum(end);
                    if isempty(S_query)
                        S_query = 0; % 길이가 1m 미만일 때 예외 처리
                    end
                    
                    sig_pts = interp1(S_cum, sig_dense, S_query);
                    r_pts = interp1(S_cum, r_dense, S_query);
                    
                    % [Step 4] delta=1m로 찍힌 점들에서 요구 가속도 계산 (sigma_p = sigma_ref 가정)
                    req_acc = (V_p ./ r_pts) .* (V_t * sin(sig_pts) - V_p * sin(sigma_ref));
                    
                    % [Step 5] Max Acc 영역(가속도 포화)에 들어간 점 개수 카운트
                    num_sat_pts = sum(abs(req_acc) > a_max);
                    
                    % n개 이상이면 실패, 미만이면 성공 판정
                    if num_sat_pts >= n
                        plot_color = [1 0.4 0.4]; % Red (Failure)
                    else
                        plot_color = 'b'; % Blue (Success)
                    end
                end
            catch
                % 식 풀이에 실패하면 빨간색 처리
                plot_color = [1 0.4 0.4]; % Red (Failure)
            end
            
            % [Step 6] 결과 점 찍기 (시각적 과정 생략)
            plot(Xp, Yp, '.', 'Color', plot_color, 'MarkerSize', 8, 'HandleVisibility', 'off');
        end
    end

    % 마무리 레이블 및 범례 디자인
    title(sprintf('Analytical Reachability Map (Failure if $\\ge %d$ pts saturated)', n), ...
        'Interpreter', 'latex', 'FontSize', 14);
    xlabel('East [m]');
    ylabel('North [m]');
    
    % 범례용 투명 점 찍기
    plot(NaN, NaN, 'b.', 'MarkerSize', 15, 'DisplayName', 'Success (Safe)');
    plot(NaN, NaN, '.', 'Color', [1 0.4 0.4], 'MarkerSize', 15, 'DisplayName', 'Failure (Acc Saturated)');
    legend('Location', 'best');
    
    xlim([-1100, 10]);
    ylim([-10, 1100]);

    fprintf('>>> 으헤~ 맵 생성 완료! 아저씨가 부탁한 결과만 예쁘게 찍어놨어~\n');
end