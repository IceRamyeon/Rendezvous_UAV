function DPG_BRS_Map()
    % 으헤~ 아저씨가 부탁한 해석적 기법(Analytical) 기반 Reachability Map이야!
    % 1. 거리와 Bearing Sweep으로 점 생성
    % 2. 논문 공식으로 sigma_ref 역산
    % 3. 곡선 길이 및 4. 가속도 포화(Max Acc) 대조
    % 5 & 6. 결과 점(파랑/빨강)만 그리기
    
    close all;

    %% 1. 파라미터 세팅 (main_rand.m 참고)
    V_p = 20.0;
    V_t = 20.0;
    limit_acc = 1.0; 
    a_max = limit_acc * 9.81;
    r_f = 2.0;
    
    % 아저씨가 말한 곡선 허용 길이 (이 길이 이상 Max Acc를 초과하면 실패!)
    epsilon = 0.5; % [m] 

    % Sweep 조건 (조금 더 조밀하게 해서 맵이 꽉 차 보이게 했어~)
    r_vec = 100:20:1000;      % [m] 0.1km ~ 1.0km
    b_vec = -180:3:180;       % [deg] Bearing -180도 ~ 180도

    %% 2. 피규어 준비
    figure('Name', 'Analytical Reachability Map', 'Theme', 'light', 'Position', [100, 100, 700, 600]);
    hold on; grid on; axis equal;
    
    % Target 위치 (중앙)
    Xt = 0; Yt = 0;
    psi_t = pi/2; % Target은 북쪽(90도) 바라봄

    % Target 화살표 예쁘게 그려주기
    quiver(Xt, Yt, 0, 100, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'HandleVisibility', 'off');
    plot(Xt, Yt, 'ko', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');

    fprintf('>>> 으헤~ 계산 시작할게! 점이 쪼오금 많아서 2~3초 걸릴 수도 있어~ 기다려!\n');

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
            sigma_t = abs(wrapToPi(psi_t - lam_init)); % Target 리드각(절댓값)

            % [Step 2] 논문 공식으로 sigma_ref 역산 찾기
            % 방정식: cos(x) - sqrt(r_f/r) * cos((x + sigma_t)/2) = 0
            f_eqn = @(x) cos(x) - sqrt(r_f/r_init) * cos((x + sigma_t)/2);
            
            try
                % 해(root)가 존재하는지 먼저 체크 후 fzero 실행
                if f_eqn(0) * f_eqn(pi/2 - 1e-5) <= 0
                    % 옵션을 꺼서 콘솔 창 지저분해지는 거 방지!
                    opt = optimset('Display', 'off'); 
                    sigma_ref = fzero(f_eqn, [0, pi/2 - 1e-5], opt);
                else
                    sigma_ref = fzero(f_eqn, 0.1, optimset('Display', 'off')); 
                end
                
                % [Step 3 & 4 & 5] 도달 가능 여부 (곡선 및 Max Acc 분석)
                if sigma_t > pi - sigma_ref
                    % 이론적으로 Target 뒤쪽으로 넘어가 도달 불가능한 영역
                    plot_color = [1 0.4 0.4]; % Red
                else
                    % [Step 3] BRS 곡선을 따라 sigma_t -> sigma_ref 까지 궤적 생성
                    sig_curve = linspace(sigma_t, sigma_ref, 200);
                    r_curve = r_f * (cos(sigma_ref)^2) ./ (cos((sig_curve + sigma_ref)/2).^2);
                    
                    % [Step 4] 요구 가속도 계산 (sigma_p = sigma_ref로 유지된다고 가정)
                    req_acc = (V_p ./ r_curve) .* (V_t * sin(sig_curve) - V_p * sin(sigma_ref));
                    
                    % 곡선의 미소 길이 계산 (Cartesian 상의 이동 거리)
                    dr = diff(r_curve);
                    dsig = diff(sig_curve);
                    dl = sqrt(dr.^2 + (r_curve(1:end-1) .* dsig).^2);
                    
                    % 가속도 한계(Max Acc) 초과 여부 확인
                    is_sat = abs(req_acc(1:end-1)) > a_max;
                    
                    % 초과한 구간의 총 길이(dl) 합산
                    sat_length = sum(dl(is_sat));
                    
                    % [Step 5] epsilon(0.5m) 이상이면 실패, 미만이면 성공
                    if sat_length >= epsilon
                        plot_color = [1 0.4 0.4]; % Red (Failure)
                    else
                        plot_color = 'b'; % Blue (Success)
                    end
                end
            catch
                % fzero 계산 실패 시 도달 불가능으로 판정
                plot_color = [1 0.4 0.4]; % Red (Failure)
            end
            
            % [Step 6] 성공(파랑) / 실패(빨강) 점 그리기
            plot(Xp, Yp, '.', 'Color', plot_color, 'MarkerSize', 8, 'HandleVisibility', 'off');
        end
    end

    % 마무리 레이블 및 범례 디자인
    title('Analytical Reachability Map (Max Acc Length Check)', 'FontSize', 14);
    xlabel('East [m]');
    ylabel('North [m]');
    
    % 범례용 투명 점 찍기 (실제 점들은 HandleVisibility를 껐거든!)
    plot(NaN, NaN, 'b.', 'MarkerSize', 15, 'DisplayName', 'Success (Safe)');
    plot(NaN, NaN, '.', 'Color', [1 0.4 0.4], 'MarkerSize', 15, 'DisplayName', 'Failure (Acc Saturated)');
    legend('Location', 'best');
    
    % 축 이쁘게 정렬
    xlim([-1100, 1100]);
    ylim([-1100, 1100]);

    fprintf('>>> 으헤~ 맵 생성 완료! 시뮬레이션 돌렸을 때랑 모양이 쏙 빼닮았을 거야~\n');
end