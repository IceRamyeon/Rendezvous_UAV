function Randezvous_MonteCarlo(cfg_base, N_sim)
    % cfg_base: main_rand.m에서 설정한 기본 설정 구조체
    % N_sim: 반복 횟수 (예: 1000)

    fprintf('>>> 몬테카를로 시뮬레이션 시작 (N=%d) <<<\n', N_sim);
    
    %% 1. 파라미터 추출
    dt = cfg_base.dt_simul;
    tf = cfg_base.tf;
    r_allow = cfg_base.r_allow;
    th_psi_deg = cfg_base.th_psi_deg;
    
    % [노이즈 설정] 표준편차 (이 값을 조절해서 불확실성을 테스트해!)
    sigma_pos = 10;       % [m] 초기 위치 오차
    sigma_vel = 0.5;      % [m/s] 속도 오차
    sigma_head = 1.0;     % [deg] 헤딩 오차
    
    %% 2. 결과 저장용 변수
    results.success = false(N_sim, 1);
    results.final_r = zeros(N_sim, 1);
    results.final_head_err = zeros(N_sim, 1);
    results.flight_time = zeros(N_sim, 1);
    
    % 진행률 표시바
    h_wait = waitbar(0, '시뮬레이션 진행 중...');
    
    %% 3. Monte Carlo Loop
    for k = 1:N_sim
        
        % --- [A] 초기 조건 Randomization ---
        % 기본 설정값 가져오기
        r_i = cfg_base.r_from_region_m;
        bearing_deg = cfg_base.bearing_from_region;
        psi_p_deg_base = cfg_base.psi_p_from_region + (cfg_base.psi_ti_deg - 90);
        
        % 노이즈 주입
        r_k = r_i + sigma_pos * randn;  % 거리 노이즈
        V_p_k = cfg_base.V_p + sigma_vel * randn; % 속도 노이즈
        
        % 각도 노이즈 (Target Heading 등)
        psi_t_deg_k = cfg_base.psi_ti_deg + sigma_head * randn;
        psi_p_deg_k = psi_p_deg_base + sigma_head * randn;
        
        % 좌표 변환
        psi_t = deg2rad(psi_t_deg_k);
        psi_p = deg2rad(psi_p_deg_k);
        bearing = deg2rad(bearing_deg);
        
        Xt_i = 0; Yt_i = 0; % Target은 원점 기준 상대위치로 생각
        global_los = psi_t - bearing;
        Xp_i = Xt_i + r_k * cos(global_los);
        Yp_i = Yt_i + r_k * sin(global_los);
        
        % 상태 변수 초기화
        curr_Xp = Xp_i; curr_Yp = Yp_i;
        curr_Xt = Xt_i; curr_Yt = Yt_i;
        curr_psi_p = psi_p;
        curr_psi_t = psi_t;
        
        % 유도탄 객체 생성 (매번 새로 생성하여 상태 리셋)
        dx = curr_Xt - curr_Xp; dy = curr_Yt - curr_Yp;
        lambda_init = atan2(dy, dx);
        init_sigma = curr_psi_p - lambda_init;
        
        % RDPG_T 객체 생성 (Noise가 섞인 초기값 반영)
        missile = RDPG_T(cfg_base.gain_k, cfg_base.limit_acc, cfg_base.r_allow, ...
                         cfg_base.rate_limit, dt, init_sigma, ...
                         cfg_base.RDPG_T_MODE, cfg_base.alpha, cfg_base.sigma_offset, cfg_base.RDPG_test);
                         
        % --- [B] Time Integration Loop (Silent Mode) ---
        is_success = false;
        final_time = tf;
        min_r = inf;
        final_head_diff = 0;
        is_stop_flag = 0; % RDPG_T용 플래그
        
        for t = 0:dt:tf
            % 1. 기하학 계산
            Rx = curr_Xt - curr_Xp; Ry = curr_Yt - curr_Yp;
            r = sqrt(Rx^2 + Ry^2);
            lambda = atan2(Ry, Rx);
            
            sigma_p = curr_psi_p - lambda;
            sigma_t = curr_psi_t - lambda;
            
            r_dot = cfg_base.V_t * cos(sigma_t) - V_p_k * cos(sigma_p);
            lambda_dot = (cfg_base.V_t * sin(sigma_t) - V_p_k * sin(sigma_p)) / r;
            V_c = -r_dot;
            
            % 2. 유도 명령 (RDPG_T)
            [acc_cmd, ~, tanker_turn] = missile.compute_commandRDPG(...
                V_p_k, lambda_dot, sigma_p, r, sigma_t, r_dot, is_stop_flag);
            
            % 3. 탱커 가속도 결정
            if tanker_turn ~= 0
                At_current = cfg_base.At_cmd * tanker_turn;
            else
                At_current = 0;
            end
            
            % 4. 적분 (Euler)
            omega_p = acc_cmd / V_p_k;
            curr_psi_p = curr_psi_p + omega_p * dt;
            curr_Xp = curr_Xp + V_p_k * cos(curr_psi_p) * dt;
            curr_Yp = curr_Yp + V_p_k * sin(curr_psi_p) * dt;
            
            omega_t = At_current / cfg_base.V_t;
            curr_psi_t = curr_psi_t + omega_t * dt;
            curr_Xt = curr_Xt + cfg_base.V_t * cos(curr_psi_t) * dt;
            curr_Yt = curr_Yt + cfg_base.V_t * sin(curr_psi_t) * dt;
            
            % 5. 성공 조건 확인 (Simulation.m의 로직 적용)
            heading_diff_deg = abs(wrapTo180(rad2deg(curr_psi_p - curr_psi_t)));
            
            % 최소 거리 기록 (분석용)
            if r < min_r, min_r = r; end
            
            reached = false;
            % 조건 1: CPA && 거리만족 && 각도만족
            if V_c < 0 && heading_diff_deg < th_psi_deg && r < r_allow
                reached = true;
            % 조건 2: 그냥 거리만족 && 각도만족
            elseif r < r_allow && heading_diff_deg < th_psi_deg
                reached = true;
            end
            
            if reached
                is_success = true;
                final_time = t;
                final_head_diff = heading_diff_deg;
                is_stop_flag = 1; % 유도 법칙에 정지 신호 전달
                break; % 루프 종료
            end
        end
        
        % 결과 저장
        results.success(k) = is_success;
        results.final_r(k) = r;       % 마지막 순간 거리 (성공 시 r_allow 이내)
        results.min_r(k) = min_r;     % 궤적 중 최소 거리
        results.final_head_err(k) = final_head_diff;
        results.flight_time(k) = final_time;
        
        if mod(k, 100) == 0
            waitbar(k/N_sim, h_wait, sprintf('진행 중... (%d/%d)', k, N_sim));
        end
    end
    close(h_wait);
    
    %% 4. 결과 분석 및 시각화
    success_count = sum(results.success);
    success_rate = (success_count / N_sim) * 100;
    
    fprintf('\n>>> Monte Carlo 결과 (N=%d) <<<\n', N_sim);
    fprintf('1. 성공률       : %.2f%% (%d/%d)\n', success_rate, success_count, N_sim);
    fprintf('2. 평균 비행시간 : %.2f 초\n', mean(results.flight_time(results.success)));
    fprintf('3. 성공 시 평균 오차: %.2f m\n', mean(results.final_r(results.success)));
    
    % 히스토그램 그리기
    figure('Name', 'Monte Carlo Analysis', 'Color', 'w');
    subplot(1,2,1);
    histogram(results.min_r, 30);
    title('Minimum Miss Distance Distribution');
    xlabel('Min Range [m]'); ylabel('Count');
    xline(r_allow, 'r--', 'LineWidth', 2, 'Label', 'Allow Limit');
    
    subplot(1,2,2);
    histogram(results.final_head_err(results.success), 20);
    title('Heading Error Distribution (Success Cases)');
    xlabel('Heading Error [deg]'); ylabel('Count');
    
end

% [Helper Function for Angle Wrapping]
function ang = wrapTo180(ang)
    ang = mod(ang + 180, 360) - 180;
end