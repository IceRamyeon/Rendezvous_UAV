function Randezvous_Region(cfg)
    %% 파라미터 및 시뮬레이션 설정
    R2D = 180/pi;
    D2R = pi/180;
    
    % [GUIDANCE_MODE 설정]
    GUIDANCE_MODE = cfg.GUIDANCE_MODE;
    
    % --- Simulation Time Param ---
    dt = cfg.dt_region;       
    tf = cfg.tf;       
    
    % --- Vehicle Spec ---
    V_p = cfg.V_p;      
    V_t = cfg.V_t;       
    At_constant = cfg.At_constant;     
    At_cmd = cfg.At_cmd;
    
    % --- Guidance Object 생성 (Simulation 코드와 동기화) ---
    switch GUIDANCE_MODE
        case 'PPNG'
            missile_guidance = PPNG(cfg.N, cfg.bias_acc, cfg.limit_acc); 

        case 'DPG'
            if isfield(cfg, 'target_lead_angle_deg')
                dpg_init_val = cfg.target_lead_angle_deg;
            else
                dpg_init_val = 0; 
            end
            missile_guidance = DPG(dpg_init_val, cfg.gain_k, cfg.limit_acc);
        
        case 'VDPG'
            missile_guidance = VDPG(cfg.gain_k, cfg.limit_acc);

        case 'RDPG'
            missile_guidance = RDPG(cfg.gain_k, cfg.limit_acc, cfg.r_allow, ...
                                    cfg.rate_limit, dt, 0, cfg.alpha, cfg.RDPG_test);
                                    
        case 'RDPG_T'
            missile_guidance = RDPG_T(cfg.gain_k, cfg.limit_acc, cfg.r_allow, ...
                                      cfg.rate_limit, dt, 0, ...
                                      cfg.RDPG_T_MODE, cfg.alpha, cfg.sigma_offset, cfg.RDPG_test);

        otherwise
            error('모드 없음');
    end
    
    % --- [Sweep Condition] 조건 설정 ---
    r_min_km = cfg.r_min_km; 
    r_max_km = cfg.r_max_km;     
    r_step_km = cfg.r_step_km;                  
    r_vec = (r_min_km:r_step_km:r_max_km) * 1000; 
    
    % 타겟 진행방향(North) 기준:
    b_min_deg = cfg.b_min_deg; 
    b_max_deg = cfg.b_max_deg;
    b_step_deg = cfg.b_step_deg; 
    b_vec = (b_min_deg:b_step_deg:b_max_deg) * D2R; 
    
    % --- [Success Criteria] 성공 기준 (Simulation과 동일) ---
    r_allow = cfg.r_allow;
    th_psi_deg = cfg.th_psi_deg;        
    
    if isfield(cfg, 'r_theory_m')
        r_theory_limit = cfg.r_theory_m;
    else
        r_theory_limit = 5; 
    end

    % 분석할 초기 리드각 목록
    sigma_cases_deg = cfg.sigma_cases_deg; 
    
    % 데이터 저장을 위한 변수 초기화
    collected_data = []; 
    
    %% 2. 시뮬레이션 및 플롯 준비
    figure('Position', [100 50 1400 900], 'Theme', 'light'); 
    t = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    
    switch GUIDANCE_MODE
        case 'PPNG'
            title_str = sprintf('[Reachability Map] Mode: %s (N=%.1f)', GUIDANCE_MODE, cfg.N);
        case 'VDPG'
            title_str = sprintf('[Reachability Map] Mode: %s (K=%.1f)', GUIDANCE_MODE, cfg.gain_k);
        case 'DPG'
            if isfield(cfg, 'target_lead_angle_deg')
                str_sig = sprintf('%.1f^\\circ', cfg.target_lead_angle_deg);
            else
                str_sig = 'Hold'; 
            end
            title_str = sprintf('[Reachability Map] Mode: %s (K=%.1f, \\sigma_{d} = %s)', ...
                GUIDANCE_MODE, cfg.gain_k, str_sig);
        case 'RDPG'
            title_str = sprintf('[Reachability Map] Mode: %s (K=%.1f, \\alpha=%.1f)', GUIDANCE_MODE, cfg.gain_k, cfg.alpha);
        case 'RDPG_T'
            title_str = sprintf('[Reachability Map] Mode: %s (K=%.1f, \\alpha=%.1f)', GUIDANCE_MODE, cfg.gain_k, cfg.alpha);
    end
    
    title(t, title_str, 'FontSize', 16, 'FontWeight', 'bold', 'Color', 'k');
    
    % Target 초기 정보 (북쪽 90도 고정)
    Xt_i = 0; Yt_i = 0;
    psi_ti_deg = 90; psi_t = psi_ti_deg * D2R;
    
    %% 3. 케이스별 루프 시작
    for s_idx = 1:length(sigma_cases_deg)
        current_sigma_deg = sigma_cases_deg(s_idx);
        current_sigma_rad = current_sigma_deg * D2R;
        
        % DPG 목표값 설정
        if strcmp(GUIDANCE_MODE, 'DPG')
            if isfield(cfg, 'target_lead_angle_deg')
                missile_guidance.sigma_des = cfg.target_lead_angle_deg * D2R;
            else
                missile_guidance.sigma_des = current_sigma_rad; 
            end
        end
        
        fprintf('>>> 분석 진행 중: 초기 리드각 %d도 (%d/%d) <<<\n', ...
            current_sigma_deg, s_idx, length(sigma_cases_deg));
        
        % 플롯용 데이터 초기화
        Suc_X = []; Suc_Y = [];
        Fail_X = []; Fail_Y = [];
        
        % --- 거리 및 방위각 루프 ---
        for r_idx = 1:length(r_vec)
            r_start = r_vec(r_idx);
            
            for b_idx = 1:length(b_vec)
                bearing = b_vec(b_idx);
                
                % 1. 초기 위치 계산
                theta_global = psi_t - bearing; 
                Xp_i = Xt_i + r_start * cos(theta_global);
                Yp_i = Yt_i + r_start * sin(theta_global);
                
                % 2. 초기 헤딩(psi_p) 설정
                dx = Xt_i - Xp_i; dy = Yt_i - Yp_i;
                lambda_init = atan2(dy, dx);
                cur_psi_p = lambda_init + current_sigma_rad; 
                
                init_psi_p = cur_psi_p;
    
                % 변수 초기화
                cur_Xp = Xp_i; cur_Yp = Yp_i;
                cur_Xt = Xt_i; cur_Yt = Yt_i;
                cur_psi_t = psi_t;
                
                % [중요] RDPG/RDPG_T 내부 상태(LPF) 초기화
                if strcmp(GUIDANCE_MODE, 'RDPG') || strcmp(GUIDANCE_MODE, 'RDPG_T')
                     missile_guidance.sigma_ref_prev = current_sigma_rad;
                end
                
                % RDPG_T 변수
                is_stop = 0; 
                is_success = false; % 성공 플래그 초기화
                sim_time = 0;
                
                % --- Trajectory Integration ---
                while sim_time < tf
                    % 기하학
                    Rx = cur_Xt - cur_Xp; Ry = cur_Yt - cur_Yp;
                    r = sqrt(Rx^2 + Ry^2);
                    lambda = atan2(Ry, Rx);
                    
                    sigma_p = cur_psi_p - lambda;
                    sigma_t = cur_psi_t - lambda;
                    
                    % 상대 운동
                    r_dot = V_t * cos(sigma_t) - V_p * cos(sigma_p);
                    V_c = -r_dot;
                    lambda_dot = (V_t * sin(sigma_t) - V_p * sin(sigma_p)) / r;
                    
                    % ---------------------------------------------------
                    % [Success Check] Simulation 코드와 동일 로직 적용
                    % ---------------------------------------------------
                    % 각도 오차 계산 (Wrap To 180)
                    heading_diff_deg = abs(mod((cur_psi_p - cur_psi_t)*R2D + 180, 360) - 180);
                    
                    % 성공 조건: 거리 만족 AND 헤딩 정렬 만족
                    % (Simulation에서는 V_c < 0 등을 따지지만, 핵심은 r과 heading임)
                    if (r <= r_allow) && (heading_diff_deg <= th_psi_deg)
                        is_success = true;
                        break; % 성공했으니 루프 탈출!
                    end
                    
                    % [Divergence Check] 너무 멀어지면 시간 낭비니까 중단 (선택사항)
                    if r > 1100 
                        break; 
                    end
                    
                    % ---------------------------------------------------
                    % [Guidance & Control]
                    % ---------------------------------------------------
                    At_current = At_constant; 

                    switch GUIDANCE_MODE
                        case 'PPNG'
                            acc_cmd = missile_guidance.compute_commandPPNG(V_c, lambda_dot);
                        case 'DPG'
                            acc_cmd = missile_guidance.compute_commandDPG(V_p, lambda_dot, sigma_p);
                        case 'VDPG'
                            acc_cmd = missile_guidance.compute_commandVDPG(...
                                V_p, lambda_dot, sigma_p, r, r_start, current_sigma_rad);
                        case 'RDPG'
                            [acc_cmd, ~] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t);
                        case 'RDPG_T'
                            [acc_cmd, ~, tanker_turn] = missile_guidance.compute_commandRDPG(V_p, lambda_dot, sigma_p, r, sigma_t, r_dot, is_stop);
                            if tanker_turn ~= 0
                                At_current = At_cmd * tanker_turn;
                            else
                                At_current = 0;
                            end
                    end
                    
                    % 적분
                    cur_psi_p = cur_psi_p + (acc_cmd/V_p)*dt;
                    cur_psi_t = cur_psi_t + (At_current/V_t)*dt;
                    
                    cur_Xp = cur_Xp + V_p * cos(cur_psi_p) * dt;
                    cur_Yp = cur_Yp + V_p * sin(cur_psi_p) * dt;
                    cur_Xt = cur_Xt + V_t * cos(cur_psi_t) * dt;
                    cur_Yt = cur_Yt + V_t * sin(cur_psi_t) * dt;
                    
                    sim_time = sim_time + dt;
                end
                
                % 결과 저장
                if is_success
                    Suc_X(end+1) = Xp_i; Suc_Y(end+1) = Yp_i;
                    new_data = [r_start, init_psi_p * R2D, bearing * R2D, current_sigma_deg];
                    collected_data = [collected_data; new_data];
                else
                    Fail_X(end+1) = Xp_i; Fail_Y(end+1) = Yp_i;
                end
            end
        end
        
        % --- 서브플롯 그리기 ---
        ax = nexttile;
        set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.2);
        hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');

        % 1. 시뮬레이션 결과 점 찍기
        if ~isempty(Fail_X), plot(ax, Fail_X, Fail_Y, '.', 'MarkerSize', 5, 'Color', [1 0.4 0.4]); end
        if ~isempty(Suc_X), plot(ax, Suc_X, Suc_Y, '.', 'MarkerSize', 6, 'Color', 'b'); end
        plot(ax, 0, 0, '^', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'Color', 'g');

        % 2. ★ 이론적 Reachable Region 덧그리기 ★
        switch GUIDANCE_MODE
            case 'PPNG'
                if exist('draw_PPNG_Region', 'file')
                    draw_PPNG_Region(ax, current_sigma_rad, r_theory_limit, cfg.N);
                end
            case 'DPG'
                if exist('draw_DPG_Region', 'file')
                    draw_DPG_Region(ax, current_sigma_rad, r_theory_limit);
                end
            case {'RDPG', 'RDPG_T'}
                 if exist('draw_DPG_Region', 'file')
                    draw_DPG_Region(ax, current_sigma_rad, r_theory_limit);
                 end
        end
        
        title(ax, sprintf('\\sigma_{p0} = %d^\\circ', current_sigma_deg), 'Color', 'k');
        if s_idx > 3, xlabel('East [m]', 'Color', 'k'); end
        if mod(s_idx, 3) == 1, ylabel('North [m]', 'Color', 'k'); end
        
        draw_max = abs(r_max_km) * 1000 + 100;
        xlim([-draw_max 10]); ylim([-10 draw_max]); 
        drawnow;
    end
    
    fprintf('시뮬레이션 완료!\n');
    
    %% 4. 결과 CSV 저장
    if ~isempty(collected_data)
        T = array2table(collected_data, ...
            'VariableNames', {'Initial_Range_m', 'Initial_Pursuer_Heading_Deg', 'Initial_Bearing_Deg', 'Sigma_Case_Deg'});
        
        switch GUIDANCE_MODE
            case 'DPG'
                if isfield(cfg, 'target_lead_angle_deg')
                    filename = sprintf('Success_Data_%s_V=%.1f_m_s,TargetSigma=%.1f,r_allow = %.1f.csv', ...
                        GUIDANCE_MODE, V_t, cfg.target_lead_angle_deg, r_allow);
                else
                    filename = sprintf('Success_Data_%s_V=%.1f_m_s_SigmaHold.csv', ...
                        GUIDANCE_MODE, V_t);
                end

            case {'RDPG', 'RDPG_T'}
                filename = sprintf('Success_Data_%s_V=%.1f_m_s,r_allow=%.1f,alpha=%.1f.csv', GUIDANCE_MODE, V_t, r_allow, cfg.alpha);
            otherwise
                filename = sprintf('Success_Data_%s_V=%.1f_m_s.csv', GUIDANCE_MODE, V_t);
        end
        try
            writetable(T, filename);
            fprintf('\n>>> 데이터 저장 완료! 파일명: %s (%d개 데이터) <<<\n', filename, height(T));
        catch ME
            fprintf('\n>>> 으헤? 파일이 열려 있어서 저장이 안 되네. (Error: %s) <<<\n', ME.message);
            timestamp = char(datetime('now', 'Format', '_HHmmss'));
            new_filename = strrep(filename, '.csv', [timestamp '.csv']);
            fprintf('>>> 걱정 마, "%s"로 저장할게.\n', new_filename);
            writetable(T, new_filename);
            fprintf('>>> 비상 저장 완료! (%d개 데이터) <<<\n', height(T));
        end
    else
        fprintf('\n>>> 성공한 데이터 없음. <<<\n');
    end
end