function Randezvous_Region_GPU(cfg)
% Randezvous_Region_GPU
% GPU Acceleration Version for Zone-Based Guidance Simulation
% RDPG Guidance Logic is inlined for vectorization.

    %% 0. GPU 초기화 및 설정
    if canUseGPU()
        g = gpuDevice;
        reset(g);
        fprintf('>>> 으헤~ GPU 가속 모드 가동! 사용 장치: %s <<<\n', g.Name);
    else
        error('GPU를 찾을 수 없습니다. Parallel Computing Toolbox를 설치하거나 하드웨어를 확인해주세요.');
    end

    % 상수 정의
    D2R = pi/180;
    R2D = 180/pi;
    
    % 시뮬레이션 파라미터 추출
    dt = cfg.dt_region;
    tf = cfg.tf;
    r_allow = cfg.r_allow;
    th_psi_deg = cfg.th_psi_deg;
    
    % Vehicle Spec
    V_p = cfg.V_p; 
    V_t = cfg.V_t; 
    At_cmd = cfg.At_cmd; % Target 가속도 (보통 0)
    Max_Acc_g = cfg.limit_acc; % 가속도 제한 (g단위)
    Max_Acc_mps2 = Max_Acc_g * 9.80665;
    
    % RDPG 파라미터 추출 (cfg 구조체에 있다고 가정, 없으면 기본값)
    % RDPG.m 파일 내용을 기반으로 K와 Sigma_des 설정
    if isfield(cfg, 'RDPG')
        K_gain = cfg.RDPG.K;
        Sigma_des_deg = cfg.RDPG.sigma_d;
    else
        % cfg에 없으면 기본값 (일반적인 값)
        K_gain = 3.0; 
        Sigma_des_deg = 0; % 기본적으로 0도 유지 (또는 sigma case에 따라 변동 가능)
    end
    Sigma_des_rad = Sigma_des_deg * D2R;

    %% 1. 초기 조건 벡터화 (Vectorization)
    % Sweep Range
    r_vec = (cfg.r_min_km:cfg.r_step_km:cfg.r_max_km) * 1000; 
    b_vec = (cfg.b_min_deg:cfg.b_step_deg:cfg.b_max_deg) * D2R; 
    
    % Meshgrid 생성 (모든 초기 조건 조합)
    [R_mat_cpu, B_mat_cpu] = meshgrid(r_vec, b_vec);
    
    % 데이터 크기
    [n_rows, n_cols] = size(R_mat_cpu);
    total_sims = numel(R_mat_cpu);
    
    % 분석할 초기 Look Angle (Sigma) 케이스
    sigma_cases_deg = cfg.sigma_cases_deg;
    
    % 데이터 저장용 변수
    collected_data = [];

    %% 2. 시각화 준비
    figure('Color', 'w', 'Position', [100 100 1200 800]);
    t_layout = tiledlayout('flow', 'TileSpacing', 'compact');
    title(t_layout, 'GPU Accelerated Reachability Analysis (RDPG)', 'FontSize', 15);

    %% 3. Sigma Case 별 루프 (Active Loop)
    for s_idx = 1:length(sigma_cases_deg)
        current_sigma0_deg = sigma_cases_deg(s_idx);
        current_sigma0_rad = current_sigma0_deg * D2R;
        
        fprintf('Simulating Sigma0 = %d deg... (%d/%d)\n', current_sigma0_deg, s_idx, length(sigma_cases_deg));
        
        % ------------------------------------------------
        % [Step A] 데이터 GPU 전송 및 초기화
        % ------------------------------------------------
        R0 = gpuArray(R_mat_cpu);
        B0 = gpuArray(B_mat_cpu);
        
        % Target 초기 상태 (원점, 북쪽 90도 방향 가정)
        Xt = gpuArray.zeros(n_rows, n_cols);
        Yt = gpuArray.zeros(n_rows, n_cols);
        Psi_t = gpuArray.zeros(n_rows, n_cols) + (90 * D2R); % Target Heading (North)
        
        % Pursuer 초기 상태 계산 (기하학적 관계)
        % Beta는 Target 속도 벡터 기준 상대 방위각
        % Global Angle = Psi_t - Beta
        Theta_global = Psi_t - B0;
        
        Xp = Xt + R0 .* cos(Theta_global);
        Yp = Yt + R0 .* sin(Theta_global);
        
        % 초기 시선각(Lambda) 및 헤딩(Psi_p) 계산
        Dx = Xt - Xp;
        Dy = Yt - Yp;
        Lambda = atan2(Dy, Dx);
        
        % 초기 Look Angle 조건 적용: Sigma = Psi_p - Lambda
        % -> Psi_p = Lambda + Sigma0
        Psi_p = Lambda + current_sigma0_rad;
        
        % 초기값 저장 (결과 플롯용)
        Init_Psi_p = Psi_p; 
        
        % 시뮬레이션 루프 변수
        Time = 0;
        Active_Mask = true(n_rows, n_cols, 'gpuArray'); % 진행 중인 시뮬레이션
        Success_Mask = false(n_rows, n_cols, 'gpuArray'); % 성공 여부
        Prev_R = R0; % CPA 감지용
        
        % ------------------------------------------------
        % [Step B] Time Integration Loop (GPU)
        % ------------------------------------------------
        while Time < tf
            % 1. 기하학 정보 업데이트
            Dx = Xt - Xp;
            Dy = Yt - Yp;
            R_dist = sqrt(Dx.^2 + Dy.^2);
            Lambda = atan2(Dy, Dx);
            
            % Lead Angles
            % 각도 차이를 -pi ~ pi 로 정규화하는 것이 중요 (GPU 호환 수동 구현)
            Sigma_p = mod(Psi_p - Lambda + pi, 2*pi) - pi;
            Sigma_t = mod(Psi_t - Lambda + pi, 2*pi) - pi;
            
            % 상대 속도 및 회전율 계산
            % Vc = -R_dot
            R_dot = V_t * cos(Sigma_t) - V_p * cos(Sigma_p);
            Lambda_dot = (V_t * sin(Sigma_t) - V_p * sin(Sigma_p)) ./ R_dist;
            
            % 2. [Guidance Logic - RDPG Inline]
            % RDPG.m 로직: a_cmd = Vp * (lambda_dot + K * (Sigma_p - Sigma_des))
            % 주의: RDPG.m 파일의 정확한 수식에 따라 부호가 다를 수 있음.
            % 통상적으로 Look Angle Error를 줄이는 방향.
            
            % 목표 각도 (Sigma_des) 설정 (고정값 혹은 함수)
            % 만약 RDPG가 가변 목표각을 쓴다면 여기에 구현 필요. 
            % 여기서는 고정값(Sigma_des_rad) 사용.
            
            Sigma_err = Sigma_p - Sigma_des_rad;
            Desired_Rate = -K_gain * Sigma_err; % 오차를 줄이는 방향
            
            % 가속도 명령 생성 (횡가속도 = V * 각속도)
            % Pursuer의 경로각 변화율 = Lambda_dot + Desired_Rate (기하학적 유도)
            % 혹은 단순히 DPG 수식 적용
            Acc_cmd = V_p * (Lambda_dot - Desired_Rate); 
            
            % 3. 가속도 제한 (Saturation)
            Acc_cmd = max(min(Acc_cmd, Max_Acc_mps2), -Max_Acc_mps2);
            
            % 4. 상태 적분 (Euler Method)
            % 가속도 -> 헤딩 변화 -> 위치 변화
            Psi_p = Psi_p + (Acc_cmd / V_p) * dt;
            Psi_t = Psi_t + (At_cmd / V_t) * dt; % Target은 보통 등속
            
            Xp = Xp + V_p * cos(Psi_p) * dt;
            Yp = Yp + V_p * sin(Psi_p) * dt;
            
            Xt = Xt + V_t * cos(Psi_t) * dt;
            Yt = Yt + V_t * sin(Psi_t) * dt;
            
            % 5. 종료 조건 체크
            % Condition A: 랑데부 성공 (거리 만족 & 헤딩 정렬)
            % 헤딩 오차 계산
            Heading_Err_Deg = abs(mod(Psi_p - Psi_t + pi, 2*pi) - pi) * R2D;
            Is_Captured = (R_dist <= r_allow) & (Heading_Err_Deg <= th_psi_deg);
            
            % Condition B: 실패 (CPA 지남 & 거리가 충분히 큼)
            Is_Missed = (R_dist > Prev_R) & (R_dist > r_allow * 2);
            
            % 현재 단계에서 종료된 것들
            Finished_Now = Active_Mask & (Is_Captured | Is_Missed);
            
            % 마스크 업데이트
            Success_Mask(Finished_Now & Is_Captured) = true;
            Active_Mask(Finished_Now) = false;
            
            % 루프 탈출 조건 (모든 시뮬레이션 종료 시)
            if ~any(Active_Mask, 'all')
                break;
            end
            
            % 변수 업데이트
            Prev_R = R_dist;
            Time = Time + dt;
        end
        
        % ------------------------------------------------
        % [Step C] 결과 수집 (GPU -> CPU)
        % ------------------------------------------------
        Success_Mask_CPU = gather(Success_Mask);
        R0_CPU = gather(R0);
        B0_CPU = gather(B0);
        Init_Psi_p_CPU = gather(Init_Psi_p);
        
        % 성공한 케이스의 좌표 계산 (플롯용)
        % 초기 위치 재계산 (CPU에서 Plotting)
        Theta_global_cpu = (90*D2R) - B0_CPU;
        Xp_start_all = 0 + R0_CPU .* cos(Theta_global_cpu);
        Yp_start_all = 0 + R0_CPU .* sin(Theta_global_cpu);
        
        Suc_X = Xp_start_all(Success_Mask_CPU);
        Suc_Y = Yp_start_all(Success_Mask_CPU);
        Fail_X = Xp_start_all(~Success_Mask_CPU);
        Fail_Y = Yp_start_all(~Success_Mask_CPU);
        
        % 데이터 저장 구조체 생성
        if any(Success_Mask_CPU(:))
            idx_suc = find(Success_Mask_CPU);
            temp_data = [R0_CPU(idx_suc), Init_Psi_p_CPU(idx_suc)*R2D, B0_CPU(idx_suc)*R2D, repmat(current_sigma0_deg, length(idx_suc), 1)];
            collected_data = [collected_data; temp_data];
        end
        
        % ------------------------------------------------
        % [Step D] 플롯 그리기
        % ------------------------------------------------
        next_ax = nexttile(t_layout);
        hold(next_ax, 'on'); grid(next_ax, 'on'); axis(next_ax, 'equal');
        
        % 실패 점 (빨강)
        if ~isempty(Fail_X)
            plot(next_ax, Fail_X, Fail_Y, '.', 'Color', [1 0.7 0.7], 'MarkerSize', 4);
        end
        % 성공 점 (파랑/청록)
        if ~isempty(Suc_X)
            plot(next_ax, Suc_X, Suc_Y, '.', 'Color', [0 0.8 1], 'MarkerSize', 5);
        end
        % Target 위치
        plot(next_ax, 0, 0, '^', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
        
        xlabel(next_ax, 'X [m]'); ylabel(next_ax, 'Y [m]');
        title(next_ax, sprintf('\\sigma_0 = %d^\\circ', current_sigma0_deg));
        xlim(next_ax, [-cfg.r_max_km*1000, cfg.r_max_km*1000]/2); % 적절히 조절
        
        drawnow;
    end
    
    %% 4. 데이터 저장 (CSV)
    if ~isempty(collected_data)
        T = array2table(collected_data, 'VariableNames', {'R0_m', 'Psi_p0_deg', 'Beta0_deg', 'Sigma0_deg'});
        writetable(T, 'Simulation_Results_GPU.csv');
        fprintf('>>> 결과 저장 완료: Simulation_Results_GPU.csv <<<\n');
    else
        fprintf('>>> 성공한 케이스가 없습니다. <<<\n');
    end

end