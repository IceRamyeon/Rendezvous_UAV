function Save_Log_Data(save_dir, custom_filename, sim_out)
    % 디렉토리가 없으면 생성
    if ~exist(save_dir, 'dir')
        mkdir(save_dir);
    end
    
    % 시간 및 히스토리 데이터 추출
    time = sim_out.time;
    hist = sim_out.hist_state;
    
    % 8개 카테고리별 데이터 패키징 (Struct 활용)
    log_data = struct();
    log_data.Time = time;
    
    % 0. Trajectory (11~14행: Xp, Yp, Xt, Yt)
    log_data.Trajectory.Xp = hist(11, :);
    log_data.Trajectory.Yp = hist(12, :);
    log_data.Trajectory.Xt = hist(13, :);
    log_data.Trajectory.Yt = hist(14, :);
    
    % 1. Relative Distance (1행: r)
    log_data.Relative_Distance = hist(1, :);
    
    % 2. Closing Velocity (7행: V_c)
    log_data.Closing_Velocity = hist(7, :);
    
    % 3. Pursuer Acc (8행: acc_cmd)
    log_data.Pursuer_Acc = hist(8, :);
    
    % 4. Acc vs Range (가속도와 거리 묶음)
    log_data.Acc_vs_Range.r = hist(1, :);
    log_data.Acc_vs_Range.acc = hist(8, :);
    
    % 5. LOS Angle (2행: lambda)
    log_data.LOS_Angle = hist(2, :);
    
    % 6. Pursuer Lead Angle (5행: sigma_p)
    log_data.Pursuer_Lead_Angle = hist(5, :);
    
    % 7. Pursuer Heading Angle (3행: psi_p)
    log_data.Pursuer_Heading_Angle = hist(3, :);
    
    % mode_flag나 sigma_ref 같은 부가 정보도 덤으로 넣어두기
    log_data.Extra.mode_flag = hist(10, :);
    log_data.Extra.sigma_ref = hist(9, :);

    % 지정된 경로와 이름으로 .mat 파일 1개로 저장
    full_path = fullfile(save_dir, [custom_filename, '.mat']);
    save(full_path, 'log_data');
    
    fprintf('>>> 데이터 패키징 저장 완료: %s\n', full_path);
end