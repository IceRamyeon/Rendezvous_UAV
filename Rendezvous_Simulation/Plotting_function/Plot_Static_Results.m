function Plot_Static_Results(cfg, sim_out)
    R2D = 180/pi;
    time = sim_out.time;
    hist_state = sim_out.hist_state;
    
    r_data = hist_state(1,:);
    lambda_data = hist_state(2,:) * R2D;
    psi_p_data = hist_state(3,:) * R2D; 
    psi_t_data = hist_state(4,:) * R2D;
    sigma_p_data = hist_state(5,:) * R2D;
    sigma_t_data = hist_state(6,:) * R2D;
    Vc_data = hist_state(7,:);
    acc_p_data = hist_state(8,:);

    % mode_flag 및 sigma_ref 데이터 추출 (배열 크기가 10 이상일 때만)
    has_extra_data = size(hist_state, 1) >= 10;
    if has_extra_data
        sigma_ref_data = hist_state(9,:) * R2D;
        mode_flag_data = hist_state(10,:);
    end

    % =========================================================
    % Figure 6
    % =========================================================
    fig6 = figure(6); set(fig6, 'Position', [10, 50, 500, 750], 'Theme', 'light');
    subplot(4,1,1); plot(time, r_data, 'r-', 'LineWidth', 2); grid on; title('[Fig 6-1] Relative Distance');
    subplot(4,1,2); plot(time, -Vc_data, 'b-', 'LineWidth', 2); grid on; title('[Fig 6-2] Closing Velocity');
    subplot(4,1,3); plot(time, acc_p_data, 'r-', 'LineWidth', 2); grid on; title('[Fig 6-3] Pursuer Acc');
    subplot(4,1,4); plot(r_data, acc_p_data, 'b-', 'LineWidth', 2); grid on; set(gca, 'XDir', 'reverse'); title('[Fig 6-4] Acc vs Range');

    % =========================================================
    % Figure 7
    % =========================================================
    fig7 = figure(7); set(fig7, 'Position', [520, 50, 500, 900], 'Theme', 'light');
    subplot(4,1,1); plot(time, lambda_data, 'm-', 'LineWidth', 2); grid on; title('[Fig 7-1] LOS Angle');
    
    % Lead Angles 플롯에 sigma_ref 추가
    subplot(4,1,2); hold on; grid on;
    plot(time, sigma_p_data, 'b-', 'LineWidth', 1.5, 'DisplayName', '\sigma_p');
    plot(time, sigma_t_data, 'r--', 'LineWidth', 1.5, 'DisplayName', '\sigma_t');
    if has_extra_data
        plot(time, sigma_ref_data, 'g-.', 'LineWidth', 1.5, 'DisplayName', '\sigma_{ref}');
    end
    title('[Fig 7-2] Lead Angles'); 
    legend('Location', 'best'); 
    hold off;

    subplot(4,1,3); plot(time, psi_p_data, 'b-', time, psi_t_data, 'r-', 'LineWidth', 2); grid on; title('[Fig 7-3] Heading Angles');
    
    % [수정] Control Energy 대신 Guidance Mode Flag 플롯으로 교체
    subplot(4,1,4); hold on; grid on;
    if has_extra_data
        plot(time, mode_flag_data, 'k-', 'LineWidth', 2);
        yticks([-1 0 1 2]);
        ylim([-1.5, 2.5]);
        title('[Fig 7-4] Guidance Mode Flag (0:Safe, 1:2m, 2:r_{f,min})');
    else
        title('[Fig 7-4] Guidance Mode Flag (No Data)');
    end
    hold off;

    if strcmp(cfg.GUIDANCE_MODE, 'RDPG_MIN')
        fig8 = figure(8); set(fig8, 'Position', [1030, 50, 500, 160], 'Theme', 'light');plot(time, hist_state(17, :), 'r-', 'LineWidth', 2); grid on; 
        title('[Fig 8-1] r_{f} History');
    end

    if strcmp(cfg.GUIDANCE_MODE, 'RDPG_FRS') && isfield(sim_out, 'point_value_hist')
        fig8 = figure(8); set(fig8, 'Position', [1030, 50, 500, 400], 'Theme', 'light');
        hold on; grid on;
        title('[Fig 8] FRS Distance History (\sigma_{pc} candidates)');
        xlabel('Time (s)');
        ylabel('Distance to Boundary');
        
        num_steps = length(time);
        num_angles = 0;
        % 최대 각도 개수 파악
        for k = 1:num_steps
            if ~isempty(sim_out.point_value_hist{k})
                num_angles = size(sim_out.point_value_hist{k}, 1);
                break;
            end
        end
        
        if num_angles > 0
            dist_hist = nan(num_angles, num_steps);
            flag_hist = nan(num_angles, num_steps);
            
            % 스텝별로 거리(1열)와 채택 여부(2열) 추출
            for i = 1:num_steps
                pts = sim_out.point_value_hist{i};
                if ~isempty(pts) && size(pts, 2) >= 2
                    dist_hist(:, i) = pts(:, 1);
                    flag_hist(:, i) = pts(:, 2);
                end
            end
            
            % 각도별로 선 색상을 다르게 지정
            color_map = lines(num_angles); 
            
            for j = 1:num_angles
                % 1. 전체 구간을 얇은 선(두께 1)으로 그리기
                plot(time, dist_hist(j, :), 'Color', color_map(j,:), 'LineWidth', 1.0);
                
                % 2. 채택된 구간(값이 1)만 두꺼운 선(두께 3)으로 덧그리기
                thick_line = dist_hist(j, :);
                selected_idx = (flag_hist(j, :) == 1);
                thick_line(~selected_idx) = nan; % 채택 안 된 곳은 투명 처리
                
                plot(time, thick_line, 'Color', color_map(j,:), 'LineWidth', 3.0);
            end
        end
        hold off;
    end

    % 이미지 저장 로직
    if cfg.auto_save == 1
        for idx = 1:8
            if ishandle(idx)
                filename = sprintf('Fig%02d_Result.png', idx);
                print(figure(idx), fullfile(cfg.save_dir, filename), '-dpng', '-r150'); 
            end
        end
    end
end