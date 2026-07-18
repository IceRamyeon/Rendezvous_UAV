function [sigma_p0_deg, sigma_p0_rad] = RDPG_LeadAngle(input_a, input_b, r_allow)
%% RDPG_LeadAngle.m
% 주어진 초기 기하학 조건으로부터 Reachable한 초기 Pursuer 리드각을 역산해주는 함수(Reachability-based rendzvous(2022))

    bearing_rad = input_b * pi / 180;
    sigma_t0 = bearing_rad - pi; 
    sigma_t0 = atan2(sin(sigma_t0), cos(sigma_t0));

    % 식 (5)를 만족하는 포물선 궤적 해 탐색 방정식
    eqn = @(x) sqrt(input_a / r_allow) * cos((sigma_t0 + x)/2) - cos(x);
    
    try
        sigma_p0_rad = fzero(eqn, 100 * pi / 180); 
        sigma_p0_deg = sigma_p0_rad * 180 / pi;
    catch
        disp('Reachable한 초기 해를 찾지 못함. sigma_pc = 0으로 임시 설정.');
        sigma_p0_deg = 0; 
        sigma_p0_rad = 0;
    end   
end