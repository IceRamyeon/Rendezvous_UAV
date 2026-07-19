function theory = Max_Point(sigma_p0_rad, r_allow, V_p)
%% Max_Point.m
% Theorem 1 수식의 LHS Grid Sweep을 통해 수치 해석적으로 최대 가속도 예측점을 찾는 함수.

    n_points = 50000;                         % sweep 점 개수
    epsilon = 1e-7;                           % 분모 발산 방지 조건
    sigma_t_grid = linspace(0, pi, n_points); % 0부터 pi까지 범위 생성

    % 논문 식 (11.1) 좌변(LHS) 직접 계산 수식
    y_lhs = (sin(sigma_t_grid) - sin(sigma_p0_rad)) .* (1 + cos(sigma_t_grid + sigma_p0_rad)) / (cos(sigma_p0_rad)^2 + epsilon);

    % 최댓값 지점 매칭 검색
    [y_max, max_idx] = max(y_lhs);
    sigma_t_star_rad = sigma_t_grid(max_idx);

    % 이론적 임계 상대거리(r*) 및 최대 요구 가속도(a*) 역산
    r_star = r_allow * (cos(sigma_p0_rad)^2) / (cos((sigma_t_star_rad + sigma_p0_rad)/2)^2);
    a_star = (y_max * V_p^2) / (2 * r_allow); 

    % 구조체 포장
    theory.sigma_t_star_deg = sigma_t_star_rad * 180 / pi;
    theory.r_star = r_star;
    theory.a_star = a_star;
end