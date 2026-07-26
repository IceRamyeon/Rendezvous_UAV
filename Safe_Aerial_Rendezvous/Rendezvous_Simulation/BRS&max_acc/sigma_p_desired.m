% 으헤~ 주어진 변수들 입력
r_f = 2;
r = 800;
sigma_p = 62; % 단위는 degree로 가정

% 역코사인(acosd)을 이용해 sigma_t 계산 (degree 단위로 바로 계산됨)
term = cosd(sigma_p) / sqrt(r / r_f);
sigma_t = 2 * acosd(term) - sigma_p;

% pi - sigma_t 계산
% sigma_t가 degree 단위이므로, pi(라디안) 대신 180도를 빼주면 돼~
final_result = 180 - sigma_t;

% 결과 출력
fprintf('계산된 sigma_t: %.4f도\n', sigma_t);
fprintf('최종 결과 (pi - sigma_t): %.4f도\n', final_result);