classdef draw_2d_uav < handle
    properties
        m
        J_x
        J_y
        J_z
        J_xz

        spmd
        b
        c
        rho
        e

        V_max
        D_prop
        K_V
        K_Q
        R_motor
        i_o
        C_Q2
        C_Q1
        C_Q0
        C_T2
        C_T1
        C_T0
        end
    end
    
    methods
        % 2D UAV 시뮬레이션
        figure(1); clf; hold on;
        axis([-5 5 -5 5]); grid on;
        xlabel('North (x)'); ylabel('East (y)');
        title('2D Top-Down UAV');

        % 1. 기체의 위치(평행이동)와 방향(회전) 설정
        pn = 1;      % 북쪽(x) 위치
        pe = 2;      % 동쪽(y) 위치
        psi = pi/4;  % 요(Yaw) 각도 (45도 회전)

        % 2. UAV의 꼭짓점(Vertices) 정의 [x; y] 
        % 기체 정면이 x축, 오른쪽이 y축 기준이야 
        nose = [2; 0];         % 기수 (앞)
        right_wing = [-1; 2];  % 오른쪽 날개
        tail = [-0.5; 0];      % 꼬리
        left_wing = [-1; -2];  % 왼쪽 날개

        V_body = [nose, right_wing, tail, left_wing];

        % 3. 2D 회전 행렬 (Yaw) 
        % 문서의 3D R_yaw 행렬에서 z축 부분을 뺀 2D 버전이야[cite: 209, 210, 211, 212, 213].
        R_yaw = [cos(psi), -sin(psi);
                sin(psi),  cos(psi)];

        % 4. 꼭짓점 회전 및 평행이동 
        V_inertial = R_yaw * V_body; % 회전 
        V_inertial(1,:) = V_inertial(1,:) + pn; % 평행이동 
        V_inertial(2,:) = V_inertial(2,:) + pe;

        % 5. patch 함수로 면 그리기 
        patch(V_inertial(1,:), V_inertial(2,:), 'cyan', 'EdgeColor', 'blue', 'LineWidth', 1.5);

        % 기체가 어딜 보고 있는지 알기 쉽게 코끝에 빨간 점 하나 찍어줄게~
        plot(V_inertial(1,1), V_inertial(2,1), 'ro', 'MarkerFaceColor', 'r');

    end
end