function main_cubli()
    %% 1. 파라미터 설정
    P.m = 0.419;                % 큐브 질량 [kg]
    P.L = 0.15;                 % 한 변 길이 [m]
    P.r = P.L / sqrt(3);        % 중심에서 꼭짓점까지 거리
    P.Ix = 2.6e-3;              % 본체 관성 모멘트 [kg·m^2]
    P.Iy = 2.6e-3;
    P.Iz = 2.6e-3;
    P.Iw = 0.57e-3;             % 반응휠 관성 [kg·m^2]
    P.g = 9.81;                 % 중력 [m/s^2]

    %% 2. 상태공간 모델 (소각도 근사)
    % 상태: [phi, theta, psi, phidot, thetadot, psidot]'
    A = [zeros(3), eye(3);
         zeros(3), zeros(3)];
    B = [zeros(3);
         eye(3) / P.Ix];  % 단순화 (축별 동일 관성 가정)

    % LQR 제어기 설계
    Q = diag([80 80 30 2 2 2]);   % 상태 가중
    R = diag([0.05 0.05 0.05]);   % 입력 가중
    K = lqr(A, B, Q, R);

    % %% 3. 초기 상태 (corner에서 약간 기울어진 상태)
    % x0 = deg2rad([0; 0; 0; 0; 0; 0]);  % [phi, theta, psi, phidot, thetadot, psidot]
    x0 = deg2rad([2; -3; 1; 0; 0; 0]);
    %% 초기 회전 (corner 기준으로 세워주기)
    phi0 = deg2rad(35.264);   % Y축 회전 (약 35.26도)
    theta0 = deg2rad(45);     % X축 회전 (45도)
    R0 = eul2rotm([0, phi0, theta0], 'ZYX');

    %% 4. 시뮬레이션
    tspan = [0 10];
    [t, X] = ode45(@(t,x) cubli_dynamics(t, x, -K*x, P), tspan, x0);

    %% 5. 시각화
    % figure('Name', 'Cubli Simulation', 'Color', 'w');
    % axis equal; grid on; hold on;
    % xlabel('X'); ylabel('Y'); zlabel('Z');
    % view(45, 25);
    % axis([-0.2 0.2 -0.2 0.2 0 0.3]);
    % 
    % % for k = 1:10:length(t)
    % %     cla;
    % %     R = eul2rotm(X(k,1:3), 'ZYX');
    % %     draw_cubli(R, P.L);
    % %     title(sprintf('Time = %.2f s', t(k)));
    % %     pause(0.03);
    % % end
    % for k = 1:10:length(t)
    %     cla;
    %     R = R0 * eul2rotm(X(k,1:3), 'ZYX');
    %     draw_cubli(R, P.L);
    %     title(sprintf('Time = %.2f s', t(k)));
    %     pause(0.03);
    % end
    figure('Name', 'Diagonal Cubli', 'Color', 'w');
    hold on; view(45,25);
    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis([-0.2 0.2 -0.2 0.2 0 0.3]);
    
    draw_cubli(0.15);

end

%% ---------------- Dynamics Function ----------------
% function dx = cubli_dynamics(~, x, u, P)
%     phi = x(1); theta = x(2); psi = x(3);
%     phidot = x(4); thetadot = x(5); psidot = x(6);
%     omega = [phidot; thetadot; psidot];
% 
%     % 단순화된 회전 동역학 (small angle)
%     domega = (1/P.Ix) * (u - [P.m*P.g*P.r*phi;
%                               P.m*P.g*P.r*theta;
%                               P.m*P.g*P.r*psi]);
% 
%     dx = [omega; domega];
% end
function dx = cubli_dynamics(~, x, u, P)
    phi = x(1); theta = x(2); psi = x(3);
    phidot = x(4); thetadot = x(5); psidot = x(6);
    omega = [phidot; thetadot; psidot];

    % 중력 모멘트 (비선형)
    Tgrav = P.m * P.g * P.r * [sin(phi);
                               sin(theta);
                               sin(psi)];

    % 동역학
    % domega = (1/P.Ix) * (u - Tgrav);
    domega = (1/P.Ix) * (u - [P.m*P.g*P.r*phi;
                          P.m*P.g*P.r*theta;
                          P.m*P.g*P.r*psi]);
    dx = [omega; domega];
end

%% ---------------- Visualization Function ----------------
% function draw_cubli(R, L)
%     % 정육면체 꼭짓점 정의
%     v = L/2 * [-1 -1 -1;
%                 1 -1 -1;
%                 1  1 -1;
%                -1  1 -1;
%                -1 -1  1;
%                 1 -1  1;
%                 1  1  1;
%                -1  1  1]';
% 
%     vR = R * v;  % 회전 적용
%     f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
% 
%     patch('Vertices', vR', 'Faces', f, ...
%           'FaceColor', [0.6 0.7 0.9], 'FaceAlpha', 0.8, ...
%           'EdgeColor', 'k', 'LineWidth', 1.0);
%     plot3(0,0,0,'ro','MarkerSize',6,'MarkerFaceColor','r'); % pivot 표시
% end
% function draw_cubli(R, L)
%     % 꼭짓점 기준으로 큐브를 그리는 함수
% 
%     % 기본 정육면체 (중심 기준)
%     v = L/2 * [-1 -1 -1;
%                 1 -1 -1;
%                 1  1 -1;
%                -1  1 -1;
%                -1 -1  1;
%                 1 -1  1;
%                 1  1  1;
%                -1  1  1]';
% 
%     % --- 회전 적용 ---
%     vR = R * v;
% 
%     % --- 꼭짓점 기준 translation 적용 ---
%     % (−1, −1, −1) 꼭짓점이 원점에 오도록 전체 평행이동
%     corner_offset = vR(:,1);  % 첫 번째 꼭짓점 좌표 (pivot)
%     vR_shifted = vR - corner_offset;
% 
%     % 면 정의
%     f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
% 
%     % --- 큐브 그리기 ---
%     patch('Vertices', vR_shifted', 'Faces', f, ...
%           'FaceColor', [0.6 0.7 0.9], 'FaceAlpha', 0.8, ...
%           'EdgeColor', 'k', 'LineWidth', 1.0);
% 
%     % pivot 표시
%     plot3(0,0,0,'ro','MarkerSize',6,'MarkerFaceColor','r');
% end
function draw_cubli(L)
    % L: 한 변 길이 [m]
    % 한 꼭짓점이 원점에 있고, 반대 꼭짓점이 (0,0,sqrt(3)*L)에 오도록 배치

    % 정육면체의 8개 꼭짓점 (중심 기준)
    v = L/2 * [-1 -1 -1;
                1 -1 -1;
                1  1 -1;
               -1  1 -1;
               -1 -1  1;
                1 -1  1;
                1  1  1;
               -1  1  1]';

    % --- 회전행렬: 대각선을 z축으로 맞추기 ---
    % 공간 대각선 방향 [1;1;1] → z축 [0;0;1]
    R_align = rotationBetweenVectors([1;1;1], [0;0;1]);

    % --- 회전 및 이동 ---
    vR = R_align * v;

    % 첫 번째 꼭짓점(최하단)을 원점으로 평행이동
    min_z = min(vR(3,:));
    idx_bottom = find(vR(3,:) == min_z, 1);
    offset = vR(:, idx_bottom);
    vR_shifted = vR - offset;

    % --- 면 정의 ---
    f = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

    % --- 큐브 그리기 ---
    patch('Vertices', vR_shifted', 'Faces', f, ...
          'FaceColor', [0.6 0.7 0.9], 'FaceAlpha', 0.8, ...
          'EdgeColor', 'k', 'LineWidth', 1.0);

    % pivot 표시
    plot3(0,0,0,'ro','MarkerSize',6,'MarkerFaceColor','r');

    % 축 표시
    axis equal; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
end

% === 벡터 회전행렬 계산 함수 ===
function R = rotationBetweenVectors(a, b)
    a = a / norm(a);
    b = b / norm(b);
    v = cross(a,b);
    s = norm(v);
    c = dot(a,b);
    if s == 0
        R = eye(3);
        return;
    end
    vx = [  0   -v(3)  v(2);
           v(3)   0   -v(1);
          -v(2)  v(1)   0 ];
    R = eye(3) + vx + vx^2 * ((1 - c)/(s^2));
end
