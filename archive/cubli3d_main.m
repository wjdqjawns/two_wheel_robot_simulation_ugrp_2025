function cubli3d_main()
    % 3D Cubli (reaction-wheel inverted pendulum) with backstepping control
    % Upright/corner balance about a ground pivot at a cube vertex.
    %
    % Dynamics & control based on ETH Cubli papers:
    % - ECC'13 system dynamics and identification:contentReference[oaicite:3]{index=3}
    % - CDC'13 / extended article nonlinear backstepping control:contentReference[oaicite:4]{index=4}:contentReference[oaicite:5]{index=5}
    
    %%% ---------- Physical parameters ----------
    P.L   = 0.15;          % cube edge [m]
    P.m   = 0.419;         % mass [kg] (edit to your build)
    P.g0  = 9.81;          % gravity [m/s^2]
    
    % body inertia about COM for a solid cube: (1/6)mL^2 I
    Icom  = (1/6)*P.m*P.L^2*eye(3);
    
    % position from pivot (corner) to COM in BODY frame (constant):
    rc    = 0.5*P.L*[1;1;1];          % [m]
    % parallel-axis to corner: J = Icom + m( (r·r)I - r r^T )
    P.Theta0 = Icom + P.m*((rc.'*rc)*eye(3) - (rc*rc.'));  % body inertia about pivot
    
    % wheel inertias about their axes (referenced at pivot; simple diag model)
    Iw = 0.57e-3;                      % [kg m^2] per wheel (paper's order of magnitude)
    P.ThetaW = diag([Iw Iw Iw]);
    
    % "m" vector in papers is mass*position-to-CoM in body frame【16】
    P.mvec = P.m*rc;                   % [kg·m]
    
    %%% ---------- Controller gains (backstepping) ----------
    % Gains correspond to Thm 3.1 in the 3D paper【16】
    P.alpha = 20;
    P.beta  =  8;
    P.gamma =  6;
    P.delta =  1;
    
    %%% ---------- Simulation setup ----------
    Tend = 8;        % seconds
    dt_vis = 0.02;
    
    % Initial pose: cube lying with a face nearly down; small yaw.
    % R0 = I means body axes aligned to world; gravity is -z in world.
    R0 = eul2rotm([deg2rad(0) deg2rad(0) deg2rad(0)],'ZYX');
    
    % Initial angular momenta (body & wheels)
    p_h0 = zeros(3,1);
    p_w0 = zeros(3,1);
    
    % State vector: [vec(R); p_h; p_w] where vec(R) stacks column-wise (9x1)
    x0 = [reshape(R0,9,1) ; p_h0 ; p_w0];
    
    % Integrate closed-loop dynamics
    opts = odeset('RelTol',1e-7,'AbsTol',1e-8);
    [t, X] = ode45(@(t,x) f_cl(t,x,P), [0 Tend], x0, opts);
    
    %%% ---------- Visualization ----------
    figure('Name','Cubli 3D Corner Balance','Color','w');
    ax = gca; view(45,25); grid on; axis equal
    xlabel X; ylabel Y; zlabel Z; hold on
    L = P.L;
    axis([-0.25 0.25 -0.25 0.25 0 0.35]);
    
    % for k = 1:round(dt_vis/mean(diff(t))):length(t)
    %     cla(ax)
    %     R = reshape(X(k,1:9),3,3);
    % 
    %     % Draw cube with the ground pivot at origin, opposite vertex along +z
    %     draw_cube_at_pivot(R, L, ax);
    % 
    %     % ground
    %     patch(ax,'XData',[-.5 .5 .5 -.5],'YData',[-.5 -.5 .5 .5], ...
    %         'ZData',[0 0 0 0],'FaceColor',[0.95 0.95 0.95],'EdgeColor','none','FaceAlpha',1);
    %     plot3(ax,0,0,0,'ro','MarkerSize',6,'MarkerFaceColor','r');
    %     title(ax, sprintf('Time = %.2f s', t(k)));
    %     drawnow
    % end
    filename = 'cubli3d_balance.gif';  % 저장 파일 이름

    for k = 1:round(dt_vis/mean(diff(t))):length(t)
        cla(ax)
        R = reshape(X(k,1:9),3,3);
    
        draw_cube_at_pivot(R, L, ax);
        patch(ax,'XData',[-.5 .5 .5 -.5],'YData',[-.5 -.5 .5 .5], ...
            'ZData',[0 0 0 0],'FaceColor',[0.95 0.95 0.95],'EdgeColor','none','FaceAlpha',1);
        plot3(ax,0,0,0,'ro','MarkerSize',6,'MarkerFaceColor','r');
        title(ax,sprintf('Time = %.2f s',t(k)));
        drawnow;
    
        % --- GIF 캡처 ---
        frame = getframe(gcf);
        [A,map] = rgb2ind(frame2im(frame),256);
        if k==1
            imwrite(A,map,filename,"gif","LoopCount",inf,"DelayTime",dt_vis);
        else
            imwrite(A,map,filename,"gif","WriteMode","append","DelayTime",dt_vis);
        end
    end
    %%% ---------- Post-processing & Plots ----------
    omega_h_hist = zeros(length(t),3);
    T_hist = zeros(length(t),3);
    angles_hist = zeros(length(t),3);
    
    for i = 1:length(t)
        R = reshape(X(i,1:9),3,3);
        p_h = X(i,10:12)';
        p_w = X(i,13:15)';

        omega_h_hist(i,:) = (P.Theta0 \ (p_h - p_w))';
        eul = rotm2eul(R,'ZYX');      % [yaw pitch roll]
        angles_hist(i,:) = rad2deg(eul);
        gB = R.'*[0;0;-P.g0];
        tau_g = cross(P.mvec,gB);
        omega_h = (P.Theta0 \ (p_h - p_w));
        Pg = eye(3) - (gB*gB.')/(P.g0^2);
        p_perp = Pg*p_h;
        K1 = eye(3) + (P.alpha + P.beta*P.gamma + P.delta)*P.Theta0;
        K2 = P.Theta0*(P.alpha*hat(p_perp)+P.beta*hat(P.mvec)*hat(gB)) + hat(p_h);
        K3 = P.gamma*(eye(3)+P.alpha*P.Theta0*Pg);
        K4 = P.gamma*eye(3);
        T_hist(i,:) = (K1*tau_g + K2*omega_h + K3*p_h - K4*p_w)';
    end
    
    figure('Name','Cubli Simulation Data','Color','w');
    
    subplot(3,1,1);
    plot(t,angles_hist);
    ylabel('Euler angles [deg]');
    legend('yaw','pitch','roll');
    grid on;
    
    subplot(3,1,2);
    plot(t,omega_h_hist);
    ylabel('\omega_h [rad/s]');
    legend('\omega_x','\omega_y','\omega_z');
    grid on;
    
    subplot(3,1,3);
    plot(t,T_hist);
    ylabel('T_{wheel} [Nm]');
    xlabel('Time [s]');
    legend('T_x','T_y','T_z');
    grid on;
    
    % Save data for report
    simdata.t = t;
    simdata.euler_deg = angles_hist;
    simdata.omega_h = omega_h_hist;
    simdata.T = T_hist;
    save('cubli3d_results.mat','simdata');
end

%%% ========== Closed-loop dynamics ==========
function dx = f_cl(~, x, P)
    % Unpack state
    R    = reshape(x(1:9),3,3);
    p_h  = x(10:12);
    p_w  = x(13:15);
    
    % Orthonormalize (numerically keep R in SO(3)) via polar decomposition
    [U,~,V] = svd(R); R = U*V.';
    
    % World gravity (in inertial frame)
    gI = [0;0;-P.g0];
    % Body-expressed gravity
    gB = R.'*gI;
    
    % Current angular velocity of housing (Theta0*(ωh) = p_h - p_w)
    omega_h = P.Theta0 \ (p_h - p_w);
    
    % Wheel angular velocity (not used explicitly for control)
    % omega_w = (P.ThetaW \ p_w) - omega_h;
    
    % Gravity torque in BODY frame: tau_g = m × g (m is mass*rc in body frame)
    tau_g = cross(P.mvec, gB);
    
    % ---------- Backstepping control law (3D) ----------
    % g⊥-projection for p_perp
    Pg   = eye(3) - (gB*gB.')/(P.g0^2);
    p_perp = Pg * p_h;
    
    K1 = eye(3) + (P.alpha + P.beta*P.gamma + P.delta)*P.Theta0;
    K2 = P.Theta0*( P.alpha*hat(p_perp) + P.beta*hat(P.mvec)*hat(gB) ) + hat(p_h);
    K3 = P.gamma*( eye(3) + P.alpha*P.Theta0*Pg );
    K4 = P.gamma*eye(3);
    
    T = K1*tau_g + K2*omega_h + K3*p_h - K4*p_w;  % control torque to wheels
    % (Same structure as Eq. (10) in CDC'13 / extended article):contentReference[oaicite:6]{index=6}:contentReference[oaicite:7]{index=7}
    
    % ---------- Reduced rigid-body dynamics (body frame) ----------
    % ṗ_h = -ωh×p_h + τ_g
    p_h_dot = -cross(omega_h, p_h) + tau_g;
    % ṗ_w = T
    p_w_dot = T;
    % Ṙ = R * ω̂h
    R_dot   = R * hat(omega_h);
    
    % Pack derivative
    dx = [reshape(R_dot,9,1); p_h_dot; p_w_dot];
end

%%% ========== Helpers ==========
function A = hat(v)
    % Skew-symmetric matrix (hat operator)
    A = [  0   -v(3)  v(2);
         v(3)   0   -v(1);
        -v(2)  v(1)   0  ];
end

function draw_cube_at_pivot(R, L, ax)
    % Cube vertices in body coordinates (centered at origin first)
    V = (L/2)*[-1 -1 -1;
                1 -1 -1;
                1  1 -1;
               -1  1 -1;
               -1 -1  1;
                1 -1  1;
                1  1  1;
               -1  1  1]';
    
    % Rotate to world
    VW = R*V;
    
    % Choose the lowest vertex (closest to ground) as the ground pivot
    [~,idx] = min(VW(3,:));
    pivot_w = VW(:,idx);
    
    % Shift so that pivot is exactly at (0,0,0)
    VW = VW - pivot_w;
    
    F = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];
    patch(ax,'Vertices',VW','Faces',F,'FaceColor',[0.60 0.72 0.90], ...
          'FaceAlpha',0.85,'EdgeColor','k','LineWidth',1.0);
end