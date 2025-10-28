classdef CubliModel2D
    properties
        mb; mw; lb; l; Ib; Iw; g;
        Cb; Cw; Km;
    end

    methods
        function obj = CubliModel2D(modelParam)
            obj.mb = modelParam.mb;
            obj.mw = modelParam.mw;
            obj.lb = modelParam.lb;
            obj.l  = modelParam.l;
            obj.Ib = modelParam.Ib;
            obj.Iw = modelParam.Iw;
            obj.g  = modelParam.g;
            obj.Cb = modelParam.Cb;
            obj.Cw = modelParam.Cw;
            obj.Km = modelParam.Km;
        end

        % ------------------------------
        % Linearized model (\theta_b≈\theta)
        % ------------------------------
        function [A,B] = getStateSpace(obj)
            Jb = obj.Ib + obj.mw * obj.l^2;
            Jw = obj.Iw;
            mgl = 0.1*(obj.mb*obj.lb + obj.mw*obj.l)*obj.g;

            A = [ 0 1 0 0;
                 mgl/Jb -obj.Cb/Jb 0  obj.Cw/Jb;
                  0 0 0 1;
                 -mgl/Jw obj.Cb/Jw 0 -obj.Cw/Jw];

            B = [0; -obj.Km/Jb; 0; obj.Km/Jw];
        end

        % ------------------------------
        % Nonlinear dynamics
        % ------------------------------
        function dx = dynamics(obj, x, u)
            theta_b = x(1); dtheta_b = x(2);
            theta_w = x(3); dtheta_w = x(4);

            % Input torque from motor
            Tm = obj.Km * u;

            % Constants
            A = (obj.mb*obj.lb + obj.mw*obj.l) * obj.g * sin(theta_b);
            denom = (obj.Ib + obj.mw * obj.l^2);

            % system dynamics
            ddtheta_b = (A - Tm - obj.Cb*dtheta_b + obj.Cw*dtheta_w) / denom;

            ddtheta_w = ((obj.Ib + obj.Iw + obj.mw*obj.l^2)*(Tm - obj.Cw*dtheta_w) ...
                        - (obj.Iw + obj.mw*obj.l^2)*(obj.Cb*dtheta_b + (obj.mb*obj.lb + obj.mw*obj.l)*obj.g*sin(theta_b))) ...
                        / (obj.Iw * (obj.Ib + obj.mw*obj.l^2));

            dx = [dtheta_b; ddtheta_b; dtheta_w; ddtheta_w];
        end
    end
end