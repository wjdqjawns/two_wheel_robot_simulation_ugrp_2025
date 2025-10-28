classdef LQRController
    properties
        K
        Q
        R
    end

    methods
        function obj = LQRController(params)
            obj.Q = params.Q;
            obj.R = params.R;
            obj.K = [];
        end

        function computeGain(obj, A, B)
            if size(A,1)~=4 || size(A,2)~=4 || size(B,1)~=4 || size(B,2)~=1
                error('LQRController:BadAB','A must be 4x4 and B must be 4x1.');
            end
            if isempty(obj.Q) || isempty(obj.R)
                error('LQRController:QR','Q/R not set.');
            end
            
            disp('A matrix:'); disp(A);
            disp('B matrix:'); disp(B);
            ctrb_rank = rank(ctrb(A,B));
            disp(['Controllability rank = ', num2str(ctrb_rank)]);

            obj.K = lqr(A, B, obj.Q, obj.R);
        end

        function u = compute(obj, x)
            if isempty(obj.K)
                error('LQR gain K has not been computed yet. Call computeGain(A,B) first.');
            end
            u = -obj.K * x; % 상태 피드백 제어
        end
    end
end