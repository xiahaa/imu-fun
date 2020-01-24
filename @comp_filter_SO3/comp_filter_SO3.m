classdef comp_filter_SO3 < handle
    %% Public properties
    properties (Access = public)
        SamplePeriod = 1/256;
        R = eye(3); %% rotation matrix from ground to body
        kp = 1;
        ki = 0.1;
        bias = zeros(3,1);
    end
 
    %% Public methods
    methods (Access = public)
        function obj = comp_filter_SO3(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'R0'), obj.R = varargin{i+1};
                elseif  strcmp(varargin{i}, 'kp'), obj.kp = varargin{i+1};
                elseif  strcmp(varargin{i}, 'ki'), obj.ki = varargin{i+1};
                elseif strcmp(varargin{i}, 'bias'), obj.bias = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end

        function obj = Update(obj, gyro, acc, mag)
            if size(gyro,1) == 1
                gyro = gyro';
            end
            if size(acc,1) == 1
                acc = acc';
            end
            if size(mag,1) == 1
                mag = mag';
            end

            R0 = obj.R;
            %% --------- actual measurement --------- %%
            % Normalise accelerometer measurement
            if(norm(acc) == 0)
                a = zeros(3,1);
            else
                a = acc ./ norm(acc);% normalise magnitude
            end
            if(norm(mag) == 0)
                m = zeros(3,1);
            else
                m = mag ./ norm(mag);% normalise magnitude
            end

            %% --------- nominal reference --------- %%
            an = R0' * [0;0;1];

            %% if we have the mag reference, we should use the local mag reference,
            % otherwise, do as follows
            mge = R0*m;
            mgn = [sqrt(mge(1)*mge(1)+mge(2)*mge(2));0;mge(3)];
            mn = R0' * mgn;

            %% Explicit complementary filter on SO3
            wmeas = cross(a, an) + cross(m, mn); %% may apply different weight
            
            fskew = @(x) ([0 -x(3) x(2);x(3) 0 -x(1);-x(2) x(1) 0]);
            
            so3 = fskew((gyro - obj.bias + wmeas * obj.kp));
            Rdot = so3;
            obj.R = R0*expm(Rdot.*obj.SamplePeriod);
            obj.bias = obj.bias - obj.ki * wmeas;
            
            RtR = (obj.R)'*(obj.R);
            E = RtR - eye(3);
            err = max(abs(E));
            if err > 1e-3
                disp('orthogonization');
                %% orthogonization
                [U, ~, V] = svd(obj.R); obj.R = U*V'; if det(obj.R)<0, obj.R = U*diag([1 1 -1])*V'; end
            end
        end
    end
end