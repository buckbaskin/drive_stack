classdef EKFv1
    %EKFv1 Representation of an Extended Kalman Filter for robotic
    % localization
    %  Uses an implementation of a Kalman Filter from:
    %   Probabilistic Robotics (2006) by Thrun, Burgard, Fox pg. 204
    %   Algorithm EKF_localization_known_correspondences(...)
    
    properties
        last_pose;
        last_covariance;
        last_likelihood;
        last_update_time;
        last_control;
        map;
    end
    methods(Static)
        % utility methods
        function [time] = time_now()
            c = clock;
            time = c(4:length(c));
        end
        function [cov] = default_covariance(len)
            % TODO change this to be something more useful
            cov = eye(len,len);
        end
    end
    methods
        % ROS callbacks
        function [obj] = control_cb(obj, cmd)
            obj.last_control = cmd;
            obj = motion_update(obj, cmd, EKFv1.time_now()-obj.last_update_time());
            obj.last_update_time = EKFv1.time_now();
        end
        function [obj] = default_features_cb(obj, data)
            % Run a motion update then run a sensor update
            obj = control_cb(obj, obj.last_control);
            obj = default_sensor_update(obj, data.data, data.corresp);
        end
        
        % useful functions for initialization
        function [obj] = set_map(obj, map)
            obj.map = map;
        end
        function [obj] = user_pose(obj, pose) 
            obj.last_pose = pose;
            obj.last_covariance = default_covariance(5);
        end
    
        
        function [obj] = update(obj, control, features, corresp)
            % calculate ekf update for new data
            % inputs:
            % - obj: ekf object to update
            % - control: 2x1 vector of (linear vel, angular vel). Either
            %    requested or executed control (wheel odom?)
            % - features: mx3 vector of measurements to features:
            %    m x (distance, angle, signature) where the signature is
            %    ignored, because we know which beacon is which
            % - corresp: mx1 vector of labels for the corresponding feature
            %    on the map
            
            obj = totalUpdateEKF(obj, control, features, corresp);
        end
        
        function [obj] = totalUpdateEKF(obj, con, fea, cor)
            dt = EKFv1.time_now() - obj.last_update_time;
            obj = motion_update(obj, con, dt);
            obj = default_sensor_update(obj, fea, cor);
            obj.last_update_time = EKFv1.time_now();
        end
        
        function [pos] = get_current_pos_est(obj)
            pos = obj.last_pose;
        end
        function [cov] = get_current_cov_est(obj)
            cov = obj.last_covariance;
        end
        function [lik] = get_current_lik_est(obj)
            lik = obj.last_likelihood;
        end
        
        function [obj] = motion_update(obj, control, dt)
            old_pose = obj.last_pose;
            old_covar = obj.last_covariance;
            % some useful constants
            % heading = old_pose(theta)
            h = old_pose(3);
            v = control(1);
            w = control(2);
            
            % pos = old_pose + [vel model updates w/o noise]
            pose_est = old_pose + [-v/w*sin(h) + v/w*sin(h + w*dt);
                         v/w*cos(h) - v/w*cos(h + w*dt);
                         w*dt];
            
            % bigG = Jacobian/derviative of the motion update function g
            % wrt old_pose for linearizing the system.
            % G = dg/d(old_pose) = [dx/d(pose_x), dx/d(pose_y), dx/d(pose_z);
            %                       dy/d(pose_x), dy/d(pose_y), dy/d(pose_z);
            %                       dz/d(pose_x), dz/d(pose_y), dz/d(pose_z)];
            bigG = [ 1, 0, -v/w*cos(h) + v/w*cos(h + w*dt);
                     0, 1, -v/w*sin(h) + v/w*sin(h + w*dt);
                     0, 0, 1];
            
            % bigV = [dx'/dv, dx'/dw; dy'/dv, dy'/dw; dh'/dv, dh'/dw];
            % bigV = Jacobian/derivative of the motion model wrt the
            % control inputs (v, w)
            cmplx2 = v*(sin(h)-sin(h+w*dt))/(w*w) + v*cos(h+w*dt)*dt/w;
            cmplx4 = -1*v*(cos(h)-cos(h+w*dt))/(w*w) + v*sin(h+w*dt)*dt/w;
            bigV = [(-1*sin(h)+sin(h+w*dt))/w, cmplx2;
                    (cos(h)-cos(h+w*dt))/w, cmplx4;
                    0, dt];
            
            % bigM = covariance of noise in control space
            % derived from the equation:
            % (actual executed command v, w) = (requested command v,w) + 
            % Error terms as seen below (a1*v^2, etc.)
            % bigM = [a1*v*v+a2*w*w, 0; 0, a3*v*v+a3*w*w];
            % a1-4 = robot motion noise parameters
            % a1 = proportion of error in v from v
            % a2 = proportion of error in v from w
            % a3 = proportion of error in w from v
            % a4 = proportion of noise in w from w
            a = [.1,.05,.05,.1];
            bigM = [a(1)*v*v+a(2)*w*w, 0; 0, a(3)*v*v+a(4)*w*w];
            
            % cov_est = old covariance transformed through the motion model
            %  jacobian at pose_est + mapping of control noise (M) into
            %  state space (via V)
            % cov_est = bigG*old_covar*(G transpose)+V*M*(V transpose);
            % - bigG*old_covar*(G transpose) is the projection of the old
            %    covariance into the new pose space using the linearization
            %    of the motion model of the current position ignoring error
            cov_est = bigG*old_covar*(G.')+bigV*bigM*(bigV.');
            
            obj.last_pose = pose_est;
            obj.last_covariance = cov_est;
        end
        function [obj] = default_sensor_update(obj, fea, cor)
            % bigQ = covariance of measurement noise
            sigma_r = 1;
            sigma_heading = 1;
            sigma_s = 1;
            bigQ = [sigma_r*sigma_r, 0, 0;
                    0, sigma_heading*sigma_heading, 0;
                    0, 0, sigma_s*sigma_s];
            
            feat_est = zeros(len(fea),1);
            
            pose_est = obj.last_pose;
            cov_est = obj.last_covariance;
            
            for i = 1:length(features)
                j = cor(i);
                % q is the distance squared to the corresponding feature
                % q = pow(map(j).x-pose_estimate.x, 2) + pow(map(j).y - pose_estimate.y,2);
                coord = obj.map(j);
                q = pow(coord(1)-pose_est(1), 2) + pow(coord(2) - pose_est(2),2);
                
                % feat_est = [distance, bearing, signature of the known coordinate]
                feat_est(i) = [sqrt(q);
                               atan2(coord(2)-pose_est(2), coord(1)-pose_est(1)) - h;
                               coord(3)];
                
                % bigH = Jacobian/derivative of h wrt to robot location
                %  computed at the pose_estimate
                % h is the measurement model. if h were exact, and location
                %  known, h would exactly match the measurements without
                %  noise. Instead, the estimated measurement is the
                %  h(estimate pose, etc) + gaussian noise.
                % again, H is the derivative of that for linearizing at the
                %  robot's current estimated pose.
                % H = dh/d(pose) = [dr/dx, dr/dy, dr/dz;
                %                   d(bearing)/dx, d(bearing)/dy, d(bearing)/dz;
                %                   ds/dx, ds/dy, ds/dz];
                bigH = [ -1*(coord(1)-pose_est(1))/sqrt(q), -1*(coord(2)-pose_est(2))/sqrt(q), 0;
                         (coord(2)-pose_est(2))/q, -1*(coord(1)-pose_est(1))/q, -1;
                         0, 0, 0];
                
                % bigS = overall measurement prediction uncertainty, sum of
                % below:
                % - H*cov_est*(H transpose) = project the estimated
                %    covariance into the measurement model, giving
                %    uncertainty based on uncertainty
                % - Q = uncertainty due to measurement noise
                bigS = bigH*cov_est*(bigH.')+bigQ;
                
                % bigK = Kalman gain. adjusts update to the pose_est based
                %  on difference in measurement data from expected data
                %  adjusts the cov_est with the linearized measurement model
                %  H and adjusts for uncertainty in measurement prediction
                % ex. if you're confident in your measurement model, then
                %  the gain will be bigger and a bigger difference between
                %  the actual measurement and the predicted one will lead to
                %  a bigger update of the estimated pose.
                bigK = cov_est*(bigH.')/(bigS); % matrix inverse, the slow part
                pose_est = pose_est + bigK*(fea(i) - feat_est(i));
                cov_est = (eye(length(features)) - bigK*bigH)*cov_est;
            end
            
            obj.last_pose = pose_est;
            obj.last_covariance = cov_est;
            
            obj = update_likelihood(obj, fea, feat_est);
        end
        
        function [obj] = update_likelihood(obj, features, feat_est)
            lik = 1;
            for i = 1:len(features)
                % TODO(buckbaskin): check the math here
                % TODO(buckbaskin): check fea vs. feat_est
                det_term = (det(2*pi()*bigS(i)))^(-1/2);
                exp_term = exp(-0.5*((features(i)-feat_est(i)).')/bigS(i)*(features(i)-feat_est(i)));
                lik = lik * det_term * exp_term;
            end
            obj.last_likelihood = lik;
        end
    end
    
end

