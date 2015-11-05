classdef EKFv1
    %EKFv1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        last_pose;
        last_covariance;
        last_update_time;
        last_control;
        new_features;
        new_corresp;
        map;
    end
    
    methods
        function pose_cb(pose)
            obj.last_pose = pose;
            obj.last_covariance = pose.covariance;
            % last_update_time = now();
        end
        function control_cb(cmd)
            obj.last_control = cmd;
        end
        function features_cb(data)
            obj.new_features.append(data);
            obj.new_corresp.append(data.corresp);
        end
        function set_map(map)
            obj.map = map;
        end
        
        % TODO(buckbaskin): break up update into functions that run when
        % new data for any given piece comes in (ex. change in cmd, new
        % sensor reading, that kinda thing)
        
        function [pose, covar, likelihood] = update(old_pose, old_covar, control, features, corresp, map, dt)
            % calculate ekf update for new data
            % inputs:
            % - old_pose: nx1 vector of state from the last time step 
            %   (x, y, heading) n=3
            % - old_covar: nxn covariance matrix of state from the last
            %    time step
            % - control: 2x1 vector of (linear vel, angular vel). Either
            %    requested or executed control (wheel odom?)
            % - features: mx3 vector of measurements to features:
            %    m x (distance, angle, uncertainty)
            % - corresp: mx1 vector of labels for the corresponding feature
            %    on the map
            % - map: a matlab containers.Map(k,v) that maps labels to
            %    feature locations in the map frame
            % - dt: time step between updates
            
            [pose, covar, likelihood] = updateEKF(old_pose, old_covar, control, features, corresp, map, dt);
        end
        
        function [pos, cov, lik] = updateEKF(opo, oco, con, fea, cor, map, dt)
            % heading = opo(theta)
            heading = opo(3);
            
            v = con(1);
            w = con(2);
            % bigG = Jacobian/derviative of the motion update function g
            % wrt old_pose for linearizing the system.
            bigG = [ 1, 0, -v/w*cos(heading) + v/w*cos(heading + w*dt);
                     0, 1, -v/w*sin(heading) + v/w*sin(heading + w*dt);
                     0, 0, 1];
            
            % not in a version of algorithm
            % bigV = [cmplx1, cmplx2; cmplx3, cmplx4; 0, dt];
            % bigV = TODO(buckbaskin): English description
            cmplx2 = v*(sin(heading)-sin(heading+w*dt))/(w*w) + v*cos(heading+w*dt)*dt/w;
            cmplx4 = -1*v*(cos(heading)-cos(heading+w*dt))/(w*w) + v*sin(heading+w*dt)*dt/w;
            bigV = [(-1*sin(heading)+sin(heading+w*dt))/w, cmplx2;
                    (cos(heading)-cos(heading+w*dt))/w, cmplx4;
                    0, dt];
            
            % bigM = [a1*v*v+a2*w*w, 0; 0, a3*v*v+a3*w*w];
            % a1-4 = robot motion noise parameters
            a = [.1,.05,.05,.1];
            bigM = [a(1)*v*v+a(2)*w*w, 0; 0, a(3)*v*v+a(4)*w*w];
            
            % pos = opo + [vel model updates w/o noise]
            pose_est = opo + [-v/w*sin(heading) + v/w*sin(heading + w*dt);
                         v/w*cos(heading) - v/w*cos(heading + w*dt);
                         w*dt];
            
            % cov = bigG*oco*(G transpose)+V*M*(V transpose);
            cov_est = bigG*oco*(G.')+bigV*bigM*(bigV.');
            
            % bigQ = covariance of measurement noise
            sigma_r = 1;
            sigma_heading = 1;
            sigma_s = 1;
            bigQ = [sigma_r*sigma_r, 0, 0;
                    0, sigma_heading*sigma_heading, 0;
                    0, 0, sigma_s*sigma_s];
            
            feat_est = zeros(len(fea),1);
            for i = 1:length(features)
                j = cor(i);
                % q = pow(map(j).x-pose_estimate.x, 2) + pow(map(j).y - pose_estimate.y,2);
                coord = map(j);
                q = pow(coord(1)-pose_est(1), 2) + pow(coord(2) - pose_est(2),2);
                feat_est(i) = [sqrt(q);
                               atan2(coord(2)-pose_est(2), coord(1)-pose_est(1)) - heading;
                               coord(3)];
                % bigH = TODO(buckbaskin): English description
                bigH = [ -1*(coord(1)-pose_est(1))/sqrt(q), -1*(coord(2)-pose_est(2))/sqrt(q), 0;
                         (coord(2)-pose_est(2))/q, -1*(coord(1)-pose_est(1))/q, -1;
                         0, 0, 0];
                % bigS = TODO(buckbaskin): English description
                bigS = bigH*cov_est*(bigH.')+bigQ;
                
                % bigK = Kalman gain. adjusts update to the pose_est based
                % on difference in measurement data from expected data
                bigK = cov_est*(bigH.')/(bigS); % matrix inverse, the slow part
                pose_est = pose_est + bigK*(fea(i) - feat_est(i));
                cov_est = (eye(length(features)) - bigK*bigH)*cov_est;
            end
            
            pos = pose_est;
            cov = cov_est;
            
            % lik = % complicated formula % TODO(buckbaskin): start here
        end
    end
    
end

