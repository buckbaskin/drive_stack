classdef AMCLv1
    %AMCLV1 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        w_slow;
        w_fast;
        a_slow;
        a_fast;
        
        last_particles;
        last_update_time;
        
        map;
    end
    
    methods
        function [ obj ] = update(obj, control, measurement)
            obj = update_AMCL(obj, obj.last_particles, control, measurement, obj.map);
        end
        function [ obj ] = update_AMCL(obj, particles, control, measurements, map)
            candidate_particles = [];
            output_particles = [];
            w_avg = 0;
            for m = 1:length(particles)
                new_pose = sample_motion_model(obj, control, particles(m);
                new_likelihood = measurement_model(obj, measurements, new_pose, map);
                new_pair = [new_pose, new_likelihood];
                candidate_particles = [candidate_particles; new_pair];
                w_avg = w_avg + 1/length(particles)*new_likelihood;
            end
            
            obj.w_slow = obj.w_slow+obj.a_slow*(w_avg - obj.w_slow);
            obj.w_fast = obj.w_fast+obj.a_fast*(w_avg - obj.w_fast);
            
            for m = 1:length(particles)
                rand_prob = max(0.0, 1 - obj.w_fast/obj.w_snow);
                if frandom() < rand_prob
                    output_particles = [output_particles; random_pose(obj)];
                else
                    output_particles = [output_particles; select_pose(obj, candidate_particles)];
                end
            end
            
            obj.last_particles = output_particles;
            obj.last_update = time.now();
        end
        function [ pose ] = sample_motion_model(obj, control, start_pose)
            % TODO(buckbaskin): implement this for a particular motion
            % model
            x = start_pose(1);
            y = start_pose(2);
            heading = start_pose(3);
            v = control(1);
            w = control(2);
            dt = time.now() - obj.last_time();
            dheading = w*dt;
            heading_avg = heading+dheading/2;
            dx = v*cos(heading_avg)*dt;
            dy = v*sin(heading_avg)*dt;
            
            x = x + dx;
            y = y + dy;
            heading = heading + dheading;
            pose = [x; y; heading];
        end
        function [ lik ] = measurement_model(obj, measurements, pose, map)
            % TODO(buckbaskin): implement this for a specific set of
            % measurements
        end
        function [ pose ] = random_pose(obj)
            max_x = 10;
            max_y = 10;
            x = frandom()*max_x;
            y = frandom()*max_y;
            heading = frandom()*2*pi;
            pose = [x; y; heading];
        end
        function [ pose ] = select_pose(obj, candidates)
            total_likelihood = 0;
            for i = 1:length(candidates)
                pair = candidates(i);
                total_likelihood = total_likelihood + pair(2);
            end
            
            randal = frandom()*total_likelihood;
            counter = 0;
            while (randal > 0)
                pair = candidates(i);
                randal = randal - pair(2);
                counter = counter + 1;
            end
            pose = candidates(counter);
        end
    end
    
end

