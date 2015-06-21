classdef Pso < handle
    properties
        SwarmSize = 64;                       % number of the swarm particles
        MaxIteration = 50;                          % maximum number of iterations
        Inertia = 1.0;
        CorrectionFactor = 2.0;
        DistributionFactor = 5;
        SwarmParticles;
        CostFunction;
    end

    methods
        function obj = Pso(size, m_iteration, inertia, c_factor, distribution)
            obj.SwarmSize = size;                       % number of the swarm particles
            obj.MaxIteration = m_iteration;                          % maximum number of iterations
            obj.Inertia = inertia;
            obj.CorrectionFactor = c_factor;
            obj.DistributionFactor = distribution*sqrt(2)/2;
            obj.SwarmParticles = zeros(obj.SwarmSize,4,2);
            obj.CostFunction = zeros(obj.SwarmSize,1);
        end
        function SetProperty(obj, name, value)
            switch name
                case 'size' 
                    obj.SwarmSize = value;
                case {'maxiterations'}
                    obj.MaxIteration = value;
                case {'inertia'}
                    obj.Inertia = value;
                case {'correctionfactor'}
                    obj.CorrectionFactor = value;
                end
        end
        function InitializeSwarm(obj, pos)
            temp(:,1) = obj.DistributionFactor*(2*rand(obj.SwarmSize, 1) - 1) + pos(1);
            temp(:,2) = obj.DistributionFactor*(2*rand(obj.SwarmSize, 1) - 1) + pos(2);
            obj.SwarmParticles(1:obj.SwarmSize,1,1:2) = temp;          % set the position of the particles in 2D
            obj.SwarmParticles(:,2,:) = 0;                       % set initial velocity for particles
            obj.SwarmParticles(:,4,1) = 10000000; 
            
        end
        function bestpos = Optimize(obj, map , robot, time)
            for iter = 1:obj.MaxIteration
                obj.SwarmParticles(:, 1, 1) = obj.SwarmParticles(:, 1, 1) + obj.SwarmParticles(:, 2, 1)/1.3; %update x position with the velocity
                obj.SwarmParticles(:, 1, 2) = obj.SwarmParticles(:, 1, 2) + obj.SwarmParticles(:, 2, 2)/1.3;       %update y position with the velocity
                x = obj.SwarmParticles(:, 1, 1);                                         % get the updated position
                y = obj.SwarmParticles(:, 1, 2);                                         % updated position
                                                       % evaluate the function using the position of the particle
    
                % compare the function values to find the best ones
                for ii = 1:obj.SwarmSize
                    obj.CostFunction(ii,1) = map.CostFunction( obj.SwarmParticles(ii, 1, :), time);
                    %fprintf('I-%d, S-%d, C-%d, P - %d %d \n', iter, ii, obj.CostFunction(ii,1), obj.SwarmParticles(ii, 1, 1), obj.SwarmParticles(ii, 1, 2));
                    
                    if obj.CostFunction(ii,1) < obj.SwarmParticles(ii,4,1)
                        obj.SwarmParticles(ii, 3, 1) = obj.SwarmParticles(ii, 1, 1);                  % update best x position,
                        obj.SwarmParticles(ii, 3, 2) = obj.SwarmParticles(ii, 1, 2);                  % update best y postions
                        obj.SwarmParticles(ii, 4, 1) = obj.CostFunction(ii,1);                       % update the best value so far
                    end
                end
    
                [~, gbest] = min(obj.SwarmParticles(:, 4, 1));                           % find the best function value in total
    
                % update the velocity of the particles
                temp1 = obj.Inertia*(rand(obj.SwarmSize,1).*obj.SwarmParticles(:, 2, 1)) + obj.CorrectionFactor*(rand(obj.SwarmSize,1).*(obj.SwarmParticles(:, 3, 1) ...
                                                - obj.SwarmParticles(:, 1, 1))) + obj.CorrectionFactor*(rand(obj.SwarmSize,1).*(obj.SwarmParticles(gbest, 3, 1) - obj.SwarmParticles(:, 1, 1)));   %x velocity component
                temp2 = obj.Inertia*(rand(obj.SwarmSize,1).*obj.SwarmParticles(:, 2, 2)) + obj.CorrectionFactor*(rand(obj.SwarmSize,1).*(obj.SwarmParticles(:, 3, 2) ...
                                                - obj.SwarmParticles(:, 1, 2))) + obj.CorrectionFactor*(rand(obj.SwarmSize,1).*(obj.SwarmParticles(gbest, 3, 2) - obj.SwarmParticles(:, 1, 2)));   %y velocity component
    
                for sn = 1:obj.SwarmSize
                    status = robot.MotionFlag(temp1(sn),temp2(sn));
                    if(status == 1)
                        obj.SwarmParticles(sn, 2, 1) = temp1(sn);
                        obj.SwarmParticles(sn, 2, 2) = temp2(sn);
                    end
                end        
                % plot the particles
               % clf;
               % figure(105)
                %imshow(map.MapImage);
               % hold on
               % plot(obj.SwarmParticles(:, 1, 1), obj.SwarmParticles(:, 1, 2), 'gx');             % drawing swarm movements
               % hold off
                %axis([-2 40 -2 40]);
               % pause(.001);                                                 % un-comment this line to decrease the animation speed
                %disp(['iteration: ' num2str(iter)]);
            end
            %fprintf('P - %d, %d',obj.SwarmParticles(ii, 3, 1),obj.SwarmParticles(ii, 3, 2));
            bestpos = [obj.SwarmParticles(ii, 3, 1),obj.SwarmParticles(ii, 3, 2)];

        end
        
        function DisplaySwarmLocation(obj, mapimage)
                %figure(104)
                %imshow(mapimage);
                %hold on
                plot(obj.SwarmParticles(:, 1, 1), obj.SwarmParticles(:, 1, 2), 'gx','LineWidth',10);
        end
     
    end

end