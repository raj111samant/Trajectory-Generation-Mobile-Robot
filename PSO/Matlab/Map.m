classdef Map < handle
    properties
        Goal = [0, 0];
        MapImage;
        Centroids;
        Variance;
        BoundaryTolerance = 1.1;
        C1 = 0.06;
        C2 = 1200;
        MaxIterations = 1000;
    end

    methods
        function SetGoal(obj, goal)
            obj.Goal = goal;
        end
        
        function goal = GetGoal(obj)
            goal = obj.Goal;
        end
        
        function SetMap(obj, mapname)
            obj.MapImage = im2bw(rgb2gray(imread(mapname)));
        end
        
        function SetMaxIterations(obj, maxiter)
            obj.MaxIterations = maxiter;
        end
        
        function ShowMap(obj)
            figure(100);
            imshow(obj.MapImage);
        end
        
        function ComputeObstracle(obj)
            [B,L] = bwboundaries(obj.MapImage);
            %obj.b = B;
            %obj.l = L;
            for i = 2:length(B) 
                boundry_temp = cell2mat(B(i));
                obj.Centroids(i-1,:) = sum(boundry_temp)./size(boundry_temp,1);
                diff_temp1 = boundry_temp(:,1) - obj.Centroids(i-1,1);
                diff_temp2 = boundry_temp(:,2) - obj.Centroids(i-1,2);
                diff_temp1 = diff_temp1.^2;
                diff_temp2 = diff_temp2.^2;
                norm_temp = diff_temp1 + diff_temp2;
                obj.Variance(i-1) = sqrt(max(norm_temp))*obj.BoundaryTolerance;
            end   
        end
        
        function PlotVariance(obj)
            %figure(102);
            ang=0:0.01:2*pi;
            %imshow(obj.MapImage);
            %hold on
            for i = 1:length(obj.Variance)
                xp=obj.Variance(i)*cos(ang);
                yp=obj.Variance(i)*sin(ang);
                plot(obj.Centroids(i,2)+yp, obj.Centroids(i,1)+xp, '-r');
            end
            %hold off
        end
        
        function cost = CostFunction(obj, pos, time)
            cost1(1,1) = pos(1,1,1) - obj.Goal(1);
            cost1(1,2) = pos(1,1,2) - obj.Goal(2);
            cost1 = sqrt(cost1(1,1)*cost1(1,1) + cost1(1,2)*cost1(1,2));
            cost2 = 0;
            for i=1:length(obj.Variance)
                x1 = pos(1) - obj.Centroids(i,1);
                x2 = pos(2) - obj.Centroids(i,2);
                sig = obj.Variance(i);
                cost2 = cost2 + 10*exp(-(x1*x1 + x2*x2)/(2*sig*sig))/(sig*sqrt(2*pi));
            end
            
            cost =  obj.C2*cost2 + obj.C1*cost1 + cost1^(time*8/obj.MaxIterations);
        end
        
    end

end