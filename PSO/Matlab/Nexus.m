classdef Nexus < handle
    properties
        Pos = [0, 0];
        StepSize = 20;
    end

    methods
        function obj = Nexus(pos, stepsize)
            obj.Pos = pos;
            obj.StepSize = stepsize;
        end
        function UpdatePosition(obj, newpos)    % Add to speed
            obj.Pos = newpos;
        end
        function pos = GetPosition(obj)
            pos = obj.Pos;
        end
        
        function status = MotionFlag(obj, temp1, temp2)
           dist = sqrt((temp1 - obj.Pos(1))*(temp1 - obj.Pos(1)) + (temp2 - obj.Pos(2))*(temp2 - obj.Pos(2))); 
           if (dist < obj.StepSize*2)
               status = 1;
           else 
               status = 1;
           end
        end
    end

end