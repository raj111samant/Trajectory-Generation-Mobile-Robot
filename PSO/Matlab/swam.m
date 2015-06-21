%%
clc
clear all
close all

writerObj = VideoWriter('out.avi'); % Name it.
writerObj.FrameRate = 15; % How many frames per second.
open(writerObj); 

tic
Inferno = Map;
Inferno.SetGoal([80,400]);
Inferno.SetMaxIterations(800);
Inferno.GetGoal()
Inferno.SetMap('map1.png');
%Inferno.ShowMap();
% figure(100)
% imshow(Inferno.MapImage);
% hold on
% plot(80,400,'or','LineWidth',20)
% hold off
N7 = Nexus([500,10], 10);
%N7.GetPosition();
%N7.UpdatePosition([3,4]);
%N7.GetPosition();

ShowDetails(Inferno, N7);
Inferno.ComputeObstracle();
 Inferno.PlotVariance();
    
n7 = Pso(25, 15, 0.2, 0.5, N7.StepSize/1.1);

%%
figure(1)

imshow(Inferno.MapImage);
title(sprintf('PSO based Trajectory generation and Obstracle avoidance on mobile robot'))
hold on

 plot(Inferno.Goal(1),Inferno.Goal(2),'or','LineWidth',10);
     Inferno.PlotVariance();
for j =1:125
 frame = getframe(gcf);
n7.InitializeSwarm(N7.Pos);
n7.DisplaySwarmLocation(Inferno.MapImage);
plot(Inferno.Goal(1),Inferno.Goal(2),'or','LineWidth',10);

newpos =  n7.Optimize(Inferno, N7, j);
N7.UpdatePosition(newpos);
%ShowDetails(Inferno, N7);
plot(N7.Pos(1),N7.Pos(2),'*b','LineWidth',1);
 writeVideo(writerObj, frame);
pause(0.005)
end


close(writerObj); % Saves the movie.



