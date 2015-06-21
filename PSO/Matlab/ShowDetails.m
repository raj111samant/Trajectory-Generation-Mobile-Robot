function ShowDetails( map, robot )

    figure(101)
    imshow(map.MapImage);
    hold on
    plot(map.Goal(1),map.Goal(2),'or','LineWidth',10);
    plot(robot.Pos(1),robot.Pos(2),'*b','LineWidth',10);
end

