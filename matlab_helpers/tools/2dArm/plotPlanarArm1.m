function h = plotPlanarArm1(arm, conf, color, width, joint)
%PLOTPLANARARM Plot Arm class in 2D
%
%   Usage: PLOTPLANARARM(arm, conf, color, width)
%   @arm    Arm object
%   @conf   arm configuration vector
%   @color  color string, use plot convention, e.g. 'r' is red
%   @width  line width
%   @joint if we plot the joint dot or not

position = arm.forwardKinematicsPosition(conf);
position = position(1:2, :);
position = [[0;0], position];

% style = strcat(color, '-');
h(1) = plot(position(1,:), position(2,:),'Color', color, 'LineWidth', width);

if joint 
    h(2) = plot(position(1,1:end-1), position(2,1:end-1), 'k.', 'MarkerSize', 20);
else
    h(2) = plot(position(1,1:end-1), position(2,1:end-1), 'k.', 'MarkerSize', 20, 'MarkerEdgeColor','none');
end

end