function patch_RobomasterF = RobomasterF_patch()
%ROBOMASTERF_PATCH This is a helper function to generate patches for the
%simulated ROBOMASTERF.  YOU SHOULD NEVER HAVE TO USE THIS FUNCTION.
%
% PATCH_DATA = ROBOMASTERF_PATCH() generates a struct containing patch data
% for a robomasterF patch.

%        _______________________
%      3|                       |2
%       |                       |
%       |                       | %robot_width
%       |                       |
%       |                       |
%      4|_______________________|1
              %robot_length
    % Make it facing 0 rads
    robot_length = 0.32;
    robot_width = 0.24; 
    wheel_length = 0.10; 
    wheel_width = 0.045; 
    
    % Helper functions to generate vertex coordinates for a centered
    % rectangle and a helper function to shift a rectangle
    % Verteces in ROW format to be compatible with matlab's patch
    % vertex-face format. 1 in the third column because we will use
    % homogeneous coordinates in 2D!!
    rectangle = @(l, w) [ l/2, -w/2, 1;...
                          l/2,  w/2, 1;...
                         -l/2,  w/2, 1;...
                         -l/2, -w/2, 1];

    shift = @(mat, shiftx, shifty) mat + repmat([shiftx, shifty, 0], size(mat,1), 1);
    
    % Create vertices for body and wheels.
    body = rectangle(robot_length, robot_width);
    wheel = rectangle(wheel_length, wheel_width);
    wheel_1 = shift(wheel,0.1,-0.1);
    wheel_2 = shift(wheel,0.1, 0.1);
    wheel_3 = shift(wheel,-0.1, 0.1);
    wheel_4 = shift(wheel,-0.1,-0.1);
    
    % Putting all the robot vertices together IN ROWS, so that
    % we can use matlab's vertex-face patch definition.
    vertices = [body;...
                wheel_1;...
                wheel_2;...
                wheel_3;...
                wheel_4];

    % Only color the body of the robot.  Everything else is black.
    colors = [ 169/255, 169/255, 169/255;...
                  0   ,    0   ,    0   ;...
                  0   ,    0   ,    0   ;...
                  0   ,    0   ,    0   ;...
                  0   ,    0   ,    0   ];

    % Each "row" of the vertices (body,wheel_i) vector makes a "face". 
    % Tell matlab to connect the vertices of each "row". 
    faces = repmat([1, 2, 3, 4, 1], 5, 1);
    for i = 2:5
       faces(i, :) = faces(i, :) + (i-1)*4;
    end
    
   patch_RobomasterF = []; 
   patch_RobomasterF.vertices = vertices;
   patch_RobomasterF.colors = colors;
   patch_RobomasterF.faces = faces;
end