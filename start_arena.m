function r = start_arena(N,initial_conditions,config)

    go_to_initial_positions(N,initial_conditions,config)
    
    %%Simulation properties%%
    r.Dt = 0.03; %Iteration time
    
    r.checked_poses_already = false; % Whether GET_POSES has been checked this iteration
    r.called_step_already = true; % Whether STEP has been called this iteration
    r.iteration = 0; % How many times STEP has been called
    %errors = {}; % Accumulated errors for the simulation

    %%Arena Properties%%

    %arenavertices = [x_i,y_i,z_i]
    r.arena_vertices = [ 6.5 ,-2.5,0;...
                         6.5 , 2.5,0;...
                        -6.5 , 2.5,0;...
                        -6.5 , 0.5,0;...
                        -3.14,-2.5,0];
    %arean_boundaries = [xmin, xmax, ymin, ymax]
    arena_boundaries = [min(r.arena_vertices(:,1)), max(r.arena_vertices(:,1)), min(r.arena_vertices(:,2)), max(r.arena_vertices(:,2))];
    offset = 0.05;

    %%Display arena%%
    
    %Figuer handle
    r.figure_handle = figure;
    fig = r.figure_handle;    
    %Boundary patch
    r.boundary_patch = patch('XData', [r.arena_vertices(:,1);r.arena_vertices(1,1)], ...
                             'YData', [r.arena_vertices(:,2);r.arena_vertices(1,2)], ...
                             'FaceColor', 'none', ...
                             'LineWidth', 3, ...,
                             'EdgeColor', [0, 0, 0]);
    % Set axis
    ax = fig.CurrentAxes;
    % Limit view to xMin/xMax/yMin/yMax
    axis(ax, [arena_boundaries(1)-offset, arena_boundaries(2)+offset, arena_boundaries(3)-offset, arena_boundaries(4)+offset])
    set(ax, 'PlotBoxAspectRatio', [1 1 1], 'DataAspectRatio', [1 1 1])
    axis(ax, 'off')            
    hold on

    %%Display robots%%

    assert(N.RobomasterF >= 0 && N.RobomasterF <= 5, 'Number of RobomasterF (%i) must be >= 0 and <= 5', N.RobomasterF);
    
    r.poses.RobomasterF = zeros(3,N.RobomasterF);
    r.ptu.RobomasterF = zeros(2,N.RobomasterF);

    %RobomasterF
    %r.robot_handle.robomasterF = cell(1, N.robomasterF);
    data = RobomasterF_patch;
    %vertices in ROWS to use matlab's patch vertex-face format
    base_robot_vertices = data.vertices;
    for i=1:1:N.RobomasterF
        r.poses.RobomasterF(:,i) = initial_conditions.RobomasterF(1:3,i);
        %r.ptu.robomasterF(:,i)   = intial_conditions.RobomasterF(4:5,i);
        x  = r.poses.RobomasterF(1,i);
        y  = r.poses.RobomasterF(2,i);
        th = r.poses.RobomasterF(3,i);
        %Not, WRB, but WTB because we are using homogeneous coords in 2D!!
        WTB = [cos(th), -sin(th), x;
               sin(th),  cos(th), y;
                  0   ,     0   , 1]; 
        current_robot_vertices = (base_robot_vertices*WTB');
        r.patch_handle.RobomasterF(i) = patch('Vertices', current_robot_vertices(:, 1:2), ...
                                      'Faces', data.faces, ...
                                      'FaceColor', 'flat', ...
                                      'FaceVertexCData', data.colors, ...
                                      'EdgeColor','none');
    end
end