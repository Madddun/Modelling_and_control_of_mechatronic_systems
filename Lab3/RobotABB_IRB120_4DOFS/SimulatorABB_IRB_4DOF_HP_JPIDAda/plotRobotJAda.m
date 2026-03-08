function plotRobotJAda(u)

%PLOTROBOT Animate the robot motion
% u(1): current time
% u(2:ndof): robot joint positions
% u(end): controls if the additional CF and EF markers should be visible
%           1: show additional markers, 0: no additional markers

% global variables to have persistant markers
persistent robot1 iviz samples vsr state_s efMarker cfMarker number_links cf_names fh

% current time
t=u(1);

% Visualization Prameters
lengthEF=0.1;
fontSize=10;
queueSize=15;
fw=1.5;
workSpace=[-fw fw];
plotUpdateFreq=15; % 15 hz visualization

% Figure size and position
sz=get(0,'ScreenSize');
fullScreen=false;

if fullScreen
    f_position=sz;
else
    f_position=[100 400 floor(0.5*sz(3:4))];%[1846 -1014 1147 908];
end

% defines how the axis view point is controlled, 1: from simulink model, 0:
% from plot
axis_view_control=0;

% Plot robot flag. Controlled nby the VisRobot switch in the simulink model.
% This switch should not be changed in run time.
plot_robot=u(9);

if(plot_robot)

    % Plot initialization
    if(t==0)

        % Close all open figures
        close all

        % Get robot name
        rName=evalin('base', 'robot_name');

        % Load robot
        robot1 = loadrobot(rName,"DataFormat","column");

        % Get the DOF from the workspace
        state_s=evalin('base', 'n_dofs');

        % Initialize the robot visualization
        iviz = interactiveRigidBodyTree(robot1,'ShowMarker',false);

        % Get the figure handler and set its position and size
        fh=gcf;
        fh.Position=f_position;

        % Get the axis handler and define the worksapce
        ax = fh.CurrentAxes;
        ax.XLim=workSpace;
        ax.YLim=workSpace;
        ax.ZLim=workSpace;

        % Robot scenario
        plane = collisionBox(1.5,1.5,0.05);
        plane.Pose = trvec2tform([0.25 0 -0.025]);
        show(plane,'Parent', ax);

        leftShelf = collisionBox(0.25,0.1,0.2);
        leftShelf.Pose = trvec2tform([0.3 -.65 0.1]);
        [~, patchObj] = show(leftShelf,'Parent',ax);
        patchObj.FaceColor = [0 0 1];

        rightShelf = collisionBox(0.25,0.1,0.2);
        rightShelf.Pose = trvec2tform([0.3 .65 0.1]);
        [~, patchObj] = show(rightShelf,'Parent',ax);
        patchObj.FaceColor = [0 0 1];

        % Initialize additional markers, coordinate frames
        efMarker=initEF(eye(4,4),lengthEF,0,fontSize,queueSize);

        cf_names={'cf_w','cf_0','cf_1','cf_2','cf_3'};
        number_links=numel(cf_names);

        for i=1:number_links
            cfMarker{i}=initGenericCF(lengthEF,0,cf_names{i},fontSize);
        end

        % Shows figure with all the objects
        showFigure(iviz)

        % Sampler counter to define when to update the plot
        samples=0;

        % Visualization sample rate
        smodel_name=get_param(bdroot, 'Name');
        ssr=str2double(get_param(smodel_name,'FixedStep'));
        vsr=floor(1/(plotUpdateFreq*ssr)); % Update freq

    end

    % Update the robot link poses and additional markers. Currently set for
    % 15Hz, see vsr.

    if(samples==vsr)

        % new joint positions
        q=u(2:state_s+1);

        % Updates the plot with the new joint positions
        % In this case, we are using just joints 1 2 3 and 5.
        % Then, the real q state for the visualization is:
        % q1 q2 q3 0 q4 0 (joint 4 and 6 are fixed)

        qreal=[q(1:3); 0.0; q(4); 0.0];

        % Changes the robot position
        iviz.Configuration=qreal;

        % Controls the plot view point from the simulink model
        if axis_view_control
            v=u(state_s+2:end-1);
            ax = fh.CurrentAxes;
            ax.View=v;
        end

        % Set the current time as the plot title
        ax.Title.String=sprintf('time = %2.2f s', t);

        % Controls if the additional CF and EF are plotted, when activated the
        % simulation will be slower
        if u(7)

            % Update CFs poses
            % Kinematic Parameters L1,L2,L7,L8 and alpha
            L=abbIRB4_params;

            % HT robot base (link0) wrt world cf. In this case is I (4x4)
            T0_W=eye(4);

            % Get the absolute HTs for all the joints with the new joint position
            [~, HT_W] = getAbsoluteHT_abbIRB4(q,L, T0_W);

            % Update the CF markers
            for i=1:number_links
                updateCF(cfMarker{i},HT_W{i},lengthEF);
            end

            %Update EF pose
            updateEF(efMarker,HT_W{end},eye(4),eye(4),lengthEF,queueSize);

        else
            % When dissabled, the markers are still active. Then, we make them "invisible"
            for i=1:number_links
                updateCF(cfMarker{i},zeros(4),0);
            end
            updateEF(efMarker,zeros(4),zeros(4),zeros(4),0,queueSize);

        end


        % Update figure
        %drawnow limitrate (low graphic resources, it will slow down also the scopes)
        drawnow

        %Reset the visualizer sampler
        samples=0;
    end

    % increase the sampler
    samples=samples+1;

end

end



