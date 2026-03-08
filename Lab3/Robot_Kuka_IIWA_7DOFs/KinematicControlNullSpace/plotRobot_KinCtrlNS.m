function plotRobot_KinCtrlNS(u)
%plotRobot_KinCtrlNS Animates the robot motion. Nothing to do here!
% u(1): current time
% u(2:8): robot joint positions
% u(9:24): Ht EF desired pose as a row vector 16x1
% u(25:26): Camera view angles 2x1
% u(27): controls if the additional CF and EF markers should be visible
%           1: show additional markers, 0: no additional markers

% global variables to have persistant markers
persistent robot1 iviz samples vsr cfMarker number_links cf_names fh efMarker
% persistent Mellips sxe elipseHandler textHandler

% current time
t=u(1);

% Visualization Prameters
lengthEF=0.12;
fontSize=10;
queueSize=15;
fw=1.35;
workSpace=[-fw fw];
plotUpdateFreq=20; % 20 hz visualization
label_offset=0.3;
colorS='kr';
smodel_name=get_param(bdroot, 'Name');
dt=str2double(get_param(smodel_name,'FixedStep'));

% Figure size and position
sz=get(0,'ScreenSize');
fullScreen=false;

if fullScreen
    f_position=sz;
else
    f_position=[1 -354 1707 1343]; %[100 400 floor(0.5*sz(3:4))];%[1591 -1079 1920 988];
end

% defines how the axis view point is controlled, 1: from simulink model, 0:
% from plot
axis_view_control=0;

%% Joint State
% Joint positions
Q=u(2:8);

% Desired EF pose wrt wcf
Hd_W=reshape(u(9:24),4,4);


%% Plot initialization
if(t==0)

    % Close all open figures
    close all

    % Get robot name
    rName=evalin('base', 'robot_name');

    % Load robot model
    robot1 = loadrobot(rName,"DataFormat","column","Gravity",[0 0 -9.81]);

    %% Initialize the robot visualization
    iviz = interactiveRigidBodyTree(robot1,'ShowMarker',false);

    % Get the figure handler and set its position and size
    fh=gcf;
    fh.Position=f_position;

    % Get the axis handler and define the worksapce
    ax = fh.CurrentAxes;
    ax.XLim=workSpace;
    ax.YLim=workSpace;
    ax.ZLim=[0 fw];
    ax.CameraViewAngle=3;

    %% Initialize additional markers, coordinate frames
    efMarker=initEF(eye(4,4),lengthEF,0,fontSize,queueSize);

    %% CF initialization
    cf_names={'cf_w','cf_0','cf_{7}','cf_{ef_d}'};
    number_links=numel(cf_names);

    for i=1:number_links
        cfMarker{i}=initGenericCF(lengthEF,0,cf_names{i},fontSize);
    end


    % Shows figure with all the objects
    showFigure(iviz)

    % Sampler counter to define when to update the plot
    samples=0;

    % Visualization sample rate 60Hz
    vsr=floor(1/(plotUpdateFreq*dt)); % Update freq

end

if(samples==vsr)

    % Update the robot link poses and additional markers. Currently set for
    % 20Hz, see vsr.

    %% Plot Robot

    % Updates the plot with the new joint positions
    qreal=Q;

    % Changes the robot position
    iviz.Configuration=qreal;

    % Controls the plot view point from the simulink model
    if axis_view_control
        v=u(25:26);
        ax = fh.CurrentAxes;
        ax.View=v;
    end

    % Set the current time as the plot title
    ax.Title.String=sprintf('time = %2.2f s', t);

    %% Update additional CFs
    % Controls if the additional CF and EF are plotted, when activated the
    % simulation will be slower
    if u(27)

        %% Update the CF markers
        % Kinematic Parameters L1,L2,L3,and L4
        L=kukaIIWA7_params;

        % HT robot base (link0) wrt world cf. In this case is I (4x4)
        T0_W=eye(4);

        % Get the absolute HTs for all the joints with the new joint position
        [~, HT_W] = getAbsoluteHT_kukaIIWA7(Q,L, T0_W);

        %Update EF pose
        updateEF(efMarker,HT_W{end},eye(4),eye(4),lengthEF,queueSize);
        
        updateCF(cfMarker{1},HT_W{1},lengthEF);
        updateCF(cfMarker{2},HT_W{2},lengthEF);
        updateCF(cfMarker{3},HT_W{9},lengthEF);
        updateCF(cfMarker{4},Hd_W,lengthEF);

    else
        % When dissabled, the markers are still active. Then, we make them "invisible"
        updateEF(efMarker,zeros(4),zeros(4),zeros(4),0,queueSize);
        for i=1:number_links
            updateCF(cfMarker{i},zeros(4),0);
        end

    end


    % Update figure
    %drawnow limitrate (low graphic resources, it will slow down also the scopes)
    drawnow

    %% Reset the visualizer sampler
    samples=0;

end


% increase the sampler
samples=samples+1;


end



