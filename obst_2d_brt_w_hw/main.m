%% cam setup
webcamlist
cam = webcam(2)


%%
cam.AvailableResolutions
cam.Resolution = '864x480';

%%
%preview(cam)

%% cam define
focalLength    = [1394.6027293299926 1394.6027293299926]; 
principalPoint = [995.588675691456 599.3212928484164];
imageSize      = [480 864];
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize)
tagSize = 0.075;

%% tag define
tagFamily = ["tag36h11","tagCircle21h7","tagCircle49h12","tagCustom48h12","tagStandard41h12"];

%% get obst pose, cal brt based on that

    img = snapshot(cam);
    I = undistortImage(img,intrinsics,OutputView="same");
    [id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);
    worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];

    for i = 1:length(pose)
        % Get image coordinates for axes.
        imagePoints = world2img(worldPoints,pose(i),intrinsics);

        % Draw colored axes.
        I = insertShape(I,Line=[imagePoints(1,:) imagePoints(2,:); ...
            imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
            Color=["red","green","blue"],LineWidth=7);

        I = insertText(I,loc(1,:,i),id(i),BoxOpacity=1,FontSize=25);
    end

    rbt_pose = [];
    origin_pose = [];
    obst_pose_1 = [];
    obst_pose_2 = [];

    % get tag id=16 pose
    for i = 1:length(id)
        if(id(i)==16)
            rbt_pose = pose(i);
        elseif(id(i)==20)
            origin_pose = pose(i);
        elseif(id(i)==17)
            obst_pose_1 = pose(i);
        elseif(id(i)==21)
            obst_pose_2 = pose(i);
        end
    end


    % calculate the relate pos of obst
    brt_params.obst_pos_1 = obst_pose_1.Translation - origin_pose.Translation % x y z
    brt_params.obst_pos_2 = obst_pose_2.Translation - origin_pose.Translation % x y z
    brt_params.obst_l = 0.1; % 10cm box

    % motion box range, x 0.7m, y 0.35m
    brt_params.grid_size = [0.7 0.35];

    imshow(I)

%% calcuate brt
addpath('brt_cal\')
addpath(genpath('tool_box\helperOC'))  
addpath(genpath('tool_box\ToolboxLS'))
params = get_params(brt_params);

disp('Pre-computing the safety controller with the BRT.........................')
% the BRT gives us the safety controller for free
[params.safety_controller, params.worst_dist, params.data, params.tau, params.g, params.derivatives, params.grid_info] = BRT_computation(params);

%% brt vis
color_map_t = get_brt_colormap(params.data,pi/2);
imshow(color_map_t);

%% connect to rbt hardware
delete(instrfindall);
robot_hw = serial('COM8','BaudRate',115200)
fopen(robot_hw);

%%
fclose(robot_hw);
delfete(robot_hw);

%%
%%fprintf(robot_hw,'a\n'); 
%%

  v = VideoWriter('rbt_10','MPEG-4');
  v.FrameRate = 10;
  open(v);


for f = 1:400
    
    img = snapshot(cam);
    I = undistortImage(img,intrinsics,OutputView="same");
    [id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);
    worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];

    for i = 1:length(pose)
        % Get image coordinates for axes.
        imagePoints = world2img(worldPoints,pose(i),intrinsics);

        % Draw colored axes.
        I = insertShape(I,Line=[imagePoints(1,:) imagePoints(2,:); ...
            imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
            Color=["red","green","blue"],LineWidth=7);

        I = insertText(I,loc(1,:,i),id(i),BoxOpacity=1,FontSize=25);
    end

    rbt_pose = [];

    rbt_flag  = 0;

    % get tag id=16 pose
    for i = 1:length(id)
        if(id(i)==16)
            rbt_pose = pose(i);
            rbt_flag = 1;
        end
    end

    if(rbt_flag == 1)
        R_mat = rbt_pose.R;
        rbt_yaw = rotm2eul(rbt_pose.R);
        rbt_yaw_brt = rbt_yaw(1) - pi/2; % rbt yaw angle, -pi/2 -y, 0 +x, pi/2 +y

        % overlay brt colormap
        color_map_t = get_brt_colormap(params.data,-1*rbt_yaw(1) + pi/2 + 0.001);
        I(:,:,2:3) = I(:,:,2:3) + color_map_t;

        % control part
        rbt_pos = rbt_pose.Translation - origin_pose.Translation;
        rbt_state = [rbt_pos(1); rbt_pos(2); rbt_yaw_brt]


        signed_distance = eval_u(params.g,params.data(:,:,:,end),rbt_state)
        %add signed dist to image
        if(~isnan(signed_distance))
            I = insertText(I,loc(1,:,1),signed_distance,BoxOpacity=1,FontSize=25);
        end

        if(signed_distance < 0.05)
            %unsafe
            u_filtered = eval_u(params.g,params.safety_controller,rbt_state)

            if(u_filtered < 0)
                fprintf(robot_hw,'b\n'); 
            else
                fprintf(robot_hw,'c\n'); 
            end
            
        else
            % normal bangbang control
            if(rbt_yaw_brt < -0.3)
                fprintf(robot_hw,'c\n'); 
            elseif(rbt_yaw_brt > 0.3)
                fprintf(robot_hw,'b\n'); 
            else
                fprintf(robot_hw,'a\n'); 
            end

        end


    end

    %calculate the relate pos of obst
    
    imshow(I)

    writeVideo(v,I);
end

  close(v);
