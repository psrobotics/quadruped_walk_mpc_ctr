function u_filtered = get_safety_controller(x_curr,u_nom,params)  
   
    %% TODO
    % Code in the least restrictive safety filter that returns the safe
    % controller if the nominal controller leads the next state into an
    % unsafe region
    % BRT signed distant data params.data(dim_x,dim_y,dim_theta,index_t)

    % Get wrost distrubance for current timestep
    wrost_dist_curr = [eval_u(params.g,params.worst_dist(1),x_curr);...
                       eval_u(params.g,params.worst_dist(2),x_curr)]
                   
    % Simulate next state with nominal control and wrost distrubance
    x_next = simulate(x_curr, u_nom, 0, params.dt, wrost_dist_curr);
    % Use final BRT to get next step signed distance
    next_signed_distance = eval_u(params.g,params.data(:,:,:,end),x_next)
    
    if (next_signed_distance < 0)  % if next step signed distance < 0, the system will end in unsafe state!
            u_filtered = eval_u(params.g,params.safety_controller,x_curr); % get safety controller
            fprintf('safety_ctroller take control');
    else
            u_filtered = u_nom;
    end

end