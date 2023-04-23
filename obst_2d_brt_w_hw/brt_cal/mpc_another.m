function X = mpc_another(H, goalX, goalY, dt, wMax, xinit, v)

    %% Optimization variables
    % [x(1), x(2), ..., x(101),y(1), y(2), ...y(101), theta(1), theta(2), ...,theta(101),
    % thetadot(1), thetadot(2), ..., thetadot(100)]
    % Total number of varaibles: 101*3 + 100 = 403 

    % setup number of decision variables
    num_state_decision = 3*(H+1);
    num_control_decision = 1*(H);
    num_decision = num_state_decision + num_control_decision;
    

    %% setup inital condition
    x0 = zeros(num_decision,1);
    x0(1) = xinit(1);
    x0(H+2) = xinit(2);
    x0(3*(H+1)+1) = xinit(3);
    
    %% Setup control bounds
    % Lower bounds
    LB = [-100*ones(H+1, 1); -100*ones(H+1, 1);-100*ones(H+1, 1); -wMax*ones(H, 1)];
    UB = -1*LB;
    
    %% Perform the optimization and solve the MPC problem
    options = optimoptions('fmincon','Display','off', 'Algorithm', 'sqp', 'MaxFunctionEvaluations', 300000);
    nonlcon = @(z)dynConst(z,v,xinit,dt);
    X = fmincon(@(x) myfun(x, H, goalX, goalY), x0,[],[], [], [], LB(:, 1), UB(:, 1), nonlcon, options);

end

%% Setup cost function
function f = myfun(x, H, goalX, goalY)
    % Penalize the distance from the goal state
    f = sum((x(1:H+1) - goalX).^2) + sum((x(H+2:2*H+2) - goalY).^2);
end