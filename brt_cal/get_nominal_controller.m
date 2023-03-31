function u_nom = get_nominal_controller(xinit,params)
    % return the mpc based nominal controller
    X = mpc_another(params.H, params.goalX, params.goalY, params.dt, params.wMax, xinit, params.speed);
    u_nom = X(3*(params.H+1)+1,1);
end