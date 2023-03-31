function noise = tests(state,params)
    switch params.test_choice
        case 0
            noise = [0;0];
        case 1
            noise = test_1();
        case 2
            noise = test_2();
        case 3
            noise = test_3();
        case {4,5} 
            noise = test_4(state,params);
    end
end

function optNoise = test_1()
    randdeg = pi*(2*rand() - 1);
    optNoise = 0.1*[cos(randdeg); sin(randdeg)]; % only for simulation
end

function optNoise = test_2()
    randdeg = pi*(2*rand() - 1);
    optNoise = 0.8*[cos(randdeg); sin(randdeg)];
end

function optNoise = test_3()
    randdeg = pi*(2*rand() - 1);
    optNoise = (0.7*rand() + 0.1)*[cos(randdeg); sin(randdeg)];
end

function optNoise = test_4(state,params)
    optNoise = [eval_u(params.precom_optDist_g,params.precom_optDist_values(1),state);...
        eval_u(params.precom_optDist_g,params.precom_optDist_values(2),state)];
end
    