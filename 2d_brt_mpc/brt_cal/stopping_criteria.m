function stop = stopping_criteria(current_state, params)
 stop = (norm(current_state(1:2) - [params.goalX; params.goalY]) - params.goalR) < 0;
end