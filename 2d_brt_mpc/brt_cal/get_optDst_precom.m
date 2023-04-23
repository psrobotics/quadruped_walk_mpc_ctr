function [precom_optDist_g, precom_optDist_values]=get_optDst_precom(filename)
    % returns the grid and the value of the precomputed disturbance
    precom_optDist_struct = load(filename);
    precom_optDist_g = precom_optDist_struct.g;
    precom_optDist_values{1} = precom_optDist_struct.optDst_val{1};
    precom_optDist_values{2} = precom_optDist_struct.optDst_val{2};
end