function xy = simulate_trajectory(x_init, X, H, dt, noise_drift)
    xy(:,1) = x_init;
    controller = X(3*(H+1)+1:end,1);
    for i=1:H      
      xy(:,i+1) = simulate(xy(:,i),controller(i),[0;0;0],dt,noise_drift);
    end
end