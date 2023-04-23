function plots = plot_controller(choice, cont_traj)
    figure(2);
    clf;
    plot(1:length(cont_traj),cont_traj(:,1),'LineWidth', 3);
    if choice > 0
        hold on; 
        plot(1:length(cont_traj),cont_traj(:,2),'LineWidth', 3);
        if choice == 1
            legend('nominal controller','least restrictive controller');
        elseif choice == 2
            legend('nominal controller','qp filtered controller');
        end
    end
    if choice == 0
        legend('nominal controller');
    end
    xlabel('Timesteps')
    ylabel('Control Value')
    x0=80;
    y0=10;
    width=600;
    height=600;
    set(gcf,'position',[x0,y0,width,height])
    title('Controller Profile')
    plots = 0;
end