function [] = rbt_anime(x_arr,f_arr,fp_arr,x_ref_arr,T,N,obst)

size_arr = size(x_arr);
len = size_arr(2);

figure(1);

  % v = VideoWriter('obst_mpc_t2','MPEG-4');
  % v.FrameRate = N/T;
  % open(v);

for k = 1:25:len-1
    x_t = x_arr(:,k);
    fp_w_1 = fp_arr(1:3,k);
    fp_w_2 = fp_arr(4:6,k);
    fp_w_3 = fp_arr(7:9,k);
    fp_w_4 = fp_arr(10:12,k);
    
    r_mat = rot_zyx(x_t(1:3));
    
    foot_pos = fp_arr(:,k);
    feetforce_used = f_arr(:,k)*0.5;
    
    %clf;
    hold on;
    axis equal;
    grid on;
    
    view(-34,33);
    axis([-0.5,10,-3,3,0,2]); % viewpoint range, x,y,z
   
    plot_cube(r_mat,0.34,0.2,0.08,x_t(4:6),'black');
    plot3(fp_w_1(1),fp_w_1(2),fp_w_1(3),'o','linewidth',1.2,'color','b','markersize',3);
    plot3(fp_w_2(1),fp_w_2(2),fp_w_2(3),'o','linewidth',1.2,'color','b','markersize',3);
    plot3(fp_w_3(1),fp_w_3(2),fp_w_3(3),'o','linewidth',1.2,'color','b','markersize',3);
    plot3(fp_w_4(1),fp_w_4(2),fp_w_4(3),'o','linewidth',1.2,'color','b','markersize',3);
    
    % draw obst
    r_mat_o = rot_zyx([0;0;0]);
    for i = 1:obst.num
        plot_cube(r_mat_o,obst.r_arr(i)*2/1.414,obst.r_arr(i)*2/1.414,obst.r_arr(i)*2,[obst.cood_x_arr(i);obst.cood_y_arr(i);obst.r_arr(i)],'red');
    end
    plot_cube(r_mat_o,obst.h/1.414,2.8*2,obst.h/1.414,[obst.vood_x_pos;0;obst.h/1.414],'red');
    
    for i=1:4
        x_indx=3*(i-1)+1;
        y_indx=3*(i-1)+2;
        z_indx=3*(i-1)+3;
        
         plot3([foot_pos(x_indx),foot_pos(x_indx)+0.01*feetforce_used(x_indx)],...
            [foot_pos(y_indx),foot_pos(y_indx)+0.01*feetforce_used(y_indx)],...
            [foot_pos(z_indx),foot_pos(z_indx)+0.01*feetforce_used(z_indx)],'linewidth',0.8,'color','r');
        
    end
    
    pause(T/N);
    hold on;
    
       % frame = getframe(gcf);
       % writeVideo(v,frame);
    
end

  % close(v);

end
