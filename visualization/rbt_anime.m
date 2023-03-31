function [] = rbt_anime(x_arr,f_arr,fp_arr,x_ref_arr,fp_ref_arr,T,N)

size_arr = size(x_arr);
len = size_arr(2);

figure(1);

%  v = VideoWriter('jump_d_obst2','MPEG-4');
%  v.FrameRate = N/T;
%  open(v);

for k = 1:1:len-1
    
    x_t = x_arr(:,k);
    x_ref_t = x_ref_arr(:,k);
    
    r_mat = rot_zyx(x_t(1:3));
    r_mat_ref = rot_zyx(x_ref_t(1:3));
    
    fp_w_1 = fp_arr(1:3,k);
    fp_w_2 = fp_arr(4:6,k);
    fp_w_3 = fp_arr(7:9,k);
    fp_w_4 = fp_arr(10:12,k);
    
    fp_r_1 = fp_ref_arr(1:3,k);
    fp_r_2 = fp_ref_arr(4:6,k);
    fp_r_3 = fp_ref_arr(7:9,k);
    fp_r_4 = fp_ref_arr(10:12,k);
    
    foot_pos = fp_arr(:,k);
    feetforce_used = f_arr(:,k)*0.5;
    
    clf;
    hold on;
    axis equal;
    grid on;
    
    view(-34,33);
    axis([-0.5,2*4,-2.5,1,0,1.2]);
    
    plot_cube(r_mat,0.34,0.2,0.08,x_t(4:6),'green');
    plot_cube(r_mat_ref,0.34,0.2,0.08,x_ref_t(4:6),'black');
    
    plot3(fp_w_1(1),fp_w_1(2),fp_w_1(3),'o','linewidth',1.2,'color','b','markersize',3);
    plot3(fp_w_2(1),fp_w_2(2),fp_w_2(3),'o','linewidth',1.2,'color','b','markersize',3);
    plot3(fp_w_3(1),fp_w_3(2),fp_w_3(3),'o','linewidth',1.2,'color','b','markersize',3);
    plot3(fp_w_4(1),fp_w_4(2),fp_w_4(3),'o','linewidth',1.2,'color','b','markersize',3);
    
    % plot fp ref point
    plot3(fp_r_1(1),fp_r_1(2),fp_r_1(3),'o','linewidth',1.2,'color','g','markersize',3);
    plot3(fp_r_2(1),fp_r_2(2),fp_r_2(3),'o','linewidth',1.2,'color','g','markersize',3);
    plot3(fp_r_3(1),fp_r_3(2),fp_r_3(3),'o','linewidth',1.2,'color','g','markersize',3);
    plot3(fp_r_4(1),fp_r_4(2),fp_r_4(3),'o','linewidth',1.2,'color','g','markersize',3);
    
    % plot obst
    r_0 = eye(3,3);
    obst_1_mid = [3.2;0.2;0];
    obst_2_mid = [6.2;-0.8;0];
    plot_cube(r_0,0.4,0.4,1,obst_1_mid,'red');
    plot_cube(r_0,0.4,0.4,1,obst_2_mid,'red');
    
    for i=1:4
        x_indx=3*(i-1)+1;
        y_indx=3*(i-1)+2;
        z_indx=3*(i-1)+3;
        
         plot3([foot_pos(x_indx),foot_pos(x_indx)+0.01*feetforce_used(x_indx)],...
            [foot_pos(y_indx),foot_pos(y_indx)+0.01*feetforce_used(y_indx)],...
            [foot_pos(z_indx),foot_pos(z_indx)+0.01*feetforce_used(z_indx)],'linewidth',0.8,'color','r');
        
    end
    
    pause(T/N);
    
%       frame = getframe(gcf);
%       writeVideo(v,frame);
    
end

%  close(v);

end

