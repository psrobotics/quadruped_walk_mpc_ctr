function [] = rbt_anime(x_arr,f_arr,fp_arr,x_ref_arr,T,N)

size_arr = size(x_arr);
len = size_arr(2);

figure(1);

 % v = VideoWriter('c_walking_1','MPEG-4');
 % v.FrameRate = N/T;
 % open(v);

for k = 1:len-1
    x_t = x_arr(:,k);
    fp_w_1 = fp_arr(1:3,k);
    fp_w_2 = fp_arr(4:6,k);
    fp_w_3 = fp_arr(7:9,k);
    fp_w_4 = fp_arr(10:12,k);
    
    r_mat = rot_zyx(x_t(1:3));
    
    foot_pos = fp_arr(:,k);
    feetforce_used = f_arr(:,k)*0.5;
    
    clf;
    hold on;
    axis equal;
    grid on;
    
    view(-34,33);
    %view(0,90);
    axis([x_t(4)-0.9, x_t(4)+0.9, x_t(5)-0.9, x_t(5)+0.9, 0, 1.0]);
    
    % plot body
    plot_cube(r_mat, 0.34, 0.2, 0.08,+x_t(4:6)+r_mat*[0;0;0.02], 'black', 2);
    % plot foot
    plot3(fp_w_1(1),fp_w_1(2),fp_w_1(3),'o','linewidth',1.2,'color','b','markersize',6);
    plot3(fp_w_2(1),fp_w_2(2),fp_w_2(3),'o','linewidth',1.2,'color','b','markersize',6);
    plot3(fp_w_3(1),fp_w_3(2),fp_w_3(3),'o','linewidth',1.2,'color','b','markersize',6);
    plot3(fp_w_4(1),fp_w_4(2),fp_w_4(3),'o','linewidth',1.2,'color','b','markersize',6);

    % get leg ik info
    %% front legs, remeber to convert from local to global coord
    hip_g_fr = x_t(4:6) + r_mat * [0.34/2; 0.3/2; 0];
    hip_g_fl = x_t(4:6) + r_mat * [0.34/2; -0.3/2; 0];
    leg_vec_fr = fp_w_1 - hip_g_fr;
    leg_vec_fl = fp_w_2 - hip_g_fl;

    [j_r_fr, j_p_fr] = leg_ik(leg_vec_fr,0.2,0.24);
    [j_r_fl, j_p_fl] = leg_ik(leg_vec_fl,0.2,0.24);

    p_knee_g_fr = hip_g_fr + r_mat*j_p_fr.knee;
    p_knee_g_fl = hip_g_fl + r_mat*j_p_fl.knee;

    p_leg_arr_fr = [x_t(4:6), hip_g_fr, p_knee_g_fr, fp_w_1];
    p_leg_arr_fl = [x_t(4:6), hip_g_fl, p_knee_g_fl, fp_w_2];
    %% back legs
    hip_g_br = x_t(4:6) + r_mat * [-0.34/2; 0.3/2; 0];
    hip_g_bl = x_t(4:6) + r_mat * [-0.34/2; -0.3/2; 0];
    leg_vec_br = fp_w_3 - hip_g_br;
    leg_vec_bl = fp_w_4 - hip_g_bl;

    [j_r_br, j_p_br] = leg_ik(leg_vec_br,0.2,0.24);
    [j_r_bl, j_p_bl] = leg_ik(leg_vec_bl,0.2,0.24);

    p_knee_g_br = hip_g_br + r_mat*j_p_br.knee;
    p_knee_g_bl = hip_g_bl + r_mat*j_p_bl.knee;

    p_leg_arr_br = [x_t(4:6), hip_g_br, p_knee_g_br, fp_w_3];
    p_leg_arr_bl = [x_t(4:6), hip_g_bl, p_knee_g_bl, fp_w_4];

    % leg 
    plot3(p_leg_arr_fr(1,:), p_leg_arr_fr(2,:), p_leg_arr_fr(3,:), 'black','linewidth',2);
    plot3(p_leg_arr_fl(1,:), p_leg_arr_fl(2,:), p_leg_arr_fl(3,:), 'black','linewidth',2);

    plot3(p_leg_arr_br(1,:), p_leg_arr_br(2,:), p_leg_arr_br(3,:), 'black','linewidth',2);
    plot3(p_leg_arr_bl(1,:), p_leg_arr_bl(2,:), p_leg_arr_bl(3,:), 'black','linewidth',2);

    % foot
    plot3(fp_w_1(1),fp_w_1(2),fp_w_1(3),'o','linewidth',2,'color','b','markersize',6);
    plot3(fp_w_2(1),fp_w_2(2),fp_w_2(3),'o','linewidth',2,'color','b','markersize',6);

    plot3(fp_w_3(1),fp_w_3(2),fp_w_3(3),'o','linewidth',2,'color','b','markersize',6);
    plot3(fp_w_4(1),fp_w_4(2),fp_w_4(3),'o','linewidth',2,'color','b','markersize',6);

    % hip
    plot3(hip_g_fr(1),hip_g_fr(2),hip_g_fr(3),'o','linewidth',2,'color','b','markersize',6);
    plot3(hip_g_fl(1),hip_g_fl(2),hip_g_fl(3),'o','linewidth',2,'color','b','markersize',6);

    plot3(hip_g_br(1),hip_g_br(2),hip_g_br(3),'o','linewidth',2,'color','b','markersize',6);
    plot3(hip_g_bl(1),hip_g_bl(2),hip_g_bl(3),'o','linewidth',2,'color','b','markersize',6);

    % knee
    plot3(p_knee_g_fr(1),p_knee_g_fr(2),p_knee_g_fr(3),'o','linewidth',2,'color','b','markersize',4);
    plot3(p_knee_g_fl(1),p_knee_g_fl(2),p_knee_g_fl(3),'o','linewidth',2,'color','b','markersize',4);

    plot3(p_knee_g_br(1),p_knee_g_br(2),p_knee_g_br(3),'o','linewidth',2,'color','b','markersize',4);
    plot3(p_knee_g_bl(1),p_knee_g_bl(2),p_knee_g_bl(3),'o','linewidth',2,'color','b','markersize',4);

    % draw force
    for i=1:4
        x_indx=3*(i-1)+1;
        y_indx=3*(i-1)+2;
        z_indx=3*(i-1)+3;
        
         plot3([foot_pos(x_indx),foot_pos(x_indx)+0.01*feetforce_used(x_indx)],...
            [foot_pos(y_indx),foot_pos(y_indx)+0.01*feetforce_used(y_indx)],...
            [foot_pos(z_indx),foot_pos(z_indx)+0.01*feetforce_used(z_indx)],'linewidth',1.2,'color','r');
        
    end
    
    pause(T/N);
    hold on;
    
      % frame = getframe(gcf);
      % writeVideo(v,frame);
    
end

 % close(v);

end

