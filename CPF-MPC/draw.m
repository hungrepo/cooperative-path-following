figure
scatter(getcolumn(x_path1(:,1:2),1),getcolumn(x_path1(:,1:2),2));
hold on;
scatter(getcolumn(x_path2(:,1:2),1),getcolumn(x_path2(:,1:2),2));
scatter(getcolumn(x_path3(:,1:2),1),getcolumn(x_path3(:,1:2),2));
scatter(getcolumn(x_robot1(:,1:2),1),getcolumn(x_robot1(:,1:2),2));
scatter(getcolumn(x_robot2(:,1:2),1),getcolumn(x_robot2(:,1:2),2));
scatter(getcolumn(x_robot3(:,1:2),1),getcolumn(x_robot3(:,1:2),2));

figure

plot(x_path1(:,4),x_path1(:,3))
hold on
plot(x_path1(:,4),x_path2(:,3))
plot(x_path1(:,4),x_path3(:,3))

figure
plot(V_mpc1); hold on;
plot(V_mpc2);
plot(V_mpc3);

figure 

plot(u_mpc1(end,:))
hold on
plot(u_mpc2(end,:))
hold on
plot(u_mpc3(end,:))