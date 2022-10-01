
%% Furbot Forward and reverse parking using RRT* Algorithm.


clearvars
close all

x_max = 40;
y_max = 40;
obstacle = [25,25,4,4];
obstacle1 = [10,30,4,4];
obstacle2 = [18,13,4,4];
EPS = 2;
numNodes = 1400;        



hold on





L=4; %length of the Furbot 
rc=L/tand(30); %Radius off curvature and max steering angle is 30 

park_car = input('Enter 1 for Furbot forward parking , 2 for reverse parking. ');






if park_car==1
    p1=[40 25];
    p2=[40 31];
    p3=[45 31];
    p4=[45 25];
    p5=[45 28];
    p6=[40 28];
    plot(p1(1),p1(2),'*')
    plot(p2(1),p2(2),'*')
    plot(p3(1),p3(2),'*')
    plot(p4(1),p4(2),'*')
    plot([p1(1) p4(1)],[p1(2) p4(2)],'Color', 'b', 'LineWidth', 1)
    plot([p2(1) p3(1)],[p2(2) p3(2)],'Color', 'b', 'LineWidth', 1)
    plot([p3(1) p4(1)],[p3(2) p4(2)],'Color', 'b', 'LineWidth', 1)
    plot([p6(1) p5(1)],[p6(2) p5(2)],'Color', 'b', 'LineWidth', 1)
    park=[42 28];
    x2=park(1);
    y2=park(2)+rc;
    y2f=park(2)-rc;
    angleinital=180;
    anglefinal=270;
    center=[x2;y2];
    center_forward=[x2;y2f];
    angleinital_forward=90;
    anglefinal_forward=180;
    theta= linspace(angleinital,anglefinal);
    theta_forward= linspace(angleinital_forward,anglefinal_forward);
    xcircle=center(1)+rc*cosd(theta);
    ycircle=center(2)+rc*sind(theta);
    xcircle_forward=center_forward(1)+rc*cosd(theta_forward);
    ycircle_forward=center_forward(2)+rc*sind(theta_forward);
%     xcircle_forward=[4 xcircle_forward];
%     ycircle_forward=[33 ycircle_forward];
    plot(xcircle_forward,ycircle_forward, '.');
    plot(xcircle,ycircle, '.');
    parking_path_x=xcircle_forward;
    parking_path_y=ycircle_forward;

    q_goal.coord =[xcircle_forward(end) ycircle_forward(end)];

else if park_car==2 
    p1=[40 25];
    p2=[40 31];
    p3=[45 31];
    p4=[45 25];
    p5=[45 28];
    p6=[40 28];
    plot(p1(1),p1(2),'*')
    plot(p2(1),p2(2),'*')
    plot(p3(1),p3(2),'*')
    plot(p4(1),p4(2),'*')
    plot([p1(1) p4(1)],[p1(2) p4(2)],'Color', 'b', 'LineWidth', 1)
    plot([p2(1) p3(1)],[p2(2) p3(2)],'Color', 'b', 'LineWidth', 1)
    plot([p3(1) p4(1)],[p3(2) p4(2)],'Color', 'b', 'LineWidth', 1)
    plot([p6(1) p5(1)],[p6(2) p5(2)],'Color', 'b', 'LineWidth', 1)
    
        park=[42 28];
    x2=park(1);
    y2=park(2)+rc;
    y2f=park(2)-rc;
    angleinital=180;
    anglefinal=270;
    center=[x2;y2];
    center_forward=[x2;y2f];
    angleinital_forward=90;
    anglefinal_forward=180;
    theta= linspace(angleinital,anglefinal);
    theta_forward= linspace(angleinital_forward,anglefinal_forward);
    xcircle=center(1)+rc*cosd(theta);
    ycircle=center(2)+rc*sind(theta);
    xcircle=flip(xcircle);
    ycircle=flip(ycircle);
    xcircle_forward=center_forward(1)+rc*cosd(theta_forward);
    ycircle_forward=center_forward(2)+rc*sind(theta_forward);
    xcircle=[44 xcircle];
    ycircle=[28 ycircle];
    plot(xcircle_forward,ycircle_forward, '.');
    plot(xcircle,ycircle, '.');
    q_goal.coord =[xcircle(end) ycircle(end)];
    parking_path_x=xcircle;
    parking_path_y=ycircle;
    else if park_car==3
    p7=[19 40];
    p8=[19 42.5];
    p9=[27 40];
    p10=[27 42.5];
    plot(p7(1),p7(2),'*')
    plot(p8(1),p8(2),'*')
    plot(p9(1),p9(2),'*')
    plot(p10(1),p10(2),'*')
    plot([p7(1) p8(1)],[p7(2) p8(2)],'Color', 'b', 'LineWidth', 1)
    plot([p8(1) p10(1)],[p8(2) p10(2)],'Color', 'b', 'LineWidth', 1)
    plot([p9(1) p10(1)],[p9(2) p10(2)],'Color', 'b', 'LineWidth', 1)
    
    
    %first circle for parallel parking
    
    parkparallel=[24 41.25];
    plot(parkparallel(1),parkparallel(2),'*')
    P_C1=parkparallel(1);
    P_C2=parkparallel(2)-rc;
    plot(P_C1,P_C2,'*')
    P_angle=90;
    P_anglefinal=135;
    center_parallel_1=[P_C1,P_C2];
    theta_parallel_1= linspace(P_angle,P_anglefinal);
    xcircle_P_1=center_parallel_1(1)+rc*cosd(theta_parallel_1);
    ycircle_P_1=center_parallel_1(2)+rc*sind(theta_parallel_1);
    xcircle_P_1(2:end+1)=xcircle_P_1;
    ycircle_P_1(2:end+1)=ycircle_P_1;
    ycircle_P_1(1)=ycircle_P_1(2)
    xcircle_P_1(1)=xcircle_P_1(2)+2

    % plot(xcircle_P_1,ycircle_P_2,'Color', 'b', 'LineWidth', 2);
       
    
    %Second Cricle for parking parallel
      
    P2_C1=14.2;
    P2_C2=44.15;


    P_angle_2=270;
    P_anglefinal_2=315;
    plot(P2_C1,P2_C2,'*')
    center_parallel_2=[P2_C1,P2_C2];
    theta_parallel_2= linspace(P_angle_2,P_anglefinal_2);
    xcircle_P2_1=center_parallel_2(1)+rc*cosd(theta_parallel_2);
    ycircle_P2_2=center_parallel_2(2)+rc*sind(theta_parallel_2);
    % plot(xcircle_P2_1,ycircle_P2_2,'Color', 'b', 'LineWidth', 2);
    xcircle_P2_1=flip(xcircle_P2_1);
    ycircle_P2_2=flip(ycircle_P2_2);
    Parallel_point_x=[xcircle_P_1 xcircle_P2_1 ];

    Parallel_point_y=[ycircle_P_1 ycircle_P2_2 ];


    plot(Parallel_point_x,Parallel_point_y, '.');
    Parallel_point_x(end)=Parallel_point_x(end)-4;
    Parallel_point_y(end)=Parallel_point_y(end);
    q_goal.coord = [Parallel_point_x(end) Parallel_point_y(end)];
    parking_path_x= Parallel_point_x;
    parking_path_y=Parallel_point_y;
        end
    end
end

    
q_start.coord = [1 1];
q_start.cost = 0;
q_start.parent = 0;
% q_goal.coord = [2 2];
q_goal.cost = 0;

nodes(1) = q_start;
figure(1)
axis([-10 50 -10 50])
rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
rectangle('Position',obstacle2,'FaceColor','b')
% circle(12,32,2,'color','black')
for i = 1:1:numNodes
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
    plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])
%     drawvehicle(2.3,3,0);

    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx);
    
    q_new.coord = steer(q_rand, q_near.coord, val, EPS);
    if obstaclefree(q_rand, q_near.coord, obstacle,obstacle1,obstacle2)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
        drawnow
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        % Within a radius of r, find all existing nodes
        q_nearest = [];
        r =10;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if obstaclefree(nodes(j).coord, q_new.coord, obstacle,obstacle1,obstacle2) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;
        
        % Iterate through all nearest neighbors to find alternate lower
        % cost paths
        
        for k = 1:1:length(q_nearest)
            if obstaclefree(q_nearest(k).coord, q_new.coord, obstacle,obstacle1,obstacle2) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
%                 line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'w');                
                hold on
            end
        end
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append to nodes
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
count = 1;

while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    A(1,count)=q_end.coord(1);
    A(2,count)=q_end.coord(2);
    count = count+1

   
    
    hold on
    car_phi=atan2((q_end.coord(2)-nodes(start).coord(2)),(q_end.coord(1)-nodes(start).coord(1)));
    dis=sqrt((q_end.coord(1)-nodes(start).coord(1))^2+(q_end.coord(2)-nodes(start).coord(2))^2);
    x1=linspace(q_end.coord(1),nodes(start).coord(1),abs(dis));
    y1=linspace(q_end.coord(2),nodes(start).coord(2),abs(dis));
    
    pause(0.1)

    
    q_end = nodes(start);
    
    
    


end



A(1,count)=q_start.coord(1);
A(2,count)=q_start.coord(2);
mn=flip(A(1,:));
mm=flip(A(2,:));

mybez(mn,mm,count,obstacle,obstacle2,parking_path_x,parking_path_y,park_car);

