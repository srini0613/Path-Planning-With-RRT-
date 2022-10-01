function mybez(x,y,n,obstacle,obstacle2,xcircle,ycircle,park)



figure(2)

    
    P(1,:) = x %x coordinate
    P(2,:) = y %y coordinate
    P(3,:) = 0;                                 %all points in xy plane
    
count = 1;
div = 100; %number of segments of the curve (Increase this value to obtain a
          %smoother curve
for u = 0:(1/div):1
    sum = [0 0 0]';
    for i = 1:n
        B = nchoosek(n,i-1)*(u^(i-1))*((1-u)^(n-i+1)) %B is the Bernstein polynomial value
        sum = sum + B*P(:,i);
    end
    B = nchoosek(n,n)*(u^(n));
    sum = sum + B*P(:,n);
    A(:,count) = sum; %the matrix containing the points of curve as column vectors. 
    count = count+1;  % count is the index of the points on the curve.
end
for j = 1:n %plots the points
    plot(P(1,j),P(2,j),'-');
    hold on;
end
p1 = P(:,1);
%draws the characteristic polygon describing the bezier curve
for l = 1:n-1
    p2 = P(:,l+1)';
    lineplot(p1,p2); %function the plots a line between two points.
    
    p1 = p2;
    
end
%plotting the curve
rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
rectangle('Position',obstacle2,'FaceColor',[0 .5 .5])


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





yaw_park = zeros(length(xcircle), 1);
for i = 2:length(xcircle)-1
    x_forward_park = xcircle(i+1);
    x_backward_park = xcircle(i-1);
    y_forward_park = ycircle(i+1);
    y_backward_park = ycircle(i-1);
    yaw_park(i) = atan2(y_forward_park-y_backward_park, x_forward_park-x_backward_park);
end
yaw_park(1) = yaw_park(2);
yaw_park(end) = yaw_park(end-1);
xcircle=flip(xcircle);
ycircle=flip(ycircle);
yaw_park=flip(yaw_park);
pathslot = [xcircle', ycircle', yaw_park ];
x = A(1,:);
y = A(2,:);
yaw = zeros(length(x), 1);
for i = 2:length(x)-1
    x_forward = x(i+1);
    x_backward = x(i-1);
    y_forward = y(i+1);
    y_backward = y(i-1);
    yaw(i) = atan2(y_forward-y_backward, x_forward-x_backward);
end
yaw(1) = yaw(2);
yaw(end) = yaw(end-1);
path = [x', y', yaw];

% save('path', 'path')
axis([-10 50 -10 50])
rectangle('Position',obstacle,'FaceColor',[0 .5 .5])

plot(x,y);
if park==1
    path=[path; pathslot]
    save('path', 'path')
    
else 
    if park==2
        save('path', 'path')
        save('Rparking','pathslot')
    else
        if park==3
            path=[path; pathslot]
            save('path', 'path')
%             save('Rparking','pathslot')
        end
    end
end




plot(xcircle,ycircle,'.');

axis equal;

end
function h=vehicle(xa,ya)

for j = 1:1:length(xa)
    thetha=atan2((xa(j+1)-xa(j)),(ya(j+1)-ya(j)));
        

    
    drawvehicle(xa(j),ya(j),pi/2-thetha)
    pause(0.1)
end
end
%function definitions
function [] = lineplot(A,B)
x = [A(1) B(1)]; 
y = [A(2) B(2)];
% thetha=atan2(x,y)
plot(x,y,'r'); %a dashed red  line

end 

