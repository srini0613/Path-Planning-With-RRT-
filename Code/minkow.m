A=[1 1; 2 1; 2 2; 1 2]; B=[3 3; 4 3; 4 4; 3 4];
    [S,D]=minksum(A,B);
    plot(A(:,1),A(:,2),'*',B(:,1),B(:,2),'s',S(:,1),S(:,2),'d')
    axis([0 7 0 7])