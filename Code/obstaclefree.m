function nc = obstaclefree(n2, n1, o,o2,o3)

    A = [n1(1) n1(2)];
    B = [n2(1) n2(2)];
    obs = [(o(1)-2.5) (o(2)-2.5) (o(1)+o(3)+2.5) (o(2)+o(4)+2.5)];
    obs2 = [o2(1) o2(2) o2(1)+o2(3) o2(2)+o2(4)];
    obs3 = [(o3(1)-3.5) (o3(2)-3.5) (o3(1)+o3(3)+3.5) (o3(2)+o3(4)+3.5)];
%     AM=[obs(1) obs(2);obs(3) obs(2);obs(3) obs(4);obs(1) obs(4)];
%     BM=[3 3; 4 3; 4 4; 3 4];
%     P=minksum(AM,BM)
%     [k,av]= convhull(P)
    C1 = [obs(1),obs(2)];
    D1 = [obs(1),obs(4)];
    C2 = [obs(1),obs(2)];
    D2 = [obs(3),obs(2)];
    C3 = [obs(3),obs(4)];
    D3 = [obs(3),obs(2)];
    C4 = [obs(3),obs(4)];
    D4 = [obs(1),obs(4)];
    C1_2 = [obs2(1),obs2(2)];
    D1_2 = [obs2(1),obs2(4)];
    C2_2 = [obs2(1),obs2(2)];
    D2_2 = [obs2(3),obs2(2)];
    C3_2 = [obs2(3),obs2(4)];
    D3_2 = [obs2(3),obs2(2)];
    C4_2 = [obs2(3),obs2(4)];
    D4_2 = [obs2(1),obs2(4)];
    C1_3 = [obs3(1),obs3(2)];
    D1_3 = [obs3(1),obs3(4)];
    C2_3 = [obs3(1),obs3(2)];
    D2_3 = [obs3(3),obs3(2)];
    C3_3 = [obs3(3),obs3(4)];
    D3_3 = [obs3(3),obs3(2)];
    C4_3 = [obs3(3),obs3(4)];
    D4_3 = [obs3(1),obs3(4)];
    
    % Check if path from n1 to n2 intersects any of the four edges of the
    % obstacle
    
    ints1 = ccw(A,C1,D1) ~= ccw(B,C1,D1) && ccw(A,B,C1) ~= ccw(A,B,D1); 
    ints2 = ccw(A,C2,D2) ~= ccw(B,C2,D2) && ccw(A,B,C2) ~= ccw(A,B,D2);
    ints3 = ccw(A,C3,D3) ~= ccw(B,C3,D3) && ccw(A,B,C3) ~= ccw(A,B,D3);
    ints4 = ccw(A,C4,D4) ~= ccw(B,C4,D4) && ccw(A,B,C4) ~= ccw(A,B,D4);
    ints5 = ccw(A,C1_2,D1_2) ~= ccw(B,C1_2,D1_2) && ccw(A,B,C1_2) ~= ccw(A,B,D1_2); 
    ints6 = ccw(A,C2_2,D2_2) ~= ccw(B,C2_2,D2_2) && ccw(A,B,C2_2) ~= ccw(A,B,D2_2);
    ints7 = ccw(A,C3_2,D3_2) ~= ccw(B,C3_2,D3_2) && ccw(A,B,C3_2) ~= ccw(A,B,D3_2);
    ints8 = ccw(A,C4_2,D4_2) ~= ccw(B,C4_2,D4_2) && ccw(A,B,C4_2) ~= ccw(A,B,D4_2);
    ints9 = ccw(A,C1_3,D1_3) ~= ccw(B,C1_3,D1_3) && ccw(A,B,C1_3) ~= ccw(A,B,D1_3); 
    ints10 = ccw(A,C2_3,D2_3) ~= ccw(B,C2_3,D2_3) && ccw(A,B,C2_3) ~= ccw(A,B,D2_3);
    ints11 = ccw(A,C3_3,D3_3) ~= ccw(B,C3_3,D3_3) && ccw(A,B,C3_3) ~= ccw(A,B,D3_3);
    ints12 = ccw(A,C4_3,D4_3) ~= ccw(B,C4_3,D4_3) && ccw(A,B,C4_3) ~= ccw(A,B,D4_3);
    if ints1==0 && ints2==0 && ints3==0 && ints4==0 &&ints5==0 && ints6==0 && ints7==0 && ints8==0&& ints9==0&& ints10==0&& ints11==0&& ints12==0
        nc = 1;
    else
        nc = 0;
    end
end
function val = ccw(A,B,C)
    val = (C(2)-A(2)) * (B(1)-A(1)) > (B(2)-A(2)) * (C(1)-A(1));
end