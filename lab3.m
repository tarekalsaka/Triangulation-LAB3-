clc
clear
close all
%============= defind a steroe camera without noise=============== 

T1 = transl(-1, 0, 0) * troty(22);
cam1 = CentralCamera('name', 'camera 1', 'default', ...
'focal', 0.002, 'pose', T1)

T2 = transl(1, 0,0)*troty(-22);
cam2 = CentralCamera('name', 'camera 2', 'default', ...
'focal', 0.002, 'pose', T2);


% ===========define a steroe camera with 5 different noise===================== 
j=1 
for i=0.2:0.2:1
    
    cam1_noise(j) = CentralCamera('name', 'camera 1', 'default','focal', 0.002, 'pose', T1,'noise',i);
    cam2_noise(j) = CentralCamera('name', 'camera 2', 'default','focal', 0.002, 'pose', T2,'noise',i);
    j=j+1;
end

j= j-1
%}

% ===============point 3D  12 point======================== 
Point_3D = homtrans( transl(-1, -1, 2), 2*rand(3,12) );

%============== point 2D projection without noise================
projected1= cam1.project(Point_3D);
projected2= cam2.project(Point_3D);

%===========projected with noise for camera 1====================== 
projected_point1=cell(1,5);
for k = 1:j
    projected_point1{k}= cam1_noise(k).project(Point_3D);
end 
%========projected with noise for camera 2=========================== 
projected_point2=cell(1,5);
for k = 1:j
    projected_point2{k}= cam2_noise(k).project(Point_3D);
end 


%=============== plot camera with 3D_point========================== 
axis([-3 3 -3 3 0 5])
cam1.plot_camera('color', 'b', 'label')
cam2.plot_camera('color', 'r', 'label')
plot_sphere(Point_3D, 0.03, 'b');

===========% cmmear matrix=================== 
C1= cam1.C;
C2=cam2.C;

%{
C11_noise= cam1_noise(1).C;
C12_noise= cam1_noise(2).C;
C13_noise= cam1_noise(3).C;
C14_noise= cam1_noise(4).C;
C15_noise= cam1_noise(5).C;

C21_noise= cam2_noise(1).C;
C22_noise= cam2_noise(2).C;
C23_noise= cam2_noise(3).C;
C24_noise= cam2_noise(4).C;
C25_noise= cam2_noise(5).C;
%}

%================= fundemental matrix================================= 
F = fmatrix(projected1, projected2);
F_n1= fmatrix(projected_point1{1},projected_point2{1});
F_n2=fmatrix(projected_point1{2},projected_point2{2});
F_n3=fmatrix(projected_point1{3},projected_point2{3});
F_n4=fmatrix(projected_point1{4},projected_point2{4});
F_n5=fmatrix(projected_point1{5},projected_point2{5});

%========plot epipolar line and point in image plane without noise for camera 1 
cam1.plot(Point_3D)
cam1.hold
cam1.plot_epiline(F', projected2, 'r');
cam1.hold
e1 = cam1.plot( cam2.centre, 'Marker', 'd', 'MarkerFaceColor', 'k');

%======= plot epipolar line and point in image plane without noise for camera 2
cam2.plot(Point_3D)
cam2.hold
cam2.plot_epiline(F, projected1, 'r');
cam2.hold
e2 = cam2.plot(cam1.centre, 'Marker', 'd', 'MarkerFaceColor', 'k');




%======= plot epipolar line and point in image plane with noise for camera c 
c=5
cam1_noise(c).plot(Point_3D)
cam1_noise(c).hold
cam1_noise(c).plot_epiline(F_n1',projected_point2{c}, 'r');
cam1_noise(c).hold
e1_n = cam1_noise(c).plot( cam2_noise(c).centre, 'Marker', 'd', 'MarkerFaceColor', 'k');

cam2_noise(c).plot(Point_3D)
cam2_noise(c).hold
cam2_noise(c).plot_epiline(F_n1, projected_point1{c}, 'r');
cam2_noise(c).hold
e2_n = cam2_noise(c).plot( cam1_noise(c).centre, 'Marker', 'd', 'MarkerFaceColor', 'k');

%===============================% triangulate ideal camera========================== 
pts3D = stereoReconsPts(C1, C2 , projected1 , projected2);

%pts3D_n_correct = stereoReconsPts(C1, C2 , projected_point1{1} ,projected_point2{1} ,-1,'poly',F_n1);

%triangulate using poly method with different camera noise 
pts3D_noise_poly1=stereoReconsPts(C1, C2, projected_point1{1}, projected_point2{1},-1,...
    'poly',F_n1);

pts3D_noise_poly2=stereoReconsPts(C1, C2, projected_point1{2}, projected_point2{2},-1,...
    'poly',F_n2);
pts3D_noise_poly3=stereoReconsPts(C1, C2, projected_point1{3}, projected_point2{3},-1,...
    'poly',F_n3);
pts3D_noise_poly4=stereoReconsPts(C1, C2, projected_point1{4}, projected_point2{4},-1,...
    'poly',F_n4);
pts3D_noise_poly5=stereoReconsPts(C1, C2, projected_point1{5}, projected_point2{5},-1,...
    'poly',F_n5);
%{
pts3D_noise_poly1=stereoReconsPts(C1, C2, projected_point1{1}, projected_point2{1},-1);

pts3D_noise_poly2=stereoReconsPts(C1, C2, projected_point1{2}, projected_point2{2},-1);
pts3D_noise_poly3=stereoReconsPts(C1, C2, projected_point1{3}, projected_point2{3},-1);
pts3D_noise_poly4=stereoReconsPts(C1, C2, projected_point1{4}, projected_point2{4},-1);
pts3D_noise_poly5=stereoReconsPts(C1, C2, projected_point1{5}, projected_point2{5},-1);
%}



%{
cam1_noise(4).plot(projected_point1{4})
cam1_noise(4).hold
cam1_noise(4).plot_epiline(F1_noise', projected_point2{4}, 'r');
cam1_noise(4).hold
e1 = cam1_noise(4).plot( cam2_noise(4).centre, 'Marker', 'd', 'MarkerFaceColor', 'k');

cam2_noise(4).plot(projected_point2{4})
title('camera 2')
cam2_noise(4).hold
cam2_noise(4).plot_epiline(F1_noise, projected_point1{4}, 'r');
cam2_noise(4).hold
e2 = cam2_noise(4).plot( cam1.centre, 'Marker', 'd', 'MarkerFaceColor', 'k');
%}
    
%{
Error=zeros(1,5)
mean_3Dpoint  = mean(Point_3D);
Error(1)= median(mean((pts3D_noise_poly1)- mean_3Dpoint).^2);
Error(2)= median(mean((pts3D_noise_poly2)- mean_3Dpoint).^2);
Error(3)= median(mean((pts3D_noise_poly3)- mean_3Dpoint).^2);
Error(4)= median(mean((pts3D_noise_poly4)- mean_3Dpoint).^2);
Error(5)= median(mean((pts3D_noise_poly5)- mean_3Dpoint).^2);
%}
%============find the norm between the original point and triangulated one ====    
Error=zeros(1,12); 
Errorfor1=zeros(1,12);    
Errorfor2=zeros(1,12) ;   
Errorfor3=zeros(1,12)  ;  
Errorfor4=zeros(1,12); 
Errorfor5=zeros(1,12);

p=1
for i=1: 12
Error(i)= norm(Point_3D(:,i) - pts3D(:,i),p)

Errorfor1(i)=norm(Point_3D(:,i) - pts3D_noise_poly1(:,i),p)
Errorfor2(i)=norm(Point_3D(:,i) - pts3D_noise_poly2(:,i),p)

Errorfor3(i)=norm(Point_3D(:,i) - pts3D_noise_poly3(:,i),p)
Errorfor4(i)=norm(Point_3D(:,i) - pts3D_noise_poly4(:,i),p)

Errorfor5(i)=norm(Point_3D(:,i) - pts3D_noise_poly5(:,i),p)

end

norm(Error)
E=zeros(1,5)
E(1)=norm(Error,p)
E(2)=norm(Errorfor1,p)
E(3)=norm(Errorfor2,p)
E(4)=norm(Errorfor3,p)
E(5)=norm(Errorfor4,p)
E(6)=norm(Errorfor5,p)

x = [0 , 0.2 , 0.4 , 0.6, 0.8 ,1];

plot(x,E )
title("Optimal methods, L1 norm")
xlabel('noise')
ylabel('norm')


    
