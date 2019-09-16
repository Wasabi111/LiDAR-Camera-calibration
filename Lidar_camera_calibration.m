clear all;
close all;
eps=0.00000001;%precision
x0 = [1,0,0,0,1,0,0,0,1,-0.1,0,0];
%fc = 0.025/(5.5*(10^-6));
%0926
%fc = 1782;
cameraParams = load('C:/Users/Dongming/Documents/MATLAB/yuguang/cameraParameter.mat');
%%Import camera parameter first
fcx = cameraParams.cameraParams.IntrinsicMatrix(1,1);
fcy = cameraParams.cameraParams.IntrinsicMatrix(2,2);
fc = (fcx+fcy)/2;
%fc = 1840;
%0925
%fc = 1800;
%%Haikang vision 0925
%Cx=2290/2;
%Cy=1718/2;
%%Haikang vision 0926
Cx =  cameraParams.cameraParams.IntrinsicMatrix(3,1);
Cy =  cameraParams.cameraParams.IntrinsicMatrix(3,2);
%Cx = 1041;
%Cy = 699;
%Cx = 1024;
%Cy = 723;
%Halcon
%Cx=4384/2;
%Cy=6576/2;
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12;
%%(0,5,2) angle x axis -5 y axis 10
%u = [2056,2675,2982,3734,2799,3319,3390];
%v = [3914,3848,4192,3954,3879,4027,4383];
%L_pos = [20,0,99.75;10,0,149.75;0,0,49.75;-15,0,99.75;5,0,124.75;-5,0,74.75;-3,0,37.25];
%%(0,5,0) angle x axis -10 y axis 10
%u = [2048,2675,2992,3750,2806,3333,3419];
%v = [4316,4258,4612,4382,3912,5145,5556];
%L_pos = [20,0,98;10,0,148;0,0,48;-15,0,98;5,10,123;-5,-10,73;-3,-5,35.5];
%?0,5,2) more complex
%u = [2063,2675,2982,3734,2799,3319,3390];
%v = [3452,4003,4192,3954,3879,4694,4383];
%L_pos = [20,10,99.75;10,-5,149.75;0,0,49.75;-15,0,99.75;5,0,124.75;-5,-10,74.75;-3,0,37.25];
%Haikang vision 0925
%u = [634,995,1355,1358,992];
%v = [518,863,505,860,515];
%L_pos = [0.467,0.277,2.262;0.066,-0.106,2.274;-0.328,0.290,2.253;-0.334,-0.091,2.264;0.071,0.291,2.279];
%Haikang vision 0926
%u = [522,610,735,953,886,1123,1121,1125,785,781,817,823,857,1052,1052];
%v = [824,819,817,563,564,501,180,823,821,505,1011,156,192,390,802];
%L_pos = [0.433,-0.232,1.896;0.6,-0.351,3.031;0.42,-0.348,3.05;-0.009,0.076,1.915;0.05,0.076,1.915;-0.189,0.153,2.029;-0.185,0.537,2.024;-0.193,-0.232,2.025;0.18,-0.245,2.061;0.189,0.148,2.074;0.402,-1.028,4.185;0.431,1.148,4.184;0.379,1.185,4.575;-0.072,1.209,8.357;-0.075,-0.955,8.382];
u = [522,610,735,953,886,1123,1121,1125,785,781,817,823,857,1052,1052,1029,1234,1236,1234,1444,1236,1239,1045,1128,1129,1418,948,947,946,948];
v = [824,819,817,563,564,501,180,823,821,505,1011,156,192,390,802,511,609,447,382,444,284,277,299,291,132,561,661,644,618,670];
L_pos = [0.433,-0.232,1.896;0.6,-0.351,3.031;0.42,-0.348,3.05;-0.009,0.076,1.915;0.05,0.076,1.915;-0.189,0.153,2.029;-0.185,0.537,2.024;-0.193,-0.232,2.025;0.18,-0.245,2.061;0.189,0.148,2.074;0.402,-1.028,4.185;0.431,1.148,4.184;0.379,1.185,4.575;-0.072,1.209,8.357;-0.075,-0.955,8.382;...
-0.009,1.433,23.031;-3.725,0.175,29.369;-3.711,3.159,29.093;-3.738,4.053,29.109;-6.245,3.049,27.423;-3.731,6.167,28.863;-4.400,6.166,28.411;-0.110,6.277,31.133;-0.864,6.221,30.658;-0.758,9.234,30.521;-4.204,0.771,19.978;6.346,-3.103,111.451;6.205,-1.710,111.328;6.312,-0.137,111.085;6.388,-3.801,111.281];
[n1,n] = size(u);
%%Gauss-Newton iteration method
for i = 1:1000
   %equations
   f1 = double(subs(Calibration_fun(x0,n,u,v,L_pos,fc,Cx,Cy),{'x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','x11','x12'},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11),x0(12)}));
   %Jacobian
   df = double(subs(Calibration_fun_diff(x0,n,u,v,L_pos,fc,Cx,Cy),{'x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','x11','x12'},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11),x0(12)}));
   kk = (df*df')\(df*f1');
   x = x0 - kk';
   if (abs(x-x0)<eps)
       break;
   end
   x0 = x;
end
   r1 = [x(1),x(2),x(3);x(4),x(5),x(6);x(7),x(8),x(9)];
   t1 = [x(10),x(11),x(12)]';
   t1 = r1'*t1;
disp('Extrinstic parameters: R');
r1
disp('T');
t1
disp('iteration')
i

%%Project LiDAR data to photos
 %B=importdata('C:/Users/Dongming/Desktop/livoxviewer/Livox Viewer 0.5.0/data/test0906.txt');
 %B=importdata('C:/Users/Dongming/Desktop/livoxviewer/Livox Viewer 0.5.0/data/0910morning.txt');
 %B=importdata('C:/Users/Dongming/Desktop/livoxviewer/Livox Viewer 0.5.0/data/0910test1.txt');
  B=importdata('C:/Users/Dongming/Desktop/livoxviewer/Livox Viewer 0.5.0/data/0910afternoon.txt');
 %B=importdata('C:/Users/Dongming/Desktop/livoxviewer/Livox Viewer 0.5.0/data/0906testafternoon.txt');
 A=[];
 A(:,1)=-B(:,3);
 A(:,2)=B(:,2);
 A(:,3)=B(:,1);
 %A=L_pos;
%photo=imread('C:/Users/Dongming/Desktop/letterx_Camera2m_10degreeL_chess.jpg'); 
%photo=imread('C:/Users/Dongming/Desktop/letter_l3.tif');
%photo=imread('C:/Users/Dongming/Desktop/0905afternoon.PNG'); 
%photo=imread('C:/Users/Dongming/Desktop/0906morning2.bmp'); 
%photo=imread('C:/Users/Dongming/Desktop/0906afternoon2.png');
%photo=imread('C:/Users/Dongming/Desktop/0910morning.png');
%photo=imread('C:/Users/Dongming/Desktop/0910test1.png');
photo=imread('C:/Users/Dongming/Desktop/0910afternoon.png');
%file has 3 coordinates as well as reflectivity data,so (n,4)
 X=A(:,1:3);%coordinates
 [n2,n3] = size(X);
 Trans1 = [r1' -t1;0,0,0,1];
 Trans2 = [fcx,0,Cx,0;0,fcy,Cy,0;0,0,1,0];
 C_pos ={};
 for i=1:n2
    X_pos = [X(i,1),X(i,2),X(i,3),1]';
    C_pos{i} = Trans2*Trans1*X_pos;
 end
 C = [C_pos{1},C_pos{2}];
 for i=1:n2
     C(:,i)=C_pos{i};
 end
%C(1,:)=C(1,:)./C(3,:);
%C(2,:)=C(2,:)./C(3,:);
C(1,:) =Cx*2-C(1,:)./C(3,:);
C(2,:) =Cy*2-C(2,:)./C(3,:);
%Plot camera info and corresponding point info

photo_grey = rgb2gray(photo);
figure,imshow(photo),title('Camera');hold on
scatter(C(1,:),C(2,:),1,'filled','cdata',255*(1-C(3,:)/50));
%colorbar('YTickLabel',...
%    {'15m','12m','9m','6m','3m'})
%pcolor(int32(C(1,:)),int32(Cy*2-C(2,:)),photo_grey);
%plot(C(1,:),C(2,:),'LineStyle','none','Marker','.','MarkerSize',3,'MarkerEdgeColor',[C(3,:),C(3,:),C(3,:)],...
%    'MarkerFaceColor',[C(3,:),C(3,:),C(3,:)]);

%plot(C(1,:),C(2,:),'R','LineStyle','none','Marker','o','MarkerSize',6);
%plot(C(1,:)/Cx*300,C(2,:)/Cy*400,'R','LineStyle','none','Marker','o','MarkerSize',6);