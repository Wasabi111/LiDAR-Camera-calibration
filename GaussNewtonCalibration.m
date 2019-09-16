clear all;
close all;
eps=0.0000000001;%precision
x0 = [1,0,0,0,1,0,0,0,1,0,5,2];
syms x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12;

for i = 1:1000
   %equations
   f1 = double(subs(Calibration_fun_left(x0),{'x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','x11','x12'},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11),x0(12)}));
   %Jacobian
   df = double(subs(Calibration_fun_left_diff(x0),{'x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','x11','x12'},{x0(1),x0(2),x0(3),x0(4),x0(5),x0(6),x0(7),x0(8),x0(9),x0(10),x0(11),x0(12)}));
   kk = (df*df')\(df*f1');
   x = x0 - kk';
   if (abs(x-x0)<eps)
       break;
   end
   x0 = x;
end
   rl = [x(1),x(2),x(3);x(4),x(5),x(6);x(7),x(8),x(9)];
   tl = [x(10),x(11),x(12)];
disp('left r');
rl
disp('left t');
tl
disp('iteration')
i

x1 = [1,0,0,0,1,0,0,0,1,2,5,2];


for i = 1:1000
   %equations
   f = double(subs(Calibration_fun_right(x1),{'x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','x11','x12'},{x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),x1(7),x1(8),x1(9),x1(10),x1(11),x1(12)}));
   %Jacobian
   df = double(subs(Calibration_fun_right_diff(x1),{'x1','x2','x3','x4','x5','x6','x7','x8','x9','x10','x11','x12'},{x1(1),x1(2),x1(3),x1(4),x1(5),x1(6),x1(7),x1(8),x1(9),x1(10),x1(11),x1(12)}));
   kk = (df*df')\(df*f');
   x = x1 - kk';
   if (abs(x-x1)<eps)
       break;
   end
   x1 = x;
end
   rr = [x(1),x(2),x(3);x(4),x(5),x(6);x(7),x(8),x(9)];
   tr = [x(10),x(11),x(12)];
disp('right r');
rr
disp('right t');
tr
disp('iteration')
i
disp('Relative Translate Vector');
Trel = rl'*(tr'-tl')
disp('Relative Rotation Matrix');
Rrel = rl'*rr
%Camera transformation test
Pos1 = [0;5;2];
disp('Camera 2 position');
Pos2 = Pos1+Trel
%*************************TEST FOR WORLD TO CAMERA PIXEL******************
%Real World to image:
%Re_pos=[10,0,149.75,1]';
%Re_pos2 = [15,0,99.75,1]';
%Re_pos3 = [-3,0,37.25,1]';
%TranRight = [rr tr';0,0,0,1];
%fc=25;
%k1=5.5*(10^-3);
%Tran2 = [fc 0 0 0;0 fc 0 0;0 0 1 0];
%Tran3 = [1/k1 0 4384/2;0 1/k1 6576/2;0 0 1];

%Tran4 = [fc/k1 0 4384/2 0;0 fc/k1 6576/2 0;0 0 1 0];
%disp('10,0,150->1097,4276');
%New_pos_Right1 = Tran3*(Tran2*(TranRight*Re_pos))/149.75
%New_pos_Right_1 = Tran4*TranRight*Re_pos/149.75
%disp('15,0,100->438,4390');
%New_pos_Right2 = Tran3*(Tran2*(TranRight*Re_pos2))/99.75
%disp('-3,0,37.5->1888,4737');
%New_pos_Right3 = Tran3*(Tran2*(TranRight*Re_pos3))/37.25
%TranLeft = [rl tl';0,0,0,1];
%TranLeft = [cosd(10) 0 sind(10) 0;0 cosd(10) sind(10) 5;-sind(10) -sind(10) cosd(10)^2 0;0 0 0 1];
%disp('10,0,150->2675,4259');
%New_pos_Left1 = Tran3*(Tran2*(TranLeft*Re_pos))/149.75
%disp('15,0,100->2061,4307');
%New_pos_Left2 = Tran3*(Tran2*(TranLeft*Re_pos2))/99.75
%disp('-3,0,37.5->3377,4785');
%New_pos_Left3 = Tran3*(Tran2*(TranLeft*Re_pos3))/37.25