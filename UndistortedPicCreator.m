%%Obtain undistorted pictures using camera parameters (from Matlab calibration tool box)
%%Import camera parameters
%%The image you want to undistort
I=imread('C:/Users/Dongming/MVS/Data/Image_20190910031250983.bmp');
cameraParams = load('C:/Users/Dongming/Documents/MATLAB/yuguang/cameraParameter.mat');
%%The undistorted image
J = undistortImage(I,cameraParams.cameraParams);
%J=imresize(J,[2*cameraParams.IntrinsicMatrix(3,2),2*cameraParams.IntrinsicMatrix(3,1)]);
figure,
imshow(I,'border','tight','initialmagnification','fit'); 
figure,
imshow(J,'border','tight','initialmagnification','fit'); 


  

