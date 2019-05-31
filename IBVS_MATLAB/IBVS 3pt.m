clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define the world-points for the landmarks in the VisualServo function %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Rotation is considered about y-axis
% Basic camera parameters
cam=CentralCamera('focal',0.035,'pixel',1.8e-04,'resolution',[640,480]);

% Defining a landmark coordinates for three LED point w.r.t world frame
% change the co-ordinates in the VisualServo.m
p1=[-0.1 0 0.5]';       
p2=[0 0 0.5]';
p3=[0.1 0 0.5]';

% Combined 3x3 co-ordinate position matrix for the landmark
P=[p1 p2 p3];

% Camera projection for the world points
p01=cam.project(p1);
p02=cam.project(p2);
p03=cam.project(p3);

p0=[p01 p02 p03];
% s1=plot_sphere(p1,0.03,'g');
% s2=plot_sphere(p2,0.03,'g');
% s3=plot_sphere(p3,0.03,'b');

% cam.plot(p0);

%Initial pose of the camera
 Tc0=SE3(-0.1,-0.1,0.2)*SE3.Ry(pi/10);    % Change the value of Tc0 in VisualServo.m
% Tc0=SE3(-0.1,0,0.2);

% Displacing the camera along the -ve X-axis by 0.5 unit
px1=cam.project(p1,'pose',Tc0);
px2=cam.project(p2,'pose',Tc0);
px3=cam.project(p3,'pose',Tc0);

px=[px1 px2 px3]; 
% cam.plot(px);

% To check the gap difference in-between points for their corresponding
% projection
Shift_1=(px1-px2);  
Shift_2=(px2-px3);

% pStar=bsxfun(@plus,[-200 0 200;0 0 0],cam.pp');
% Center point and 3-point matrix
p_center=[320 320 320;240 240 240];
p_c0=[320 240]';

% Desired points for first motion
pDes=[(px1-(px2-p_c0)) p_c0 (px3-(px2-p_c0))];

% p_cam=cam.plot(px,'pose',Tc0);
%  
e=round(pDes-px);

J1=cam.visjac_p(p1,1);
J2=cam.visjac_p(p2,1);
J3=cam.visjac_p(p3,1);
%   
J_exp=[J1;J2;J3];
% % % 
% lambda=0.5;      %%%%0.5
% v=lambda*pinv(J)*e(:);
%  
%   Tc0=Tc0.*delta2tr(v);

ibvs=IBVS(cam,'pose0',Tc0,'pstar',pDes);
%  
%  ibvs2=IBVS(cam,'pose0',pStar1,'pstar',pStar2);
%  
%  ibvs3=IBVS(cam,'pose0',pStar2,'pstar',pStar3);
 
ibvs.run();
ibvs.vel_p()
% ibvs.plot_p()
ibvs.plot_all()

