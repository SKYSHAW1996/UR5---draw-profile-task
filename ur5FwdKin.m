% Forward Kinematics Function
function gst  = ur5FwdKin(q)
% syms l1 l2 l3 l4 l5 l6 real;
gst = eye(4);
% (UR5)
l0=0.0892; l1=0.425;l2=0.392;l3=0.1093;l4=0.09475;l5=0.0825;%in m
% l0=8.92;l1=42.5;l2=39.2;l3=10.93;l4=9.475;l5=8.25;%in cm

%offset between S to base_link is l0
% l0=0;
xi1 = [0 0 0 0 0 1]';
xi2 = [0 l0 0 1 0 0]';
xi3 = [0 l0+l1 0 1 0 0]';
xi4 = [0 l0+l1+l2 0 1 0 0]';
xi5 = [0 -l3 0 0 0 1]';
xi6 = [0 l0+l1+l2+l4 0 1 0 0]';
Xi = [xi1 xi2 xi3 xi4 xi5 xi6];
gst0 = [eye(3),[l3+l5 0 l0+l1+l2+l4]';0 0 0 1];

for i =1:6
    xi = Xi(:,i);
    theta = q(i);
    
    v = xi(1:3,1);
    w = xi(4:6,1);
    XI = [SKEW3(w),v;0 0 0 0];
    gi = expm(XI*theta);
    gst = gst*gi;
end
gst = gst*gst0;
Roffset = ROTX(-pi/2)*ROTY(pi/2);
goffset = [Roffset,[0 0 0]';0 0 0 1];
gst = gst*goffset;
% disp(gst);
end

% accepts a 3x1 vector x = [x1; x2; x3]T and returns the corresponding canonical 3x3 skew-symmetric matrix
function R = SKEW3(x)
R = [0 -x(3) x(2);x(3) 0 -x(1);-x(2) x(1) 0];
end
function RX = ROTX(alpha)
RX = [1 0 0;0 cos(alpha) -sin(alpha);0 sin(alpha) cos(alpha)];
end
function RY = ROTY(theta)
RY = [cos(theta) 0 sin(theta);0 1 0;-sin(theta) 0 cos(theta)];
end
function RZ = ROTZ(beta)
RZ = [cos(beta) -sin(beta) 0;sin(beta) cos(beta) 0;0 0 1];
end