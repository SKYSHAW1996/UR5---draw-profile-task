function professor_bw_small = profile(m)
% m='0.jpg';
professor = imread(m);
deltaT=0.5;
[professor_bw,T,index]=ISODATA(professor,deltaT);
professor_bw_small = double(imresize(professor_bw,[100,100]));
% figure;imshow(professor_bw_small,[]);
end

function [BW,T,index] = ISODATA(image,delta)
index=1;
error=10;
avergT=mean2(image);
while error > delta
R1=find(image<=avergT);
R2=find(image>avergT);
t1=mean2(image(R1));t2=mean2(image(R2));
newT=(t1+t2)/2;
error=abs(avergT-newT);
avergT=newT;
index = index+1;
end
T = avergT;
BW=im2bw(image,T/255);
end