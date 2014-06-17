%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                      DEPTH DETECTION ALGORITHM                         %
%                     by - Hussain K. Manasawala                          %  
%                          MTech, Electronic Systems,                     %
%                          IIT Bombay.                                    %
%                                                                         %
% This program performs depth detection by computing the Disparity between%
% two stereo images.                                                      %
% From this Depth Map the Position of various objects is computed.        %                                                               %  
%                                                                         %  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%% Code Start %%%%%%%%%%%%%%%%%%%%%%%%
clear all
clc;

%% Capture pair of stereo images

%-- Create video object camera 2
vid2 = videoinput('winvideo', 2);

%-- Set the properties of the video object
set(vid2, 'FramesPerTrigger', Inf);
set(vid2, 'ReturnedColorspace', 'rgb')
vid2.FrameGrabInterval = 5;

%-- Start the video aquisition here
start(vid2)

%-- Set a loop that stop after 50 frames of aquisition
while(vid2.FramesAcquired<=10)    
    %-- Get the snapshot of the current frame
    imgR = getsnapshot(vid2);
end

%-- Create video object camera 3
vid3 = videoinput('winvideo', 3);

%-- Set the properties of the video object
set(vid3, 'FramesPerTrigger', Inf);
set(vid3, 'ReturnedColorspace', 'rgb')
vid3.FrameGrabInterval = 5;

%-- Start the video aquisition here
start(vid3)

%-- Set a loop that stop after 50 frames of aquisition
while(vid3.FramesAcquired<=10)  
    %-- Get the snapshot of the current frame
    imgL = getsnapshot(vid3);
end

figure(1);
subplot(1,2,1), imshow(imgL);
subplot(1,2,2), imshow(imgR);

%-- Flush the video objects
stop(vid2);
delete(vid2);
clear vid2;
stop(vid3);
delete(vid3);
clear vid3;


%% Call to the stereo disparity extraction code

%-- Set the parameters
mins = 3;  
maxs = 30; 
hs = 10;   
hr = 5;    
M = 50;    
%-- here's the main call
[d p s l] = husstereo(imgR,imgL, hs,hr,M,mins, maxs);

y=ones(size(d));
%y=255*y;
%d=y-d;

%-- Display stuff
figure(2);
subplot(2,2,1), imshow(imgL);
subplot(2,2,2), imshow(imgR);
subplot(2,2,3), imshow(s,[]);
subplot(2,2,4), imshow(p,[])

figure(3);
imshow(d,[]);


 %% Create the Position matrix from Depth map

[height width] = size(d);
data = zeros(height, width);

%-- Thresholding of the depth map
blk=min(max(d));
for i=1:height
    for j=1:width

        if( d(i,j)>blk )
            data(i,j) = 1;
        end
    end
end

%-- Remove all those pixels less than 300px
diff_im = bwareaopen(data,500);

%-- Label all the connected components in the image.
bw = bwlabel(diff_im, 8);

%-- Here we do the image blob analysis.
%-- We get a set of properties for each labeled region.
stats = regionprops(bw, 'basic');

%-- Display the image
figure(4);
imshow(data)

%-- This is a loop to bound the near objects in a rectangular box.
for object = 1:length(stats)
    bb = stats(object).BoundingBox;
    area = stats(object).Area;
    if area > 1000
        x = bb(1);
        y = bb(2);
        w = bb(3);
        h = bb(4);
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
    end
end  

%-- Save stuff
imwrite(imgR,'stereo8R.jpg');
imwrite(imgL,'stereo8L.jpg');
imwrite(uint8(d),'disp8.jpg');
imwrite(data,'depth8.jpg');

%-- Simple Optics
obh=10.0;
y=height-(y+h);
obd=(y*obh)/h;

if((x+w/2)>width/2)
    x=(x+w)-(width/2);
    right=1;
else
    x=(width/2)-x;
    right=0;
end

obw=(x*obh)/h;

fwd=obd;
rgt=obw;

%-- Creating the motion control vector
Pos=['w'; 'w'; 's'; 's'; 's'; 's'; 's'; 's'; 's'; 's'; 's'; 's'] 

stp=3;
while((fwd/5.0)>=1.0)
    Pos(stp)='w';
    fwd=fwd-5.0
    stp=stp+1;    
end    

if(right==1)
    Pos(stp)='a';
else
    Pos(stp)='d';
end

stp=stp+1;
while((rgt/5.0)>=1.0)
    Pos(stp)='w';
    rgt=rgt-5.0
    stp=stp+1;    
end    
Pos(stp)='w';
stp=stp+1;

if(right==1)
    Pos(stp)='d'
else
    Pos(stp)='a'
end


%% Transmitting the Navigation commands Serailly to FB5

%-- Create object for the serial port
s = serial('COM2');

%-- Set parameters for the serial port
set(s,'BaudRate',9600);
fopen(s);

%-- Transmit the motion commands in the desired sequence
for i=1:10
    out=Pos(i);
    fprintf(s,out);
    in = fscanf(s)
    while (in ~= out )
        in = fscanf(s);
    end
end

%-- Flush the serial object
fclose(s);
delete(s);
clear s;

%%%%%%%%%%%%%%%%%%%%%%%% Code Ends %%%%%%%%%%%%%%%%%%%%%%%%