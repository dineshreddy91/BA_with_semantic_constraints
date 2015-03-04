clear all;
close all;
dim_num = 3;
  
  %% random points generator on a cube
  n = 1000;
  seed = 123456789;
  [x] = uniform_on_cube ( dim_num, n, seed );
  
  %% random intilization of the trajectories  here its moving in z direction 
  rotation_matrices(:,:,1)=[1 0 0 1;0 1 0 0; 0 0 1 20;0 0 0 1]; % z=0
  rotation_matrices(:,:,2)=[1 0 0 1;0 1 0 0; 0 0 1 19;0 0 0 1]; % z=1
  rotation_matrices(:,:,3)=[1 0 0 1;0 1 0 0; 0 0 1 18;0 0 0 1]; % z=2
  rotation_matrices(:,:,4)=[1 0 0 1;0 1 0 0; 0 0 1 17;0 0 0 1]; % z=3
  rotation_matrices(:,:,5)=[1 0 0 1;0 1 0 0; 0 0 1 16;0 0 0 1]; % z=4
  rotation_matrices(:,:,6)=[1 0 0 1;0 1 0 0; 0 0 1 15;0 0 0 1]; % z=5
  rotation_matrices(:,:,7)=[1 0 0 1;0 1 0 0; 0 0 1 14;0 0 0 1]; % z=6
  rotation_matrices(:,:,8)=[1 0 0 1;0 1 0 0; 0 0 1 13;0 0 0 1]; % z=7
  rotation_matrices(:,:,9)=[1 0 0 1;0 1 0 0; 0 0 1 12;0 0 0 1]; % z=8
  rotation_matrices(:,:,10)=[1 0 0 1;0 1 0 0; 0 0 1 11;0 0 0 1]; % z=9
  
  %% view of the 3d box
  %scatter3(x(1,:),x(2,:),x(3,:));
  
  %% loop for num of movements
  number_of_views=size(rotation_matrices,3);
  for num=1:number_of_views
  %% move the box to be made visible to camera
  rec_points=fcn_transformPoints(rotation_matrices(:,:,num),x');
  Point3d(:,:,num)=rec_points;
  cameraRtC2W(:,:,num)=rotation_matrices(1:3,:,1);
  %% project to image frame
  K=[718.856 0 607.1928; 0 718.856 185.2157; 0 0 1];
  img_points=projectPoints(rec_points,K);
  Point2d(:,:,num)=img_points;
  scatter(img_points(:,1),img_points(:,2));
  %ginput(1);
  end
  
 %% data save for ceres
 
 % helper for pointObservedValue
  for i=1:number_of_views
    if i==1
        pointCloud_helper = Point3d(:,:,i)';
    else
        pointCloud_helper = [pointCloud_helper Point3d(:,:,i)'];
    end
  end
[aa,bb] = size(pointCloud_helper);
pointCloud = zeros(aa,bb);
pointCloud(1:3,1:n) = Point3d(:,:,1)' ;

%%% helper loop, to create 3d points equivalent to pointCloud_helper
for i=1:number_of_views
    if i==1
        pointCloud2d = Point2d(:,:,i)';
    else
        pointCloud2d = [pointCloud2d Point2d(:,:,i)'];
    end
end

for i=1:size(pointCloud_helper,2)
    
        
    pointObservedValue(1,i) = pointCloud_helper(1,i);
    pointObservedValue(2,i) = pointCloud_helper(2,i);
    pointObservedValue(3,i) = pointCloud_helper(3,i);
    pointObservedValue(4,i) = pointCloud2d(1,i);
    pointObservedValue(5,i) = pointCloud2d(2,i);
    pointObservedValue(6,i) = 0.1;
end

% w3D is some threshold. 
w3D = 10;

save('synthetic_data.mat', 'K', 'cameraRtC2W', 'pointCloud', 'pointObserved', 'pointObservedValue', 'w3D');