function [cameraRtC2W, pointCloud, boxConstVar, optm] = bundleAdjustment2D3DBoxFile(cameraRtC2W,pointCloud,pointObserved, pointObservedValue, K, weight, mode, rand_pts,N_new)
[camID,ptsID,valID] = find(pointObserved);
num_of_rand_points = size( rand_pts, 1 );
nCam=uint32(size(cameraRtC2W,3));
nPts=uint32(size(pointCloud,2));
nObs=uint32(length(valID));
fx=K(1,1);
fy=K(2,2);
px=K(1,3);
py=K(2,3);
N=N_new;
% transform the cameraRt
cameraRtW2C = zeros(3,4,size(cameraRtC2W,3));
for cameraID=1:size(cameraRtC2W,3)
cameraRtW2C(:,:,cameraID) = transformCameraRt(cameraRtC2W(:,:,cameraID));
end
fname_inout = tempname;
fname_in = [fname_inout '.txt'];
fname_out = [fname_inout '.txt'];
global objectLabel;
nObjects = objectLabel.length;
objectRtW2O = zeros(3,4,objectLabel.length);
for objectID = 1:objectLabel.length
objectRtW2O(:,:,objectID) = transformCameraRt(objectLabel.objectRtO2W(:,:,objectID));
end
objectHalfSize = objectLabel.objectSize(1:objectLabel.length,:)'/2;
objectWeights = objectLabel.optimizationWeight(1:objectLabel.length);
fin = fopen(fname_in, 'wb');
fwrite(fin, nCam, 'uint32');
fwrite(fin, nPts, 'uint32');
fwrite(fin, nObs, 'uint32');
fwrite(fin, nObjects, 'uint32'); %<= Object
fwrite(fin, num_of_rand_points, 'uint32');
fwrite(fin, fx, 'double');
fwrite(fin, fy, 'double');
fwrite(fin, px, 'double');
fwrite(fin, py, 'double');
fwrite(fin, N, 'double');
fwrite(fin, cameraRtW2C, 'double');
fwrite(fin, objectRtW2O, 'double'); %<= Object
fwrite(fin, pointCloud, 'double');
% write observation
ptsObservedIndex = uint32([camID,ptsID]-1)';
ptsObservedValue = pointObservedValue(:,valID);
fwrite(fin, ptsObservedIndex, 'uint32');
fwrite(fin, ptsObservedValue, 'double');
fwrite(fin, objectHalfSize, 'double'); %<= Object
fwrite(fin, objectWeights, 'double'); %<= Object
fwrite(fin, rand_pts(:,1), 'uint32');
fwrite(fin, rand_pts(:,2), 'uint32');
fclose(fin);
cmd = sprintf('./build/ba2D3DboxType1 %d %f %s %s', mode, weight, fname_in, fname_out);
fprintf('%s\n',cmd);
tic;
system(cmd);
optm = toc;
% read the result back;
fout = fopen(fname_out, 'rb');
nCam=fread(fout,1,'uint32');
nPts=fread(fout,1,'uint32');
nObjects=fread(fout,1,'uint32');
cameraRtW2C = fread(fout,12*nCam,'double');
pointCloud = fread(fout,3*nPts,'double');
objectRtW2O = fread(fout,12*nObjects,'double');
boxConstVar = fread(fout,3*num_of_rand_points, 'double');
fclose(fout);
cameraRtW2C = reshape(cameraRtW2C,3,4,[]);
%objectRtW2O = reshape(objectRtW2O,3,4,[]);
pointCloud = reshape(pointCloud,3,[]);
% transform the cameraRt back
for cameraID=1:size(cameraRtW2C,3)
cameraRtC2W(:,:,cameraID) = transformCameraRt(cameraRtW2C(:,:,cameraID));
end
for objectID = 1:objectLabel.length
objectLabel.objectRtO2W(:,:,objectID) = transformCameraRt(objectRtW2O(:,:,objectID));
end
%delete(fname_in);
%delete(fname_out);