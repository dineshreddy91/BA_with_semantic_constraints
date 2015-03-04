function [ points ] = fcn_transformPoints(tr1, points)
%Author: Jim West
%Date: 7/5/2010
%Use: pointsOut = fcn_transformPoints(trOrigin,trDestination, pointsToBeMoved[mx3])
%tr1 and tr2 are in the form of [1 0 0 0; [Posx Posy Posz], unitx' unity' unitz']
%Description: This function will take two transformation matrices and transform from 
%tr1 to tr2 converting all the points from frame 1-->2

%Transform points from frame1 to frame2:
[r c] = size(points);
if (r == 3)
    points = points';
    [r c] = size(points);
elseif (c==3)
    %Do nothing
else
    error('You do not have an nx3 matrix');
end

pointsNew = zeros(4,r);
for i = 1:r
    pointsNew(:,i) = (tr1)*[points(i,:) 1]'; 
end
points = pointsNew(1:3,:)';