close all;
clear;
clc;

load points3d.txt
X=unique(points3d,'rows');
X=X(2:max(size(X))-5,:);
TRI=delaunay(X(:,1)./X(:,3),X(:,2)./X(:,3));

% figure(2)
% subplot(221)
figure;trimesh(TRI,X(:,1),X(:,2),X(:,3)),axis equal,xlabel('x'),ylabel('y'),zlabel('z'),grid
%subplot(222)
figure;plot3(-X(:,1),-X(:,2),-X(:,3),'o'),axis equal,xlabel('x'),ylabel('y'),zlabel('z'),view([-180,22]),grid
%subplot(223)
figure;plot3(-X(:,1),-X(:,2),-X(:,3),'o'),axis equal,xlabel('x'),ylabel('y'),zlabel('z'),view([108,-20]),grid
%subplot(224)
figure;plot3(-X(:,1),-X(:,2),-X(:,3),'o'),axis equal,xlabel('x'),ylabel('y'),zlabel('z'),view([-133,-48]),grid
