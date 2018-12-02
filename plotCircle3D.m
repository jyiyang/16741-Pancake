
function plotCircle3D(center,normal,radius)
    theta=0:0.01:2*pi;
    v=null(normal);
    points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
    patch(points(1,:),points(2,:),points(3,:),'y','EdgeColor', 'y', 'LineWidth',5);
end