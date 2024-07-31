function circle(pos,r)
% circle(pos, r)
% args: position in x,y, radius r
rectangle('Position',[pos(1)-r,pos(2)-r,2*r,2*r], 'Curvature', [1 1], 'LineWidth',1, 'EdgeColor','b');