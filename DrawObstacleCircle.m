function DrawObstacleCircle(x, y, R)
xx = [];
yy = [];
for deg = 0 : 361
    xx = [xx, x + R * cosd(deg)];
    yy = [yy, y + R * sind(deg)];
end
plot(xx, yy, 'Color', [0.5 0.5 0.5]);
end