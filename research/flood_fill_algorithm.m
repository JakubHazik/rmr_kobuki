map = load('map_dataset.csv');




goalX = 50;
goalY = 50;

% plot(map)


map = zeros(10,10);
% map(5,5) = 2;

points = CQueue([5,5, 2]);

map = fill2(map, points);

image(map)

function map = fill2(map, points)
while ~points.isempty
    
    point = points.pop;
    x = point(1);
    y = point(2);
    color = point(3);
    
    if x < 1 || x > 10
        continue
    end
    if y < 1 || y > 10
        continue
    end
    if map(x, y) ~= 0
        continue
    end
    
    map(x, y) = color;
    
%     points.push([x+1, y+1, color + 1]);
    points.push([x+1, y  , color + 1]);
%     points.push([x+1, y-1, color + 1]);
    points.push([x  , y-1, color + 1]);
%     points.push([x-1, y-1, color + 1]);
    points.push([x-1, y  , color + 1]);
%     points.push([x-1, y+1, color + 1]);
    points.push([x  , y+1, color + 1]);
    
    map = fill2(map, points);
end
end




function map = fill(map, x, y, color)
if x < 1 || x > 10
    return
end
if y < 1 || y > 10
    return
end
if map(x, y) ~= 0 && map(x, y) ~= 2
    return
end

map(x, y) = color;

map = fill(map, x + 1, y, color + 1);
map = fill(map, x - 1, y, color + 1);
map = fill(map, x, y + 1, color + 1);
map = fill(map, x, y - 1, color + 1);




end

