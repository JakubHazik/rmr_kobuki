goal = [10; 0; 1];

robot.x = 5;
robot.y =10;
robot.fi = pi;

P2 = [1 0 robot.x;
    0 1 robot.y;
    0 0 1];

P1 = [
    cos(robot.fi) -sin(robot.fi) 0;
    sin(robot.fi) cos(robot.fi) 0;
    0 0 1];

result = P2 * P1 * goal


x = goal(1) * cos(robot.fi) - goal(2) * sin(robot.fi) + robot.x
y = goal(1) * sin(robot.fi) + goal(2) * cos(robot.fi) + robot.y