function [r, theta] = car2pol(x,y)
    z = x+y*1i;

    if x > 0
        error("Input Negative x");
    elseif y < 0
        error("Input Positive y")
    end

    r = abs(z);
    theta = rad2deg(angle(z));

    ang_des = 90 - theta;

    fprintf("magnitude : %.2f\nangle: %.2f\n", r, ang_des);
end