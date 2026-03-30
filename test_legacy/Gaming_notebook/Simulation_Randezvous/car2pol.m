function car2pol(x,y)
    z = x+y*1i;

    if x > 0
        error("Input Negative x");
    elseif y < 0
        error("Input Positive y")
    end

    mag = abs(z);
    ang = rad2deg(angle(z));

    ang_des = 90 - ang;

    fprintf("magnitude : %.2f\nangle: %.2f\n", mag, ang_des);
end