%This is a small function I wrote to obtain the so2 rotational matrix

function m = rot2(theta)
    m = [cos(theta) -sin(theta); sin(theta) cos(theta)];
end