% Calculate joint frame using Denavit-Hartenberg parameters
function frame = dh(parameters)
    a = parameters.a;
    d = parameters.d;
    alpha = parameters.alpha;
    theta = sym(parameters.theta);

    frame = [
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)];
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)];
        [0, sin(alpha), cos(alpha), d];
        [0, 0, 0, 1];
    ];
end