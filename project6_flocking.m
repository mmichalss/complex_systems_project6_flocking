% project 6 - Flocking simulation

T = 400;
borders = [0 10 0 10];
v_limits = [0.1 0.3];
N = 10;
%P = [2 2; 2.2 2; 1.2 1.3];
%V = -1.*[0.1 0.11; 0.11 0.1; 0.1 0];
P = [randd(borders(1), borders(2), N);randd(borders(1), borders(2), N)]';
V = [randd(v_limits(1), v_limits(2), N);randd(v_limits(1), v_limits(2), N)]';
line_of_sight = 2;
min_distance = 0.1;
figure;
plot_bodies(P,V,T,borders,line_of_sight,min_distance);

function random_doubles = randd(a, b, length)
    % a: lower bound of the range
    % b: upper bound of the range
    % length: number of random doubles to generate
    
    % Generate random doubles in the range [a, b]
    random_doubles = a + (b - a) * rand(1, length);
    
    % Round to one decimal place
    random_doubles = round(random_doubles, 1);
end

function plot_bodies(P,V,T,borders,line_of_sight,min_distance)
    % P: [x, y] coordinates of the bodies
    % V: [vx, vy] components of the velocity vectors
    % T: number of time steps
    
    num_bodies = size(P, 1);
    border_width = abs(borders(2) - borders(1));
    all_vertices = calculate_vertices(P,V,border_width);
    
    for t = 1:T
        clf;
        hold on;
        axis(borders);
        for i = 1:num_bodies
            vertices = squeeze(all_vertices(i, :, :)); % ommit i dimenision to plot fill
            fill(vertices(:, 1), vertices(:, 2), 'k');
        end
        % Here insert the logic for touching the border of the axis.
        [P,V] = next_move(P,V,borders,line_of_sight,min_distance);
        all_vertices = calculate_vertices(P,V,border_width);
        pause(0.01)
    end
end

function all_vertices = calculate_vertices(P,V,border_width)
    % position: [x, y] coordinates of the body
    % velocity: [vx, vy] components of the velocity vector
    num_bodies = size(P, 1);
    all_vertices = zeros(num_bodies, 3, 2);

    for i = 1:num_bodies
    position = P(i, :);
    velocity = V(i, :);

    % Calculate the orientation angle of the velocity vector
    angle = atan2(velocity(2), velocity(1));
    
    % Define the vertices of the triangle relative to the body's position
    L = 0.01*border_width; % Length of the triangle
    W = 0.005*border_width; % Width of the triangle
    base_vertices = [
        L, 0;
        -L, W;
        -L, -W
    ];
    
    % Rotate the triangle vertices
    R = [cos(angle), -sin(angle); sin(angle), cos(angle)];
    rotated_vertices = (R * base_vertices')';
    vertices = rotated_vertices + position;
    all_vertices(i,:,:) = vertices;
    end
end

function [new_position, new_velocity] = border_collision(position,velocity,borders)
    borders_diagonal = sqrt(2)*abs(borders(1)-borders(2)); % assuming a square
    line = -borders_diagonal*velocity*3 + position;
    [new_position, new_velocity]= find_intersection(position,line,borders,velocity);
end

function [new_P, new_V]= next_move(P,V,borders,line_of_sight,min_distance)
    num_bodies = size(P, 1);
    new_P = zeros(num_bodies,2);
    new_V = zeros(num_bodies,2);
    for i = 1:num_bodies
         position = P(i, :);
         velocity = V(i, :);
         v1 = rule1(position,P,line_of_sight);
         v2 = rule2(position,P,min_distance);
         v3 = rule3(position,velocity,P,V,line_of_sight);
         v = velocity + v1 + v2 + v3;
         next_position = position + v;
         next_velocity = v;
         if next_position(1) < borders(1) || next_position(1) > borders(2) || next_position(2) < borders(3) || next_position(2) > borders(4)
             [new_position, new_velocity] = border_collision(position,velocity,borders);
             next_position = new_position;
             next_velocity = new_velocity; % velocity doesn't change if adjustment wasn't necessary.
         end
         new_P(i,:) = next_position;
         new_V(i,:) = next_velocity;
    end
    %new_P = P;
    %new_V = V;
end

function [new_position, new_velocity] = find_intersection(p1, p2, borders, velocity)
    % p1, p2: points on the first line [x1, y1] (particle) [x2, y2] (line end)
    % borders: coordinates of the border
    % velocity: velocity of the boid

    % I want only (0 0)(5 0), (5 0)(5 5), (5 5)(0 5) (0 5)(0 0) -> 
    % (1 1)(2 1), (2 1)(2 2), (2 2)(1 2), (1 2)(1 1)
    new_velocity = velocity;
    new_position = p1;
    border_width = abs(borders(2) - borders(1)); % assuming a square
    border_points = [borders(1) borders(1) borders(2) borders(1);
                     borders(2) borders(1) borders(2) borders(2);
                     borders(2) borders(2) borders(1) borders(2);
                     borders(1) borders(2) borders(1) borders(1)];
    for i = 1:size(border_points, 1)
        q1 = border_points(i,1:2);
        q2 = border_points(i,3:4);
    
        A = [p2(1) - p1(1), -(q2(1) - q1(1));
             p2(2) - p1(2), -(q2(2) - q1(2))];
        b = [q1(1) - p1(1);
             q1(2) - p1(2)];

        if abs(det(A)) < 1e-10
            continue; % Skip this iteration if A is singular
        end
        
        % Solve for t
        t = A \ b;
        
        % check whether the proposed border is the one on which the boid
        % is currently.
        is_on_line = is_point_on_line_segment(p1,q1,q2);
        if t(1) >= 0 && t(1) <= 1 && t(2) >= 0 && t(2) <= 1 && ~is_on_line
            %remaining_velocity = (1 - t(1)) * velocity;

            intersection = p1 + t(1) * (p2 - p1);
            new_position = intersection;% + remaining_velocity;
            break;
        end
    end
    % PROBLEM: if no intersection -> no border line behind boid.
    % SOLUTION: change the angle of the velocity towards center of the
    % boundry. 
    if new_position == p1
        degrees = 32;
        radians = degrees * pi / 180;
        angle_adjustment = radians;
        new_velocity = adjust_velocity_towards_center([p1 p2], velocity, border_width, angle_adjustment);
        new_position = p1 + new_velocity;
    end
end

function [distance, nearest_point] = point_to_line_segment_distance(p, q1, q2)
    % Calculate the distance from point p to the line segment defined by q1 and q2
    % p, q1, q2: [x, y] coordinates
    v = q2 - q1;
    w = p - q1;
    c1 = dot(w, v);
    if c1 <= 0
        distance = norm(p - q1);
        nearest_point = q1;
        return;
    end
    c2 = dot(v, v);
    if c2 <= c1
        distance = norm(p - q2);
        nearest_point = q2;
        return;
    end
    b = c1 / c2;
    pb = q1 + b * v;
    distance = norm(p - pb);
    nearest_point = pb;
end

function is_on_segment = is_point_on_line_segment(point, p1, p2)
    % point: the point to check [x, y]
    % p1, p2: endpoints of the line segment [x, y]
    % is_on_segment: boolean indicating whether the point lies on the line segment
    
    % Vector from p1 to p2
    v = p2 - p1;
    
    % Vector from p1 to the point
    w = point - p1;
    
    % Check if the point is collinear with the line segment
    cross_product = v(1) * w(2) - v(2) * w(1);
    if abs(cross_product) > eps
        % The point is not collinear
        is_on_segment = false;
        return;
    end
    
    % Check if the point lies within the bounds of the line segment
    dot_product = dot(w, v);
    if dot_product < 0 || dot_product > dot(v, v)
        % The point is outside the bounds of the line segment
        is_on_segment = false;
        return;
    end
    
    % The point is collinear and within the bounds of the line segment
    is_on_segment = true;
end

function new_velocity = adjust_velocity_towards_center(position, velocity, frame_size, angle_adjustment)
    % position: [x, y] coordinates of the particle
    % velocity: [vx, vy] components of the velocity vector
    % frame_size: size of the quadratic frame [width, height]
    % angle_adjustment: small angle to adjust towards the center (in radians)
    
    % Calculate the center of the frame
    center = frame_size / 2;
    
    % Calculate the vector towards the center
    to_center = center - position;
    
    % Calculate the current angle of the velocity
    current_angle = atan2(velocity(2), velocity(1));
    
    % Calculate the target angle towards the center
    target_angle = atan2(to_center(2), to_center(1));
    
    % Adjust the current angle slightly towards the target angle
    if target_angle > current_angle
        current_angle = current_angle + angle_adjustment;
    else
        current_angle = current_angle - angle_adjustment;
    end
    
    % Calculate the new velocity vector based on the adjusted angle
    speed = norm(velocity);
    new_velocity = speed * [cos(current_angle), sin(current_angle)];
end