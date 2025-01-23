function adjusted_velocity = rule3(bj_position,bj_velocity,P,V,line_of_sight)
    % bj_position: position of boid of interest
    % bj_velocity: velocity of boid of interest
    % P: positions of all boids
    % V: velocities of all boids
    % line_of_sight: radius of sight of each boid

    % Rule 3: velocity of each member of the group is = velocity of the center of
    % mass (i.e. velocity of the whole group)
    % 1. determine neighbours of bj boid
    % 2. calculate the perceived velocity pv_j and add a small portion of
    % it to the boid's v.

    pv_j = [];
    N = 0;
    num_boids = size(P,1);
    for i = 1:num_boids
        b_position = P(i,:);
        b_velocity = V(i,:);
        if ~isequal(b_position,bj_position)
            if norm(b_position - bj_position) <= line_of_sight
                if isempty(pv_j)
                    pv_j = b_velocity;
                else
                    pv_j = pv_j + b_velocity;
                end
                N = N +1;
            end
        end
    end
    if N ~= 0
        pv_j = pv_j/N;
        adjusted_velocity = (pv_j - bj_velocity) /8;
    else
        adjusted_velocity = 0;
    end
end