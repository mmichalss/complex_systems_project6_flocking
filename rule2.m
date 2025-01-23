function c = rule2(bj_position,P,min_distance)
    % bj_position: position of boid of interest
    % P: positions of all boids
    % min_distance: minimal distance between two boids (which triggers the velocity change 

    % Rule 2: Boids try to keep a small distance away from other objects (including other boids).
    % 1. if p of a boid is < min_distance change c to c - b.pos - b.pos
    c = 0;
    num_boids = size(P,1);
    for i = 1:num_boids
        b_position = P(i,:);
        if ~isequal(b_position,bj_position)
            if norm(b_position - bj_position) < min_distance
                c = c - (b_position - bj_position);
            end
        end
    end
end