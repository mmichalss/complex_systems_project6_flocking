function adjusted_position = rule1(bj_position,P,line_of_sight)
    % bj_position: position of boid of interest
    % P: positions of all boids
    % line_of_sight: radius of boid's sight

    % Rule 1: Boids try to fly towards the centre of mass of neighbouring boids.
    % 1. determine neighbours of bj boid
    % 2. calculate the Pc_j -> perceived centre of mass
    % 
    b_neighbours = [];
    num_boids = size(P,1);
    for i = 1:num_boids
        b_position = P(i,:);
        if ~isequal(b_position,bj_position)
           d = norm(bj_position - b_position);
           if d <= line_of_sight
                b_neighbours = [b_neighbours; b_position];
           end
        end
    end
    % calculate Pc
    if ~isempty(b_neighbours)
         N = size(b_neighbours,1);
         Pc_j = sum(b_neighbours,1)./N;
        adjusted_position = (Pc_j - bj_position)/100;
    else
        adjusted_position = 0;
    end
end