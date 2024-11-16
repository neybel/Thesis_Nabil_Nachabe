function plot_car(state, car_length,car)
    x = state(1);
    y = state(2);
    theta = state(3);

    % Coordinates of the car
    car_x = [-car_length/2, car_length/2, car_length/2, -car_length/2];
    car_y = [-car_length/4, -car_length/4, car_length/4, car_length/4];
    
    % Rotate car coordinates
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated_coords = R * [car_x; car_y];
    
    % Translate car coordinates
    x_coords = rotated_coords(1, :) + x;
    y_coords = rotated_coords(2, :) + y;
    
    % Plot car
    if car==1
    fill(x_coords, y_coords, 'b');
    else
    fill(x_coords, y_coords, 'r');
    end
end
