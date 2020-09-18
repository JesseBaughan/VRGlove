function linPos_curr = CalcLinearPos(prevPosition,velocity)

    linPos_curr = zeros(1,3);
    for i = 1:1:3
        %   //We need to pass velocity through a HP filter
        linPos_curr(1,i) = prevPosition(1,i) + (velocity(1,i) * (1/256));
    end

end

