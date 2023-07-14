function [center,mass] =  centerFinder(Points, Masses)
    %Points are taken as column vectors.
    % Points.size = (3, num_of_points).
    %Input check for whether they are proper or not.
    assert(size(Points,1) == 3);
    assert(size(Points,2) == size(Masses,1));
    %Points form
    %{
       P1x P2x P3x ... Pnx |
       P1y P2y P3y ... Pny |
       P1z P2z P3z ... Pnz |
      
    %}

    mass = sum(Masses);
    center = Points * Masses / sum(Masses);

end