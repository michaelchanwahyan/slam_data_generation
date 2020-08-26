function [isOnSurface2d , gamma_] = checkHit( P1 , P2 , unitVec, startOrigin )
    % from lidar_origin to u, extrapolate a point to hit W and L
    % startOrigin + gamma_ * unitVec shall be a point on the line P1P2
    % gamma_ shall be a nonnegative value
    % if the gamma_ value is too large,
    % we return -1 as parallel / nearly parallel wall
    % are not considered.  isOnSurface2d is also false in this case

    isOnSurface2d = false;

    xo = startOrigin(1); yo = startOrigin(2);
    xu = unitVec(1); yu = unitVec(2);

    % (y-y1)/(x-x1) = (y2-y1)/(2-x1)
    % (y2-y1)x + (-x2+x1)y + (y2-y1)x1 - (x2-x1)y1 = 0
    %    a   x +     b   y +           c           = 0
    x1 = P1(1); y1 = P1(2);
    x2 = P2(1); y2 = P2(2);
    a = y2 - y1;
    b = -x2 + x1;
    c = - (y2 - y1) * x1 + (x2 - x1) * y1;


    axo_plus_byo = a * xo + b * yo;
    axu_plus_byu = a * xu + b * yu;
    if (abs(axu_plus_byu) < 1E-3)
        gamma_ = -1;
        return;
    else
        gamma_ = - ( axo_plus_byo + c ) / axu_plus_byu;
        if (gamma_ > 1E3)
            gamma_ = -1;
            return;
        end
    end
    
    if (gamma_ < 0)
        gamma_ = -1;
        return;
    end

    q = [ xo , yo ] + gamma_ * [ xu , yu ];

    % (P1)----------------(q)------(P2)
    % P1q + P2q - P1P2 = 0

    % (P1)-------------------------(P2)------(q)
    % P1q + P2q - P1P2 != 0

    P1q = p2p_dist(P1,q);
    P2q = p2p_dist(P2,q);
    P1P2 = p2p_dist(P1,P2);
    if (abs(P1q + P2q - P1P2) < 1E-3)
        isOnSurface2d = true;
    end
end
