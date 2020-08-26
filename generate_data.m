close all
clear
clc

%  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  \  %
%  |                                               |  %
%  |   START  USER  DEFINED  PARAMETERS SETTINGS   |  %
%  |                                               |  %
%  \  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  %

%            room dimension and landmark allocation
%           ----> y-axis j-coor jdx
%                         10m
%       (0,0) ------------------------------ (0,10)
%            /                              \
%         |  |                              |
%         |  |                              |
%  x-axis v  |      / \             / \     |
%  i-coor    |      \ /             \ /     |
%  idx       |                              |
%            |                              | 5m
%            |                              |
%            |                              /
%        10m |              ---------------- (5,10)
%            |             /      5m
%            |             | (5,5)
%            |             |
%            |             |
%            |             | 5m
%            |     / \     |
%            |     \ /     |
%            |             |
%     (10,0) \             / (10,5)
%             -------------
%                  5m

% define the wall contour
W = [
       0.0,  0.0,  0.0, 10.0;
       0.0, 10.0,  5.0, 10.0;
       5.0, 10.0,  5.0,  5.0;
       5.0,  5.0, 10.0,  5.0;
      10.0,  5.0, 10.0,  0.0;
      10.0,  0.0,  0.0,  0.0;
    ];

% define the landmark contour
L = [
       2.0,  4.0,  2.0, 4.5;
       2.0,  4.5,  2.5, 4.5;
       2.5,  4.5,  2.5, 4.0;
       2.5,  4.0,  2.0, 4.0;
    ];

WL = [ W ; L ] ;          % W and L are of the same kind of structure

% define lidar moving path
l_path = [
            1.0,  1.0;
            1.0,  8.0;
            3.0,  8.0;
            3.0,  4.0;
            9.0,  4.0;
         ];

% define lidar moving speed
l_moving_speed = 0.05;    % m/s

% prepare iteration constants
% % hardware settings parameters
l_radius = 0.08;          % meter

% % operational settings  parameters
l_rps = 5;                % rotation per second
l_rpm = l_rps * 60;       % rotation per minute
%l_azimuth_res = 2048;
l_azimuth_res = 360;
%l_azimuth_res = 30;

% when moving from previous path to current path, it may require turning
% here we are defining the turning speed of the lidar
l_inter_path_turn_rps = 0.0167;  % rotation per second

%  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  \  %
%  |                                               |  %
%  |  END  OF  USER  DEFINED  PARAMETERS SETTINGS  |  %
%  |                                               |  %
%  \  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  =  %






% % auto-generated parameters
TWOPI = 2 * pi;
wallNum = size(W,1);
landmarkNum = size(L,1);
path_sectNum = size(l_path,1) - 1;
path_sectLength = zeros(1,path_sectNum);

l_origin = l_path(1,:);



scan_time = 0;
% calculate the time required for natvigation only
for i = 1 : path_sectNum
    path_sectLength(i) = p2p_dist(l_path(i,:), l_path(i+1,:));
    path_duration = path_sectLength(i) / l_moving_speed;
    scan_time = scan_time + path_duration;
    path_ptNum = ceil(scan_time * l_rps * l_azimuth_res);
    fprintf('for path %d, it takes %f seconds\n', i, path_duration);
    fprintf('\twhich accounts for %d points\n', path_ptNum);
end ; clear i
fprintf('time for natvigation = %f seconds\n', scan_time);

% calcualte the time required for lidar to turn
l_turn_angle = zeros(1,path_sectNum - 1);
l_turn_dir = zeros(1,path_sectNum - 1);
for i = 1 : path_sectNum - 1
    % (Pa)------------(Pb)
    %                \___\
    %              phi    \
    %                      \
    %                      (Pc)
    % pba_diff = Pb - Pa;
    % pbc_diff = Pb - Pc;

    % % compute the turn angle
    % \phi := acute angle    Pb := intersection point of AB and BC
    % \phi = acos( (pba_diff' * pbc_diff) / (|pba_diff| * |pbc_diff|) )
    Pa = l_path(i  ,:);
    Pb = l_path(i+1,:);
    Pc = l_path(i+2,:);
    pba_diff = Pb - Pa;
    pbc_diff = Pb - Pc;
    phi = acos((pba_diff*pbc_diff') / (norm(pba_diff,2)*norm(pbc_diff,2)));

    l_turn_angle(i) = pi - phi;
    scan_time = scan_time + l_turn_angle(i)/(TWOPI*l_inter_path_turn_rps);

    % % compute the turn dir
    pab = p2p_dist( Pa , Pb );
    pbc = p2p_dist( Pb , Pc );
    c = cos( pi - phi ); s = sin( pi - phi );
    R = [ c , -s ; s , c ];
    % if turn dir is anti-clockwise
    %     l_turn_dir(i) = 1;
    % else if turn dir is clockwise
    %     l_turn_dir(i) = -1;
    l_turn_dir(i) = p2p_dist(Pc , Pb+((Pb-Pa)*R')*pbc/pab) < 1E-3;
    l_turn_dir(i) = -1 + 2 * l_turn_dir(i);
end ; clear i
fprintf('time for natvigation and turn dir = %f seconds\n', scan_time);

path_sectCumLength = cumsum(path_sectLength); % cumulative sum

underestimated_ptNum = ceil(scan_time*l_rps*l_azimuth_res) + path_sectNum;
PCL = zeros( ceil( underestimated_ptNum * 1.05 ) , 2);
PCL_ptNum = size(PCL,1);
fprintf('allocated points for the whole journey: %d\n', PCL_ptNum);
noise_measuring = 0.025 * randn(1,ceil( underestimated_ptNum * 2 ));
noise_motion = 0.005 * randn(ceil( underestimated_ptNum * 2 ) , 2);


% --------------------------------------------------------
% define control flow parameters during LiDAR slam journey
% --------------------------------------------------------
ptIdx = 1;
path_progress = false(1,path_sectNum);
pathIdx = 1;
gotoTurningRotine = false;
t_for_latest_turn_dir = 0;
journeyDone = false;
progress_checkpt_scale = 100;
progress_interval = floor(PCL_ptNum / progress_checkpt_scale);
progress_checkpt = 0;

t_curr = 0; % in second; the progression variable used in while loop
t_next = t_curr + 1 / l_rps;
while (~journeyDone)
    %disp(l_origin);
    if (ptIdx > progress_checkpt)
        progress_checkpt = progress_checkpt + progress_interval;
        t = datetime('now');
        fprintf('[ %s ] progress: %d / %d\n', t, ptIdx, PCL_ptNum);
    end
    if (ptIdx > size(PCL,1))
        fprintf('exceed !\n');
        journeyDone = true;
        continue;
    end
    if (~gotoTurningRotine)
        % calculate (x,y)_curr
            % % natvigateProgressCnt = sum(natvigationProgress);
            % % path to be interpolated is between point
            % % l_path( natvigationProgressCnt + 1 , :)
            % %     and
            % % l_path( natvigationProgressCnt + 2 , :)
            t_ = t_curr - t_for_latest_turn_dir;
            dist_walked = t_ * l_moving_speed;
            if (dist_walked - 1E-3 >= path_sectCumLength( pathIdx ))
                path_progress( pathIdx ) = true;
                pathIdx = pathIdx + 1;
                gotoTurningRotine = true;
                if (pathIdx > path_sectNum)
                    journeyDone = true;
                end
                continue;
            end
            if (pathIdx > 1)
                dist_walked = dist_walked - path_sectCumLength(pathIdx-1);
            end
            alph_ = dist_walked / path_sectLength(pathIdx);
            x_curr = (1 - alph_) * l_path(pathIdx,1) + alph_ * l_path(pathIdx+1,1);
            y_curr = (1 - alph_) * l_path(pathIdx,2) + alph_ * l_path(pathIdx+1,2);
            % % x_curr := the x position of lidarOrigin at current time
            % % y_curr := the y position of lidarOrigin at current time

        % calculate (x,y)_next
            t_next = t_curr + 1 / l_rps;
            t_ = t_next - t_for_latest_turn_dir;
            dist_walked = t_ * l_moving_speed;
            if (pathIdx > 1)
                dist_walked = dist_walked - path_sectCumLength(pathIdx-1);
            end
            alph_ = dist_walked / path_sectLength(pathIdx);
            x_next = (1 - alph_) * l_path(pathIdx,1) + alph_ * l_path(pathIdx+1,1);
            y_next = (1 - alph_) * l_path(pathIdx,2) + alph_ * l_path(pathIdx+1,2);
            % % x_next := the x position of lidarOrigin at coming time
            % % y_next := the y position of lidarOrigin at coming time

        % generate the x y points
            % from (x,y)_curr to (x,y)_next, should be 1 rotation in lidar
            x_diff = x_next - x_curr; % get dir of lidar machine head
            y_diff = y_next - y_curr; % get dir of lidar machine head
            % u : unit vector of lidar dir
            % this variable is also used the turn routine
            u = [x_diff,y_diff] ./ norm([x_diff,y_diff],2);
            u_dir_theta = pi/2 - atan(u(2) / u(1));
            R_wrt_dir = [ cos(u_dir_theta), -sin(u_dir_theta);
                          sin(u_dir_theta), cos(u_dir_theta) ];
            for azi = 0 : l_azimuth_res - 1
                % check if l_origin touches path end-point
                if(p2p_dist(l_origin, l_path(pathIdx,:)) > path_sectLength(pathIdx))
                    fprintf('current path natvigation just exceed ! BREAK !\n');
                    break;
                end
                % update l_origin
                beta_ = azi / l_azimuth_res;
                l_origin(1) = (1 - beta_) * x_curr + beta_ * x_next;
                l_origin(2) = (1 - beta_) * y_curr + beta_ * y_next;
                % generate the point at azi from the updated l_origin
                c = cos( -TWOPI * beta_ );
                s = sin( -TWOPI * beta_ );
                R = [ c , -s ; s , c ];   % rotation matrix about z-axis
                Ru = R * u';              % u is the normalised (x,y)_diff
                % search for the gamma_ such that
                % emitted laser pulse (along the dir of Ru)
                % at l_origin + gamma_ * Ru'
                % intercepts the WALLS or LANDMARKS
                surfCandi = false(1,wallNum + landmarkNum); % surface candidates wrt WL
                gamma_arr = zeros(1,wallNum + landmarkNum); % gamma value candidates
                for wlIdx = 1 : wallNum + landmarkNum
                    [ surfCandi(wlIdx) , gamma_arr(wlIdx) ] = checkHit( WL(wlIdx,1:2) , WL(wlIdx,3:4) , Ru' , l_origin + noise_motion(ptIdx,:) );
                end
                if (~sum(surfCandi))
                    disp(surfCandi);
                    disp(sum(surfCandi));
                    continue;
                end
                gamma_ = min( gamma_arr( surfCandi ) );
                %x_ = l_origin(1) + gamma_ * Ru(1);
                %y_ = l_origin(2) + gamma_ * Ru(2);
                Ru = R_wrt_dir * Ru;
                x_ = gamma_ * Ru(1);
                y_ = gamma_ * Ru(2);

                PCL(ptIdx,:) = [ x_ , y_ ];
                ptIdx = ptIdx + 1;
            end % end for azi = 1 : l_azimuth_res
            %fprintf('finish 1 round rotation\n');
    end % end if (~gotoTurningRotine)
    
    if (gotoTurningRotine)
        % (1)----(2)----(3)
        angle_to_rotate = l_turn_dir(pathIdx-1) * l_turn_angle(pathIdx-1);
        % at (x,y)_curr
        % turn process requires l_turn_angle(pathIdx-1) / l_inter_path_turn_rps seconds
        % within which for each 1 / l_rps seconds, the lidar should
        % have turned        
        % % lesson learnt: don't use time 
        % % lesson learnt: should use total amount of azimuth increment
        
        total_azimuth_during_turn = floor(l_turn_angle(pathIdx-1) / (TWOPI * l_inter_path_turn_rps) * l_rps * l_azimuth_res);
        t_tempstored = t_;
        t_ = 0;
        u_dir_theta_original = u_dir_theta;
        for azi = 0 : total_azimuth_during_turn - 1
            % l_origin is unchanged
            beta_ = azi / l_azimuth_res;
            % generate the point at azi from the unchanged l_origin
            % note that it is not the case for angularly-stationary azimuth change
            % the lidar is turn and thus the net angle
            u_dir_theta = l_turn_dir(pathIdx-1) * t_ * TWOPI * l_inter_path_turn_rps;
            c = cos( -TWOPI * beta_ + u_dir_theta );
            s = sin( -TWOPI * beta_ + u_dir_theta );
            R = [ c , -s ; s , c ];       % rotational matrix to rotate from lidar dir
            Ru = R * u';                  % u is the normalised (x,y)_diff = (x,y)_next - (x,y)_curr
            % search for the gamma_ such that
            % emitted laser pulse (along the dir of Ru)
            % at l_origin + gamma_ * Ru'
            % intercepts the WALLS or LANDMARKS
            surfCandi = false(1,wallNum + landmarkNum); % surface candidates wrt WL
            gamma_arr = zeros(1,wallNum + landmarkNum); % gamma value candidates
            for wlIdx = 1 : wallNum + landmarkNum
                [ surfCandi(wlIdx) , gamma_arr(wlIdx) ] = checkHit( WL(wlIdx,1:2) , WL(wlIdx,3:4) , Ru' , l_origin + noise_motion(ptIdx,:) );
            end
            gamma_ = min( gamma_arr( surfCandi ) );
            R_wrt_dir = [ cos(u_dir_theta_original - u_dir_theta), -sin(u_dir_theta_original - u_dir_theta);
                          sin(u_dir_theta_original - u_dir_theta), cos(u_dir_theta_original - u_dir_theta) ];
            % offset the point w.r.t LiDAR
            Ru = R_wrt_dir * Ru;
            %x_ = l_origin(1) + gamma_ * Ru(1);
            %y_ = l_origin(2) + gamma_ * Ru(2);
            x_ = gamma_ * Ru(1);
            y_ = gamma_ * Ru(2);

            PCL(ptIdx,:) = [ x_ , y_ ];
            ptIdx = ptIdx + 1;
            t_ = t_ + 1 / (l_rps * l_azimuth_res);
        end % end for azi = 0 : total_azimuth_during_turn - 1
        %t_curr = t_ + t_tempstored;
        t_next = t_ + t_tempstored;
        t_for_latest_turn_dir = abs(angle_to_rotate) / ( TWOPI * l_inter_path_turn_rps );
        gotoTurningRotine = false;
    end % end if (gotoTurningRotine)

    t_curr = t_next;
end % end while (~natvigationDone)



fprintf('Press Enter to plot LiDAR journey observation ...\n');
%pause;

% % plot the LiDAR journey observation
N = ptIdx - 1 - path_sectNum;
figure;
for n = 1 : l_azimuth_res : N
    scatter( PCL(n:min(N,n+l_azimuth_res-1),1) , PCL(n:min(N,n+l_azimuth_res-1),2) , 'b.' );
    hold on;
    scatter( 0 , 0 , 'rx' );
    hold off;
    axis([-10,10,-10,10]);
    %axis equal;
    pause(0.02);
end ; clear n
