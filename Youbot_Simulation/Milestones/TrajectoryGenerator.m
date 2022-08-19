function traj = TrajectoryGenerator(T_se,T_sci,T_scf,T_ceg,T_ces,k)

% 각 단계에 걸리는 시간
time = [ 10;  % move to grasp standoff 5
         3; % move down to grip 3
         0.625; % close gripper default = 0.625s
         3; % move up to grasp standoff 3
         10; % move to final standoff 5
         3; % move down to final place
         0.625; % open gripper default = 0.625s
         3; % move up to final standoff
       ];

% Time scaling method
% 3 - cubic scaling -> 3차
% 5 - quintic scaling -> 5차
method =5;

% trajectory type
% 1 - Screw
% 2 - Cartesian
traj_type = 1;

% gripper state
% 0 - open
% 1 - close

configCell = { T_se, 0;       % move to end-effector initial pos
               T_sci*T_ces, 0; % move to standoff
               T_sci*T_ceg, 0; % move to grip pos 
               T_sci*T_ceg, 1; % grasping in default = 0.625s
               T_sci*T_ces, 1; % move to standoff
               T_scf*T_ces, 1; % move to final stnadoff
               T_scf*T_ceg, 1; % move to  gripper target pos =
               T_scf*T_ceg, 0; % open gripper in 0.625s
               T_scf*T_ces, 0; }; % move to final standoff
traj = [];

for i = 1:8
    N = round(time(i)*k*100);
    start = configCell{i,1};
    finish = configCell{i+1,1};
    tf = time(i);
    if traj_type == 1
        tempCell = CartesianTrajectory(start, finish, tf, N, method);
    elseif traj_type == 2
        tempCell = ScrewTrajectory(start, finish, tf, N, method);
    end
    for j = 1:length(tempCell)
        traj = [ traj; se3ToRow(tempCell{j}), configCell{i,2} ];
    end
end
end
