function action = CS6380_UAS_tom_no_subgoals(percept)
% CS6380_UAS_tom_ - UAS agent: flies trajectory
% On input:
%     percept (struct vector): agent percept data
%       .x (float): x position of agent
%       .y (float): y position of agent
%       .z (float): z position of agent
%       .dx (float): x heading
%       .dy (float): y heading
%       .dz (float): z heading
%       .speed (float): ground speed
%       .time (float): current time
%       .del_t (float): time step
% On output:
%     action (struct vector): agent actions
%       .dx (float): heading in x
%       .dy (float): heading in y
%       .dz (float): heading in z
%       .speed (float): speed to move
%       .messages (struct vector)
% Call:
%     action = CS6380_UAS_tom_(percept);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

CS6380_load_ABMS_data;
CS6380_load_UAS_tom;

MY_ID = 'UAS_tom_';

persistent state USS UAS state_vars
persistent lanes cur_lane num_lanes
persistent intentions_stack traj set_dir set_speed l_entry l_exit
persistent ATOC GRS in_flight
persistent my_flight new_award flight_id
persistent KB priorities

messages_out = [];

if isempty(state)
    state = HANDLE_PERCEPTS;
    USS = [];
    UAS = [];
    state_vars(1) = 10; % x coord
    state_vars(2) = 10; % y coord
    state_vars(3) = 0; % z coord
    state_vars(4) = 1; % dx 
    state_vars(5) = 0; % dy 
    state_vars(6) = 0; % dz 
    state_vars(7) = 0; % speed
    lanes = [];
    num_lanes = 0;
    cur_lane = 0;
    in_flight = 0;
    flight_id = 0;
    new_award = 0;
    priorities = Inf*ones(num_atoms,2);
    priorities(ASSIGNED,2) = 1;
    priorities(NOMINAL,2) = 1;
    KB(1).clauses = [-IN_LANE, -ON_HEADING, -SPEED_OK, NOMINAL];
    KB(2).clauses = [-ASSIGNED];
    KB(3).clauses = [-IN_FLIGHT];
    KB(4).clauses = [-AT_START];
    KB(5).clauses = [-AT_NEXT_WAYPT];
    KB(6).clauses = [-LAST_LANE,-AT_NEXT_WAYPT,AT_FINISH];
    intentions_stack = [ASSIGNED];
    messages_out = CS6380_make_message(BROADCAST,MY_ID,ANNOUNCE_SELF,[],[]);
end

xa = percept.x;
ya = percept.y;
za = percept.z;
dx = percept.dx;
dy = percept.dy;
dz = percept.dz;
realx = [];
realy = [];
realz = [];

loc = [xa;ya;za];
dir = [dx;dy;dz];
speed = percept.speed;
new_speed = speed;
del_t = percept.del_t;
cur_time = percept.time;

messages_in = percept.messages;

done = 0;
while done==0
    switch state
        case HANDLE_PERCEPTS % handle percepts and messages
            state = ANALYZER;
            if ~isempty(messages_in)
                num_messages_in = length(messages_in);
                for m = 1:num_messages_in
                    mess_from = messages_in(m).From;
                    mess_to = messages_in(m).To;
                    mess_type = messages_in(m).Type;
                    mess_subtype = messages_in(m).Subtype;
                    mess_data = messages_in(m).Data;
                    if ~strcmp(mess_from,MY_ID) % not from myself
                        if strcmp(mess_from(1:4),ATOC_TYPE) % from ATOC
                            ATOC.name = mess_from;
                        elseif strcmp(mess_from(1:3),GRS_TYPE) % from GRS
                            GRS.name = mess_from;
                        elseif strcmp(mess_from(1:3),UAS_TYPE) % from UAS
                            [UAS,index] = CS6380_index_UAS(UAS,mess_from);
                        elseif strcmp(mess_from(1:3),USS_TYPE) % from USS
                            [USS,index] = CS6380_index_USS(USS,mess_from);
                            if strcmp(mess_type,AWARD_CON)...
                                    &strcmp(mess_to,MY_ID)&new_award==0
                                new_award = 1;
                                my_flight = mess_data;
                                flight_id = str2num(mess_subtype);
                                realx = my_flight.traj(1,1);
                                realy = my_flight.traj(1,2);
                                realz = my_flight.traj(1,3);
                            end
                        end
                    end
                end
            end
            if CS6380_Ask_clause(KB,[-ASSIGNED])
                if new_award==1
                    KB = CS6380_Tell_clause(KB,[ASSIGNED]);
                    state = SETUP_FLIGHT;
                end
                KB = CS6380_Tell_clause(KB,[IN_LANE]);
                KB = CS6380_Tell_clause(KB,[ON_HEADING]);
                KB = CS6380_Tell_clause(KB,[SPEED_OK]);
            else
                if abs(norm(loc-l_entry)+norm(loc-l_exit)...
                        -norm(l_exit-l_entry))<IN_LANE_THRESH
                    KB = CS6380_Tell_clause(KB,[IN_LANE]);
                else
                    KB = CS6380_Tell_clause(KB,[-IN_LANE]);
                end
                if CS6380_angle_between(set_dir,dir)<ON_HEADING_THRESH
                    KB = CS6380_Tell_clause(KB,[ON_HEADING]);
                else
                    KB = CS6380_Tell_clause(KB,[-ON_HEADING]);
                end
                if abs(speed-set_speed)<SPEED_THRESH
                    KB = CS6380_Tell_clause(KB,[SPEED_OK]);
                else
                    KB = CS6380_Tell_clause(KB,[-SPEED_OK]);
                end
                if abs(loc-l_exit)<AT_NEXT_WAYPT_THRESH
                    KB = CS6380_Tell_clause(KB,[AT_NEXT_WAYPT]);
                else
                    KB = CS6380_Tell_clause(KB,[-AT_NEXT_WAYPT]);
                end
                if cur_lane==num_lanes
                    KB = CS6380_Tell_clause(KB,[LAST_LANE]);
                else
                    KB = CS6380_Tell_clause(KB,[-LAST_LANE]);
                end
            end
        case SETUP_FLIGHT
            state = ANALYZER;
            traj = my_flight.traj;
            l_entry = traj(1,1:3)';
            l_exit = traj(1,4:6)';
            cur_lane = 1;
            num_lanes = length(traj(:,1));
            new_award = 0;
            KB = CS6380_Tell_clause(KB,[AT_START]);
            KB = CS6380_Tell_clause(KB,[-AT_NEXT_WAYPT]);
        case ANALYZER
            state = FILTER;
        case FILTER
            state = EXECUTE_PLAN;
            if CS6380_Ask_clause(KB,[-ASSIGNED])...
                    |cur_time<my_flight.start_time
                plan = P_WAIT;
            else
                mo = CS6380_make_message(BROADCAST,MY_ID,TELEMETRY,...
                    my_flight.id,[xa, ya, za, dx, dy, dz, speed]);
                messages_out = [messages_out;mo];
                if intentions_stack(1)==ASSIGNED
                    [goal,intentions_stack] = CS6380_pop(intentions_stack);
                    intentions_stack = ...
                        CS6380_push(intentions_stack,[NOMINAL]);
                end
                if CS6380_Ask_clause(KB,[-NOMINAL])
                    if CS6380_Ask_clause(KB,[-IN_LANE]);
                        plan = P_GO_TO_LANE;
                    elseif CS6380_Ask_clause(KB,[-ON_HEADING])
                        plan = P_CORRECT_HEADING;
                    else
                        plan = P_CORRECT_SPEED;
                    end
                else
                    if intentions_stack(1)==NOMINAL
                        [goal,intentions_stack]...
                            = CS6380_pop(intentions_stack);
                        intentions_stack = CS6380_push(intentions_stack,...
                            AT_NEXT_WAYPT);
                    end
                    if CS6380_Ask_clause(KB,[-AT_NEXT_WAYPT])
                        plan = P_FOLLOW_LANE;
                    else
                        if cur_lane<num_lanes
                            cur_lane = cur_lane + 1;
                            l_entry = traj(cur_lane,1:3)';
                            l_exit = traj(cur_lane,4:6)';
                            plan = P_FOLLOW_LANE;
                        else 
                            plan = P_WRAP_UP;
                        end
                    end
                end
            end
        case EXECUTE_PLAN
            state = EXIT;
            switch plan
                case P_WAIT
                    state = EXIT;
                case P_GO_TO_LANE
                    [dx,dy,dz] = CS6380_P_GO_TO_LANE(loc,l_entry,l_exit);
                    new_speed = min(MAX_SPEED,norm(l_exit-loc)/del_t);
                case P_CORRECT_HEADING
                    [dx,dy,dz] = CS6380_P_CORRECT_HEADING(loc,l_exit);
                    new_speed = min(MAX_SPEED,norm(l_exit-loc)/del_t);
                case P_CORRECT_SPEED
                    new_speed = CS6380_P_CORRECT_SPEED(set_speed,speed);
                case P_FOLLOW_LANE
                    [dx,dy,dz,new_speed] = CS6380_P_FOLLOW_LANE(loc,...
                        l_exit,del_t,MAX_SPEED);
                case WRAP_UP
                    state = WRAP_UP;
            end
        case WRAP_UP
            state = EXIT;
            intentions_stack = [ASSIGNED];
            KB = CS6380_Tell_clause(KB,[IN_LANE]);
            KB = CS6380_Tell_clause(KB,[ON_HEADING]);
            KB = CS6380_Tell_clause(KB,[SPEED_OK]);
            KB = CS6380_Tell_clause(KB,[-ASSIGNED]);
            KB = CS6380_Tell_clause(KB,[-IN_FLIGHT]);
            KB = CS6380_Tell_clause(KB,[-AT_START]);
            KB = CS6380_Tell_clause(KB,[-AT_NEXT_WAYPT]);
            KB = CS6380_Tell_clause(KB,[-LAST_LANE]);
            mo = CS6380_make_message(MY_USS,MY_ID,MISSION_DONE,...
                num2str(flight_id),[]);
            messages_out = [messages_out;mo];
        case EXIT
            state_vars(4) = dx;
            state_vars(5) = dy;
            state_vars(6) = dz;
            state_vars(7) = new_speed;
            action.dx = state_vars(4);
            action.dy = state_vars(5);
            action.dz = state_vars(6);
            action.speed = state_vars(7);
            action.realx = realx;
            action.realy = realy;
            action.realz = realz;
            action.messages = messages_out;
            set_speed = action.speed;
            set_dir = [action.dx;action.dy;action.dz];
            state = HANDLE_PERCEPTS;
            return
    end
end
