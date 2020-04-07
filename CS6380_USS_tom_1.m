function action = CS6380_USS_tom_1(percept)
% CS6380_USS_tom_1 - USS agent
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
%     action = CS6380_USS_tom_1(percept);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

CS6380_load_ABMS_data;
CS6380_load_USS_tom_1;
MY_ID = 'USS_tom_1';

persistent state USS my_USS_index UAS flights GRS
persistent x_min y_min x_max y_max nx ny grid_request have_grid UTM_grid
persistent UAS_free UAS_assigned UAS_flights

% flights(f).id (int): index in flights
% flights(f).traj (kx6 array): k entry-exit points for k lanes
% flights(f).speed (float): desired speed through lanes
% flights(f).start_time (float): launch time
% flights(f).stop_time (float): arrival at destination
% flights(f).UAS (int): index into UAS_assign array
% flights(f).status (int): status of flight
%   1: intitial unassigned state
%   2: assigned UAS
%   3: unsuccessful at assigning UAS before start_time
%

% UAS(u).name: name of UAS
% UAS(u).domains (unused here)
% UAS(u).flights (flights index)
%

messages_out = [];

if isempty(state)
    state = SETUP_FLIGHTS;
    [USS,my_USS_index] = CS6380_index_USS([],MY_ID);
    UAS = [];
    flights = [];
    grid_request = 0;
    have_grid = 0;
    UAS_free = BASE + [1:num_UAS];
    UAS_assigned = [];
    for ind = 1:length(UAS_free)
        UAS_flights(ind).id = BASE + ind;
        UAS_flights(ind).name = ['UAS_tom_',num2str(UAS_flights(ind).id)];
        UAS_flights(ind).flight = 0;
    end
    messages_out = CS6380_make_message(BROADCAST,MY_ID,ANNOUNCE_SELF,[],[]);
end

del_t = percept.del_t;
cur_time = percept.time;

messages_in = percept.messages;
action.dx = percept.dx;
action.dy = percept.dy;
action.dz = percept.dz;
action.speed = percept.speed;
action.messages = messages_out;

done = 0;
while done==0
    switch state
        case SETUP_FLIGHTS  % assign waiting flights
            state = HANDLE_PERCEPTS;
            [indexes,too_late] = CS6380_get_waiting_flights(flights,...
                cur_time);
            if ~isempty(too_late) % Set unsuccessful flag
                num_too_late = length(too_late);
                for ind = 1:num_too_late
                    flights(too_late(ind)).status = 3;
                end
            end
            num_flights = length(flights);  % free up finished flights
%            for f = 1:num_flights
%                if flights(f).stop_time<=cur_time&flights(f).status==2
%                    flights(f).status = 4;
%                    UAS_num = flights(f).UAS;
%                    UAS_indexes = find(UAS_assigned~=UAS_num);
%                    UAS_assigned = UAS_assigned(UAS_indexes);
%                    UAS_free = [UAS_free,UAS_num];
%                end
%            end
            num_indexes = length(indexes);  %assign waiting flights
            for ind = 1:num_indexes
                f = indexes(ind);
                start_time = flights(f).start_time;
                if ~isempty(UAS_free)
                    index = UAS_free(1);
                    UAS_free = UAS_free(2:end);
                    UAS_assigned = [UAS_assigned,index];
                    flights(f).status = 2;
                    flights(f).UAS = index;
                    UAS(index).flights = [UAS(index).flights,f];
                    mo = CS6380_make_message(UAS(index).name,MY_ID,...
                        AWARD_CON,num2str(f),flights(f));
                    messages_out = [messages_out;mo];
                end
            end
                % consider new flight
            if rand<LAUNCH_PROB&have_grid==1
                x1 = x_min + (x_max-x_min)*rand;
                y1 = y_min + (y_max-y_min)*rand;
                x2 = x_min + (x_max-x_min)*rand;
                y2 = y_min + (y_max-y_min)*rand;
                z = Z_MIN + rand*(Z_MAX - Z_MIN);
                traj = [x1 y1 0 x1 y1 z; x1 y1 z x2 y2 z; x2 y2 z x2 y2 0];
                start_time = cur_time + MIN_START_GAP...
                    + rand*(MAX_START_GAP-MIN_START_GAP);
                speed = randi(5);
                dist = CS6380_traj_dist(traj);
                required_time = dist/speed;
                num_flights = length(flights);
                new_flight.id = num_flights + 1;
                new_flight.traj = traj;
                new_flight.speed = speed;
                new_flight.start_time_or = start_time;
                new_flight.start_time = start_time;
                new_flight.stop_time = start_time + required_time;
                new_flight.UAS = [];
                new_flight.status = 1;
                flights = [flights;new_flight];
                num_flights = num_flights + 1;
                % Assign to next UAS
                if ~isempty(UAS_free)
                    UAS_num = UAS_free(1);
                    index = UAS_num - BASE;
                    UAS_free = UAS_free(2:end);
                    UAS_assigned = [UAS_assigned,index];
                    f = num_flights;
                    flights(f).status = 2;
                    flights(f).UAS = index;
                    UAS(index).flights = [UAS(index).flights,f];
                    mo = CS6380_make_message(UAS(index).name,MY_ID,...
                        AWARD_CON,num2str(UAS(index).flights),flights(f));
                    messages_out = [messages_out;mo];
                end
            end
        case HANDLE_PERCEPTS % handle messages
            state = EXIT;
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
                            tch = 0; % unused for now
                        elseif strcmp(mess_from(1:3),GRS_TYPE) % from GRS
                            if isempty(x_min)&grid_request==0
                                mo = CS6380_make_message(mess_from,...
                                    MY_ID,REQUEST_GRID,[],[]);
                                messages_out = [messages_out;mo];
                                grid_request = 1;
                            elseif isempty(x_min)&grid_request==1 ...
                                    &strcmp(mess_type,GRID)
                                x_min = mess_data(1);
                                y_min = mess_data(2);
                                x_max = mess_data(3);
                                y_max = mess_data(4);
                                nx = mess_data(5);
                                ny = mess_data(6);
                                UTM_grid = mess_data;
                                grid_request = 0;
                                have_grid = 1;
                                GRS.name = mess_from;
                            elseif strcmp(mess_type,USS_GE) % ge_USS_list
                                f = str2num(mess_subtype);
                                flights(f).status = 2;
                                [USS,pairs] = CS6380_extract_pairs(USS,...
                                    mess_data);
                                indexes = find(pairs(:,2)==my_USS_index);
                                pairs(indexes,:) = [];
                                flights(f).USS_response = pairs;
                                if isempty(pairs)
                                    state = 3;
                                    flights(f).status = 3;
                                else
                                    num_pairs = length(pairs(:,1));
                                    USS_list = [];
                                    for p = 1:num_pairs
                                        USS_list = union(USS_list,pairs(p,2));
                                    end
                                    num_USS_list = length(USS_list);
                                    for u = 1:num_USS_list
                                        u_name = USS(USS_list(u)).name;
                                        if ~strcmp(u_name,MY_ID)
                                            ge_list = find(pairs(:,2)==...
                                                USS_list(u));
                                            mo = CS6380_make_message(u_name,...
                                                MY_ID,REQUEST_FLIGHTS,...
                                                num2str(f),ge_list);
                                            messages_out = [messages_out;mo];
                                        end
                                    end
                                end
                            end
                        elseif strcmp(mess_from(1:3),USS_TYPE) % from USS
                            [USS,index] = CS6380_index_USS(USS,mess_from);
                            if strcmp(mess_type,REQUEST_FLIGHTS)
                                ge_list = mess_data;
                                if isempty(flights)
                                    num_flights = 0;
                                else
                                    num_flights = length(flights);
                                end
                                f_flights = [];
                                for f = 1:num_flights
                                    if intersect(ge_list,flights(f).ge_list)
                                        f_flights = [f_flights;flights(f)];
                                    end
                                end
                                mo = CS6380_make_message(mess_from,MY_ID,...
                                    GE_FLIGHTS,mess_subtype,f_flights);
                                messages_out = [messages_out; mo];
                            elseif strcmp(mess_type,GE_FLIGHTS)&strcmp(...
                                    mess_to,MY_ID)
                                f = str2num(mess_subtype);
                                pairs = flights(f).USS_response;
                                indexes = find(pairs(:,2)==index);
                                pairs(indexes,:) = [];
                                flights(f).s_flights = ...
                                    [flights(f).s_flights;mess_data];
                                state = 3;
                            end
                        elseif strcmp(mess_from(1:3),UAS_TYPE) % from UAS
                            % handle UAS
                            [UAS,index] = CS6380_index_UAS(UAS,mess_from);
                            if strcmp(mess_type,BID)&strcmp(mess_to,MY_ID)
                                f_index = str2num(mess_subtype);
                                if flights(f_index).status==3
                                    flights(f_index).bidders = ...
                                        [flights(f_index).bidders;index];
                                    mo = CS6380_make_message(mess_from,...
                                        MY_ID,ACCEPT_BID,mess_subtype,[]);
                                    messages_out = [messages_out;mo];
                                else
                                    mo = CS6380_make_message(mess_from,...
                                        MY_ID,REJECT_BID,mess_subtype,[]);
                                    messages_out = [messages_out;mo];
                                end
                            elseif strcmp(mess_type,ACCEPT_CON)...
                                    &strcmp(mess_to,MY_ID)
                                f_index = str2num(mess_subtype);
                                if flights(f_index).status==3
                                    flights(f_index).status = 4;
                                    flights(f_index).USS = index;
                                    mo = CS6380_make_message(mess_from,...
                                        MY_ID,AWARD_CON,mess_subtype,...
                                        flights(f_index));
                                    messages_out = [messages_out;mo];
                                    bidders = flights(f_index).bidders;
                                    num_bidders = length(bidders);
                                    for b = 1:num_bidders
                                        if index~=bidders(b)
                                            mo = CS6380_make_message(...
                                                UAS(bidders(b)).name,...
                                                MY_ID,REJECT_CON,...
                                                mess_subtype,[]);
                                            messages_out = [messages_out;...
                                                mo];
                                        end
                                    end
                                else
                                    mo = CS6380_make_message(mess_from,...
                                        MY_ID,REJECT_CON,mess_subtype,[]);
                                    messages_out = [messages_out;mo];
                                end
                            elseif strcmp(mess_type,MISSION_DONE)
                                f = UAS(index).flights(end);
                                flights(f).status = 4;
                                UAS_num = flights(f).UAS;
                                UAS_indexes = find(UAS_assigned~=UAS_num);
                                UAS_assigned = UAS_assigned(UAS_indexes);
                                UAS_free = [UAS_free,UAS_num];
                            elseif strcmp(mess_type,TELEMETRY)...
                                    &strcmp(mess_to,MY_ID)
                                f_index = str2num(mess_subtype);
                                flights(f_index).telemetry = mess_data;
                            end
                        end
                    end
                end
            end
        case CALL_FOR_BIDS % CFB
            state = EXIT;
            break
            flights(f).USS_response = pairs;
            if isempty(pairs) % deconflict and send CFB
                [dec,start_time] = CS6380_deconflict(...
                    flights(f).s_flights,flights(f),del_t,H_T);
                flights(f).start_time = start_time;
                flights(f).status==3;
                mo = CS6380_make_message(BROADCAST,MY_ID,CFB,num2str(f),...
                    flights(f));
                messages_out = [messages_out;mo];
            end
        case EXIT % exit state
            action.messages = messages_out;
            state = SETUP_FLIGHTS;
            done = 1;
    end
end
