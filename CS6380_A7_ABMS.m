function [res,M] = CS6380_A7_ABMS(fnames,ports,max_t,del_t,draw,film,dump)
% CS6380_A7_ABMS - A7 ABMS simulator
% On input:
%     fnames (struct vector): names of agent function (filenames)
%     ports (nx2 array): launch/land ports (x1 y1)
%     max_t (float): max time to simulate
%     del_t (float): time step increment
%     draw (Boolean): display each simulation step
%     film (Boolean): make a movie from data
% On output:
%     res (px9 array): info for each agent (same as internal "agent")
%          col 1: agent type (1: USS; 2: UAS: 3: ATOC)
%          col 2: x coord
%          col 3: y coord
%          col 4: z coord
%          col 5: dx heading in x
%          col 6: dy heading in y
%          col 7: dz heading in z
%          col 8: ground speed
%          col 9: last time called
% Call:
%     r1 = CS6380_A7_ABMS(fnames,ports,max_t,del_t,0,0);
% Author:
%     T. Henderson
%     UU
%     Spring 2020
%

CS6380_load_ABMS_data;
X_MIN = min(ports(:,1));
X_MAX = max(ports(:,1));
Y_MIN = min(ports(:,2));
Y_MAX = max(ports(:,2));
MY_ID = 'SIM_tom_1';
DRAW = 'DRAW';
FILM = 'FILM';
SEND_FILM = 'SEND_FILM';

M = [];

cur_time = 0;
[num_ports,dummy] = size(ports);
num_agents = length(fnames);
agents = zeros(num_agents,9); % Same as res output variable
for a = 1:num_agents
    a_name = fnames(a).name;
    if strcmp(a_name(8:10),'USS')
        agents(a,1) = 1;
    elseif strcmp(a_name(8:10),'UAS')
        agents(a,1) = 2;
    elseif strcmp(a_name(8:11),'ATOC')
        agents(a,1) = 3;
    end
    agents(a,9) = cur_time;
end

messages_in = [];
messages_out = [];
count = 0;
wb = waitbar(0,'Run ABMS A4');
if dump==1
    fd = fopen('popo','w');
end

% Talk to ATOC
for a = 1:num_agents
    if strcmp(fnames(a).name(8:11),ATOC_TYPE)
        ATOC_index = a;
        mess_to = fnames(a).name(8:end);
    end
end
if draw==1  % tell ATOC to draw each iteration
    mi = CS6380_make_message(mess_to,MY_ID,'DRAW',[],[]);
    messages_in = [messages_in;mi];
end
if film==1  % tell ATOC to produce film of simulation
    mi = CS6380_make_message(mess_to,MY_ID,'FILM',[],[]);
    messages_in = [messages_in;mi];
end

while cur_time<=max_t
    cur_time = cur_time + del_t
    count = count + 1;
    waitbar(cur_time/max_t);
    messages_out = messages_in;
    messages_in = [];
    for a = 1:num_agents
        % Set up percept
        xa = agents(a,2);
        ya = agents(a,3);
        za = agents(a,4);
        dx = agents(a,5);
        dy = agents(a,6);
        dz = agents(a,7);
        sa = agents(a,8);
        ta = agents(a,9);
        percept.x = xa;
        percept.y = ya;
        percept.z = za;
        percept.dx = dx;
        percept.dy = dy;
        percept.dz = dz;
        percept.speed = sa;
        percept.time = cur_time;
        percept.del_t = del_t;
        percept.messages = messages_out;
        % Call agent
        action = feval(fnames(a).name,percept);
        mess = action.messages;
        af = fieldnames(action);
        if length(af)<6
            action.realx = [];
            action.realy = [];
            action.realz = [];
        end
        % Update world
        actions(a) = action;
        percepts(a) = percept;
        messages_in = [messages_in;action.messages];
    end
    % update agent state (execute actions)
    if dump==1
        CS6380_dump_messages(fd,count,messages_in);
    end
    for a = 1:num_agents
        if agents(a,1)==2
            % update real location
            if ~isempty(actions(a).realx)
                agents(a,2) = actions(a).realx;
                agents(a,3) = actions(a).realy;
                agents(a,4) = actions(a).realz;
            end
            % update heading
            dx = actions(a).dx;
            dy = actions(a).dy;
            dz = actions(a).dz;
            agents(a,5) = dx;
            agents(a,6) = dy;
            agents(a,7) = dz;
            % update speed
            speed = max(min(MAX_SPEED,actions(a).speed),0);
            % account for rain
            xe = agents(a,2);
            ye = agents(a,3);
            ze = agents(a,4);
            intensity = CS6380_rain(xe,ye,ze,cur_time);
            speed = (1-intensity/2)*speed;
            agents(a,8) = speed;
            % update position
            xa = agents(a,2);
            ya = agents(a,3);
            za = agents(a,4);
            % account for wind
            pt = agents(a,2:4)' + speed*[dx;dy;dz]*del_t;
            [mag,dir] = CS6380_wind(xa,ya,za,cur_time);
            pt = pt + ((1-exp(-mag))/2)*dir;
            pt(3) = max(0,pt(3));
            agents(a,2) = pt(1);
            agents(a,3) = pt(2);
            agents(a,4) = pt(3);
        else
            tch = 0;
        end
    end
    if max_t-cur_time<0.1
        tch = 0;
    end
end
close(wb);
res = agents;
if dump==1
    fclose(fd);
end
if film==1
    mo = CS6380_make_message(mess_to,MY_ID,SEND_FILM,[],[]);
    percept.x = 0;
    percept.y = 0;
    percept.z = 0;
    percept.dx = 0;
    percept.dy = 0;
    percept.dz = 0;
    percept.speed = 0;
    percept.time = cur_time;
    percept.del_t = del_t;
    percept.messages = mo;
    action = feval(fnames(ATOC_index).name,percept);
    M = action.messages(1).Data;
end
