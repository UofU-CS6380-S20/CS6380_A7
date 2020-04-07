function [indexes,too_late] = CS6380_get_waiting_flights(flights,cur_time)
%

indexes = [];
s_times = [];
too_late = [];
if isempty(flights)
    return
end

num_flights = length(flights);
for f = 1:num_flights
    if flights(f).status==1
        if flights(f).start_time<=cur_time
            too_late = [too_late,f];
        else
            indexes = [indexes,f];
            s_times = [s_times,flights(f).start_time];
        end
    end
end

if ~isempty(indexes)
    [vals,sindexes] = sort(s_times);
    indexes = indexes(sindexes);
end
