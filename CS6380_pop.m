function [el,stack_out] = CS6380_pop(stack)
%

el = 0;
stack_out = stack;
if isempty(stack)
    return
end

el = stack(1);
stack_out = stack(2:end);
