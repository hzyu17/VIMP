function [] = execute_one_exp(executable, arguments)
%EXECUTE_ONE_EXP Execute one experiment, given the compiled file and the
%                arguments, for computing the time.
% Hongzhe Yu

system([executable, ' ', arguments]);

end

