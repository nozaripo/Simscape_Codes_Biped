
function [state, options,optchanged] = outputFCN(options,state,flag)
%displays the function eval value at each iteration. You can change this
% disp(state.FunEval);
persistent output
optchanged = false;
switch flag
%  case 'init'
%         disp('Starting the algorithm');
    case {'iter'}
        ind_best = find(state.Score==min(state.Score));
%         disp('Iterating ...')
        plot(state.Generation,state.Population(min(ind_best),1),'.r')
        hold on
        plot(state.Generation,state.Population(min(ind_best),2),'.g')
        plot(state.Generation,state.Population(min(ind_best),3:end),'.b')
        legend('\phi_0','\tau','A')
        xlabel('Generation')
        ylabel('Best Individuals')
%     case 'done'
%         disp('Performing final task');
output(state.Generation,:) = state.Population(min(ind_best),:);
assignin('base','GenBest',output);
end