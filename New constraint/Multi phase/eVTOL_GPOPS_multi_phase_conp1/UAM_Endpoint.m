function output = UAM_Endpoint(input)

% Variables at Start and Terminus of Phase 1
t01 = input.phase(1).initialtime;
tf1 = input.phase(1).finaltime;
x01 = input.phase(1).initialstate;
xf1 = input.phase(1).finalstate;
% Variables at Start and Terminus of Phase 2
t02 = input.phase(2).initialtime;
tf2 = input.phase(2).finaltime;
x02 = input.phase(2).initialstate;
xf2 = input.phase(2).finalstate;

% Event Group 1: Linkage Constraints Between Phases 1 and 2
output.eventgroup(1).event = [x02(1:4)-xf1(1:4), t02-tf1];
%output.objective = input.phase(1).integral;
output.eventgroup(2).event = [x02(1:4), tf2];
% Inputs
% input.phase(phasenumber).initialstate -- row
% input.phase(phasenumber).finalstate -- row
% input.phase(phasenumber).initialtime -- scalar
% input.phase(phasenumber).finaltime -- scalar
% input.phase(phasenumber).integral -- row
%
% input.parameter -- row

% input.auxdata = auxiliary information

% Output
% output.objective -- scalar
% output.eventgroup(eventnumber).event -- row
% latf = input.phase.finalstate(3);

% cost
%output.objective = input.phase(1).integral;
output.objective = input.phase.integral;

