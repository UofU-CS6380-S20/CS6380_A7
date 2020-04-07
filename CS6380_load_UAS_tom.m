
% state names (for state machine)

HANDLE_PERCEPTS = 1;
SETUP_FLIGHT = 2;
ANALYZER = 3;
FILTER = 4;
EXECUTE_PLAN = 5;
S_WRAPUP = 6;
EXIT = 7;

% logical atoms

num_atoms = 12;

NOMINAL = 1;
IN_LANE = 2;
ON_HEADING = 3;
SPEED_OK = 4;
ASSIGNED = 5;
IN_FLIGHT = 6;
AT_START = 7;
AT_FINISH = 8;
AT_NEXT_WAYPT = 9;
LAST_LANE = 10;
ADVANCE_LANE = 11;
WRAP_UP = 12;

% Behavior variables

IN_LANE_THRESH = 1;
ON_HEADING_THRESH = 10*pi/180;  % 10 degrees
MAX_SPEED = 10;
SPEED_THRESH = 1;
AT_NEXT_WAYPT_THRESH = 0.1;
MY_USS = 'USS_tom_1';

% plan variables

P_WAIT = 1;
P_GO_TO_LANE = 2;
P_CORRECT_HEADING = 3;
P_CORRECT_SPEED = 4;
P_FOLLOW_LANE = 5;
P_WRAP_UP = 6;
