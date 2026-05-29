function [posRef, mode] = pathPlannerSF(detected, dirX, dirY, atMarker, ...
        projMax, projMin, tipAu, tipAv, tipBu, tipBv, xEst, yEst, zEst)
%PATHPLANNERSF  Line-following path planner for the Parrot Minidrone.
%
%   Generates the position reference that walks the drone along the detected
%   line and rounds corners. Written as a MATLAB Function so it drops into
%   the Path Planning subsystem, and it mirrors the Stateflow chart in
%   INTEGRATION.md one-to-one (build the chart for the judges, or use this
%   directly).
%
%   Method (pure pursuit): the camera looks straight down and the drone holds
%   yaw = 0. lineVision gives a look-ahead "tip" at each end of the visible
%   line; the planner picks the forward tip (by continuity with its current
%   heading) and steps the POSITION reference toward it. Steering toward the
%   tip ahead -- rather than along the local average direction -- is what
%   carries the drone around a corner instead of stalling at it.
%
%   Validated in a kinematic sim: a straight track, a single 90-degree
%   corner, and a two-corner staircase are all followed and landed.
%
%   INPUTS
%     detected,dirX,dirY,atMarker,projMax,projMin,tipAu,tipAv,tipBu,tipBv
%        : from lineVision()
%     xEst,yEst,zEst : estimated position (NED, m) from the State Estimator
%
%   OUTPUTS
%     posRef : [x; y; z] position reference (NED, single) -> Bus 'pos_ref'
%     mode   : current state as a number (log it to watch the behaviour)
%
%   >>> CALIBRATE ON TRACK 1 FIRST (the rules require Track 1 to work) <<<
%   On the straight track: if the drone slides off sideways, flip SX; if it
%   crawls the wrong way along the line, flip SY. Once a straight line works,
%   the turning tracks follow with no further sign changes.

%% ---- States --------------------------------------------------------------
TAKEOFF = uint8(1);  SEARCH = uint8(2);  FOLLOW = uint8(3);
TURN    = uint8(4);  RECOVER= uint8(5);  LAND   = uint8(6);

%% ---- Tunables ------------------------------------------------------------
DT        = single(0.2);    % planner step = vision sample time VTs (5 Hz)
HOVER_ALT = single(1.1);    % flight height (m); refZ = -HOVER_ALT in NED
ALT_TOL   = single(0.15);   % "high enough" band
SETTLE_N  = single(10);     % steps to settle before searching

VMAX   = single(0.42);      % forward speed on a clear stretch (m/s)
VMIN   = single(0.24);      % floor speed (keeps moving through bends)
SLOWING= single(0.5);       % how much a sideways tip slows the drone (0..1)
TURN_TIP = single(0.5);     % |tip sideways| above this is flagged as a TURN

FWD_MIN   = single(0.30);   % line must reach this far ahead, else end-of-track
END_N     = single(6);      % sustained "nothing ahead" steps => land
CLIMB  = single(0.6);       % climb rate (m/s)
DESC   = single(0.4);       % descent rate while landing (m/s)
LOST_N    = single(8);      % steps with no line before RECOVER
RECOVER_N = single(40);     % steps lost before giving up and landing

% Image -> world axis sign mapping (defaults assume image-right -> world +X,
% image-up/ahead -> world +Y). Flip a sign if Track 1 steers the wrong way.
SX = single(1);
SY = single(1);

%% ---- Persistent memory ---------------------------------------------------
persistent state refX refY refZ lostC settleC headX headY endC
if isempty(state)
    state   = TAKEOFF;
    refX    = single(xEst);
    refY    = single(yEst);
    refZ    = single(zEst);
    lostC   = single(0);
    settleC = single(0);
    endC    = single(0);
    headX   = single(0);
    headY   = single(0);
end

%% ---- State machine -------------------------------------------------------
switch state
    case TAKEOFF
        refZ = moveToward(refZ, -HOVER_ALT, CLIMB*DT);
        if zEst <= -(HOVER_ALT - ALT_TOL)
            settleC = settleC + 1;
            if settleC >= SETTLE_N
                state = SEARCH;  settleC = single(0);
            end
        else
            settleC = single(0);
        end

    case SEARCH
        if detected
            state = FOLLOW;  lostC = single(0);  endC = single(0);
            headX = single(0);  headY = single(0);
        end

    case {FOLLOW, TURN}
        if atMarker
            state = LAND;                       % solid round pad -> land
        elseif detected
            lostC = single(0);
            aX = dirX;  aY = dirY;

            % Pick the forward end of the line. On the first step there is no
            % history, so head toward the longer tail; afterwards keep going
            % the same way (continuity), never reversing.
            if (headX == 0) && (headY == 0)
                fwdPlus = (projMax + projMin) >= 0;
            else
                fwdPlus = (aX*headX + aY*headY) >= 0;
            end
            if fwdPlus
                fX = aX;  fY = aY;  tipu = tipAu;  tipv = tipAv;  fwdExtent = projMax;
            else
                fX = -aX; fY = -aY; tipu = tipBu;  tipv = tipBv;  fwdExtent = -projMin;
            end
            headX = fX;  headY = fY;

            % End of track: the line no longer reaches ahead.
            if fwdExtent < FWD_MIN
                endC = endC + 1;
            else
                endC = single(0);
            end
            if endC >= END_N
                state = LAND;
            else
                % Pure pursuit: step toward the look-ahead tip (image->world).
                gx = SX*tipu;  gy = SY*(-tipv);
                n = sqrt(gx*gx + gy*gy);
                if n < single(1e-6)
                    gx = SX*fX;  gy = SY*(-fY);  n = single(1);
                end
                gx = gx/n;  gy = gy/n;
                sideways = abs(tipu);                 % how sharp the bend is
                spd = max(VMAX*(single(1) - SLOWING*min(single(1), sideways)), VMIN);
                if sideways > TURN_TIP
                    state = TURN;
                else
                    state = FOLLOW;
                end
                refX = refX + spd*DT*gx;
                refY = refY + spd*DT*gy;
            end
        else
            lostC = lostC + 1;                  % hover in place while lost
            if lostC > LOST_N
                state = RECOVER;
            end
        end

    case RECOVER
        if detected
            state = FOLLOW;  lostC = single(0);
        else
            lostC = lostC + 1;
            if lostC > RECOVER_N
                state = LAND;
            end
        end

    case LAND
        refZ = moveToward(r