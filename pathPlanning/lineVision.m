function [detected, cx, cy, dirX, dirY, areaFrac, atMarker, ...
          projMax, projMin, tipAu, tipAv, tipBu, tipBv] = lineVision(R, G, B)
%LINEVISION  Detect the blue track line and report its geometry.
%
%   Reads the down-facing camera image and returns where the line is, which
%   way it runs, and a look-ahead "tip" at each end of the visible line. The
%   path planner steers toward the forward tip (pure pursuit), which lets it
%   round corners instead of stalling on them. Keeps the lightweight
%   B - R/2 - G/2 blueness test.
%
%   Put this in a MATLAB Function block right after "PARROT Image
%   Conversion" and wire the R, G, B channel outputs into it. The outputs
%   feed pathPlannerSF().
%
%   INPUTS  (each H-by-W; camera is 120x160; uint8 or single both fine)
%     R, G, B : colour channels of the down-facing camera image
%
%   OUTPUTS (all scalar, code-generation friendly)
%     detected : true if a line is in view
%     cx, cy   : line centroid, normalised to [-1,1] (handy for logging)
%     dirX,dirY: unit vector along the line in image axes (principal axis,
%                180-degree ambiguous; the planner resolves the sense)
%     areaFrac : fraction of the frame that is blue (0..1)
%     atMarker : true for a large, round blue blob (a solid landing pad)
%     projMax,projMin : how far the line reaches along its axis, ahead (+)
%                and behind (-) of frame centre (used for end-of-track)
%     tipAu,tipAv : look-ahead point at the projMax end of the line ([-1,1])
%     tipBu,tipBv : look-ahead point at the projMin end of the line ([-1,1])
%
%   Calibrate BLUE_THRESH in simulation so it isolates the actual track
%   colour. The image-to-world sign mapping is handled in the planner.

%% ---- Tunables -----------------------------------------------------------
BLUE_THRESH = single(50);     % per-pixel blueness cut
MIN_AREA    = single(0.003);  % >~0.3% blue  => a line is present
END_AREA    = single(0.18);   % >~18% blue   => candidate landing pad
BLOB_RATIO  = single(0.60);   % axis ratio above this => round (pad, not line)
TIP_BAND    = single(0.35);   % fraction of the line length used for each tip

%% ---- Blueness mask -------------------------------------------------------
M = single(B) - 0.5*single(R) - 0.5*single(G);   % B - R/2 - G/2
mask  = M > BLUE_THRESH;
maskf = single(mask);

H = size(mask,1);  W = size(mask,2);
m00 = sum(maskf(:));
areaFrac = m00 / single(H*W);

detected = areaFrac > MIN_AREA;
cx = single(0);  cy = single(0);
dirX = single(0); dirY = single(-1);
atMarker = false;
projMax = single(0); projMin = single(0);
tipAu = single(0); tipAv = single(0); tipBu = single(0); tipBv = single(0);
if ~detected
    return;
end

%% ---- Normalised pixel-coordinate grids ----------------------------------
denU = single(max(W-1,1)) / 2;   % guard a degenerate 1-pixel dimension
denV = single(max(H-1,1)) / 2;
uAxis = (single(0:W-1) - single(W-1)/2) / denU;   % -1..1 across cols
vAxis = (single(0:H-1).' - single(H-1)/2) / denV; % -1..1 down rows
U = repmat(uAxis, H, 1);
V = repmat(vAxis, 1, W);

%% ---- Centroid (first moments) -------------------------------------------
cx = sum(sum(U.*maskf)) / m00;
cy = sum(sum(V.*maskf)) / m00;

%% ---- Central moments -> principal axis and shape ------------------------
dU = U - cx;  dV = V - cy;
mu20 = sum(sum((dU.*dU).*maskf)) / m00;
mu02 = sum(sum((dV.*dV).*maskf)) / m00;
mu11 = sum(sum((dU.*dV).*maskf)) / m00;

theta = 0.5*atan2(2*mu11, mu20 - mu02);   % orientation of the line
dirX = cos(theta);
dirY = sin(theta);

% Elongation (~1 round blob, ~0 thin line), measured in the same normalised
% frame as the rest of the pipeline so L-corners stay classified as lines.
tr   = mu20 + mu02;
det2 = mu20*mu02 - mu11*mu11;
disc = sqrt(max(tr*tr/4 - det2, single(0)));
lam1 = tr/2 + disc;
lam2 = tr/2 - disc;
ratio = single(0);
if lam1 > single(1e-9)
    ratio = lam2 / lam1;
end
atMarker = (areaFrac > END_AREA) && (ratio > BLOB_RATIO);

%% ---- Extent and look-ahead tips along the line axis ---------------------
P = U.*dirX + V.*dirY;       % projection of every pixel onto the l