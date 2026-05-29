# Path planning: design and how to wire it in

This explains the two algorithm blocks (`lineVision.m`, `pathPlannerSF.m`), the Stateflow chart they correspond to, the exact changes to make inside `controller/flightControlSystem.slx`, and how to test track by track. The logic was checked in a kinematic simulation first: a straight track, a single 90-degree corner, and a two-corner staircase are all followed and landed.

## The idea in one line

Have the vision report a look-ahead point at the end of the line ahead, then let a small state machine drive the position reference toward that point (pure pursuit), which carries the drone around corners instead of stalling at them.

## Block 1 — `lineVision.m` (image processing)

Keeps the `B - R/2 - G/2` blueness test, then reads the geometry of the mask:

- centroid `(cx, cy)` — where the line sits in the frame (handy for logging);
- principal axis `(dirX, dirY)` — the line's direction from image moments;
- `areaFrac` plus a roundness test — a large round blob is a landing pad, while a large thin shape is just a corner, so corners are not mistaken for the end;
- `projMax/projMin` — how far the line reaches ahead and behind along its own axis, which is how the planner knows the track has run out;
- `tipAu,tipAv` and `tipBu,tipBv` — a look-ahead point at each end of the visible line. The planner steers toward whichever end is "forward". These are what make corner-following work.

Place it in a MATLAB Function block right after `PARROT Image Conversion` and wire the R, G, B channel outputs into it. It produces thirteen scalar outputs.

## Block 2 — `pathPlannerSF.m` (the state machine)

The drone holds yaw at 0 and the camera looks straight down. Each step the planner picks the forward end of the line (by continuity with its current heading) and steps the position reference toward that look-ahead tip. Because it aims where the line is going rather than along the local average direction, it rounds bends smoothly.

### State chart

```
        ┌─────────┐  height reached & settled    ┌────────┐  line seen   ┌────────┐
        │ TAKEOFF │ ───────────────────────────▶ │ SEARCH │ ───────────▶ │ FOLLOW │
        └─────────┘                               └────────┘              └───┬────┘
             ▲ climb to -1.1 m                         ▲ hover, wait          │ tip swings
        (start)                                        │ line seen            ▼  sideways
                                  ┌──────────┐         │                  ┌────────┐
                                  │ RECOVER  │ ────────┘                  │  TURN  │
                                  └────┬─────┘  reacquired                └───┬────┘
                                       ▲ line lost > 8                        │ tip re-centred
        FOLLOW/TURN ───────────────────┘                                     │ (back to FOLLOW)
             │  landing pad  OR  nothing-ahead sustained                      │
             ▼                                                                │
         ┌──────┐  ◀───────────────────────────────────────────────────────── ┘
         │ LAND │  descend to ground
         └──────┘
```

### Transition table

| From | Condition | To |
|------|-----------|----|
| TAKEOFF | at height for `SETTLE_N` steps | SEARCH |
| SEARCH | line detected | FOLLOW |
| FOLLOW | forward tip swings sideways (`|tipu| > TURN_TIP`) | TURN |
| TURN | tip re-centres | FOLLOW |
| FOLLOW/TURN | round landing blob (`atMarker`) | LAND |
| FOLLOW/TURN | line does not reach ahead (`fwdExtent < FWD_MIN`) for `END_N` steps | LAND |
| FOLLOW/TURN | no line at all for `LOST_N` steps | RECOVER |
| RECOVER | line detected | FOLLOW |
| RECOVER | still lost after `RECOVER_N` steps | LAND |

The reference-walk each FOLLOW/TURN step, where `(tipu,tipv)` is the forward look-ahead tip in image axes:

```
g     = normalise( [ SX*tipu ; SY*(-tipv) ] )   % direction to the tip, in world
speed = max( VMAX*(1 - SLOWING*min(1,|tipu|)), VMIN )   % ease off into a bend
refX += speed*dt*g(1)
refY += speed*dt*g(2)
```

`pathPlannerSF.m` is this chart in code. To present a Stateflow algorithm for the judging, rebuild the six states above as a chart and paste the action code into the state actions.

## Wiring it into `flightControlSystem.slx`

1. **Image Processing System.** After `PARROT Image Conversion`, replace the pixel-count tail with one MATLAB Function block running `lineVision`. Wire R, G, B into it. It now produces thirteen outputs.
2. **Carry the signals through.** Replace the old single vision line with the `lineVision` outputs — the easy way is to `Mux` them into one vector along the same route into `Path Planning` and `Demux` inside. Update the two subsystem port signatures to pass the vector.
3. **Path Planning.** Replace the contents with one MATLAB Function block running `pathPlannerSF`. It uses `detected, dirX, dirY, atMarker, projMax, projMin` and the four tip coordinates from `lineVision`, plus `xEst, yEst, zEst` taken from the `EstimatedVal` bus already entering this block (add a `BusSelector` for `X, Y, Z`). `cx, cy, areaFrac` are not needed by the planner but are useful to scope. Its `posRef` output goes into the existing `Bus Assignment` on the `pos_ref` signal. Send `mode` to a scope to watch the state.
4. **Keep yaw fixed** at 0. The follower does not need to rotate.

Nothing else changes — the PID controller, estimator, mixer, airframe, camera and 3D scene stay as they are.

## Settings to tune (top of each file)

| Name | File | Default | Purpose |
|------|------|---------|---------|
| `BLUE_THRESH` | lineVision | 50 | blueness cut; match the real track colour |
| `END_AREA` / `BLOB_RATIO` | lineVision | 0.18 / 0.60 | what counts as a landing pad |
| `TIP_BAND` | lineVision | 0.35 | how much of each end is averaged into a tip |
| `VMAX` / `VMIN` | pathPlannerSF | 0.42 / 0.24 | forward speed, clear / through a bend (m/s) |
| `SLOWING` / `TURN_TIP` | pathPlannerSF | 0.5 / 0.5 | how much a sideways tip slows it / flags a TURN |
| `FWD_MIN` / `END_N` | pathPlannerSF | 0.30 / 6 | end-of-track sensitivity |
| `SX` / `SY` | pathPlannerSF | 1 / 1 | image-to-world sign mapping |

## Test order

1. **Track 1 (straight) first** — the rules require it. If the drone drifts sideways off the line, flip `SX`; if it crawls the wrong way along the line, flip `SY`. Tune `BLUE_THRESH` until the mask is clean. It should follow the line and land at the end.
2. **A corner.** It should follow the first leg, ease into the bend, round it, follow the second leg, and land. If it cuts the corner, lower `VMAX` or raise `SLOWING`; if it creeps too slowly on the second leg, raise `VMIN`.
3. **Sharper / multi-turn tracks.** If