# AUTONOMOUS RACING v8 — Sequential Waypoint Following

## Concept

```
Centerline:  [0] → [1] → [2] → [3] → [4] → [5] → ...
                          ↑           ↑
                         CAR       TARGET
                         
1. Find closest point to car → currentWaypointIdx
2. Determine direction from velocity (+1 or -1)
3. Target = currentWaypointIdx + LOOKAHEAD_POINTS
4. When car reaches waypoint → advance to next
```

## Dual Car Support

| Car       | MAC Address           | Priority |
|-----------|-----------------------|----------|
| RED CAR   | ED:5C:23:84:48:8D    | 1st      |
| GREY CAR  | F9:AF:3C:E2:D2:F5    | 2nd      |

Auto-selection tries RED car first, then GREY car (alternating).
Uses whichever is available.

## How It Works

### Initialization (when car is locked)
```cpp
void initializeDirection(carPos, carVel, centerline) {
    // Find closest point
    closestIdx = findClosest(carPos, centerline);
    
    // Check which adjacent point is "ahead" using velocity
    toNext = centerline[closestIdx + 1] - carPos;
    toPrev = centerline[closestIdx - 1] - carPos;
    
    dotNext = velocity · toNext;
    dotPrev = velocity · toPrev;
    
    direction = (dotNext > dotPrev) ? +1 : -1;
    currentWaypointIdx = closestIdx + direction * LOOKAHEAD;
}
```

### Waypoint Advancement
```cpp
void updateWaypoint(carPos, centerline) {
    target = centerline[currentWaypointIdx];
    dist = distance(carPos, target);
    
    if (dist < WAYPOINT_REACH_DIST) {
        // Close enough! Advance to next waypoint
        currentWaypointIdx += direction;
    }
}
```

### Steering
```cpp
target = centerline[currentWaypointIdx + direction * LOOKAHEAD];
curvature = (2 * sin(alpha)) / Ld;  // Pure Pursuit
steer = atan(curvature * wheelbase);
```

## Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `WAYPOINT_REACH_DIST` | 25px | Switch to next waypoint when this close |
| `LOOKAHEAD_POINTS` | 2 | Target is 2 waypoints ahead of current |
| `STEER_SMOOTH_ALPHA` | 0.65 | Steering smoothing factor |
| `MIN_THROTTLE` | 4 | Always forward motion |

## State Machine

```
SEARCHING → LOCKING → RUNNING
              ↓
         (initialize direction from velocity)
```

## Files

| File | Module | Version |
|------|--------|---------|
| `main.cpp` | Main Controller | v8 |
| `ble_manager.hpp` | BLE Manager | v8 |
| `pure_pursuit_controller.hpp` | Pure Pursuit Controller | v8 |
| `centerline_module.hpp` | Centerline Module | v8 |
| `tracking_module.hpp` | Tracking Module | v8 |
| `safety_monitor.hpp` | Safety Monitor | v8 |

## HUD Display

- **WP:X/Y** — Current waypoint index / total points
- **TGT:Z** — Lookahead target index
- **dir:+1 FWD** — Direction (increasing indices)
- **dir:-1 REV** — Direction (decreasing indices)

## Usage

1. Place car on track pointing in desired direction
2. Car will detect direction from initial velocity
3. Follows waypoints sequentially around the track
