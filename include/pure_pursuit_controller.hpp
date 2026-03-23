#pragma once
// ============================================================
// PURE PURSUIT CONTROLLER  v8  —  Waypoint Progression
//
// CONCEPT:
//   - Car starts at closest centerline point (index 0)
//   - Target is NEXT point ahead (index + 1)
//   - When car gets close to target → advance to next waypoint
//   - Always moving FORWARD through the centerline
//
// NO direction calibration needed — we simply progress forward
// through waypoint indices as the car moves.
// ============================================================

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace cv;
using namespace std;

class PurePursuitController {
public:
    // ══════════════════════════════════════════════════════════
    // PURE PURSUIT PARAMETERS
    // ══════════════════════════════════════════════════════════
    static constexpr float PP_WHEELBASE    = 38.0f;
    static constexpr float MIN_LOOKAHEAD   = 30.0f;
    static constexpr float MAX_LOOKAHEAD   = 80.0f;

    // ══════════════════════════════════════════════════════════
    // WAYPOINT PROGRESSION
    // ══════════════════════════════════════════════════════════
    static constexpr float WAYPOINT_REACH_DIST = 25.0f;
    static constexpr int   LOOKAHEAD_POINTS    = 3;

    // ══════════════════════════════════════════════════════════
    // STEERING CONSTANTS
    // ══════════════════════════════════════════════════════════
    static constexpr int   STEER_CENTER    = 120;
    static constexpr int   STEER_RIGHT     = 40;
    static constexpr int   STEER_LEFT      = 200;
    static constexpr int   STEER_RANGE     = 80;

    // ══════════════════════════════════════════════════════════
    // THROTTLE CONSTANTS
    // ══════════════════════════════════════════════════════════
    static constexpr int   MIN_THROTTLE       = 4;
    static constexpr int   THR_REVERSE_GENTLE = 215;

    // ══════════════════════════════════════════════════════════
    // SMOOTHING
    // ══════════════════════════════════════════════════════════
    static constexpr float STEER_SMOOTH_ALPHA = 0.65f;

    // ══════════════════════════════════════════════════════════
    // STATE
    // ══════════════════════════════════════════════════════════
    int    currentWaypointIdx = -1;
    int    targetWaypointIdx  = -1;
    float  smoothSteer        = (float)STEER_CENTER;
    Point  velocity{0, 0};
    Point  predVel{0, 0};
    bool   initialized        = false;
    int    direction          = 1;    // +1 or -1, auto-determined

    void reset() {
        currentWaypointIdx = -1;
        targetWaypointIdx  = -1;
        smoothSteer        = (float)STEER_CENTER;
        initialized        = false;
        direction          = 1;
    }

    // ══════════════════════════════════════════════════════════
    // INITIALIZATION — Find starting waypoint
    // ══════════════════════════════════════════════════════════
    void initializeAtPosition(const Point& carPos, const vector<Point>& cl) {
        if (cl.empty()) return;
        
        double bestDist = 1e9;
        int bestIdx = 0;
        for (int i = 0; i < (int)cl.size(); i++) {
            double d = norm(cl[i] - carPos);
            if (d < bestDist) {
                bestDist = d;
                bestIdx = i;
            }
        }
        
        currentWaypointIdx = bestIdx;
        targetWaypointIdx = (bestIdx + 1) % (int)cl.size();
        initialized = true;
        
        cout << "[PP] Initialized at waypoint " << currentWaypointIdx 
             << ", target = " << targetWaypointIdx << endl;
    }

    // ══════════════════════════════════════════════════════════
    // DETERMINE DIRECTION from initial movement
    // ══════════════════════════════════════════════════════════
    void determineDirection(const Point& carPos, const vector<Point>& cl) {
        if (cl.empty() || currentWaypointIdx < 0) return;
        
        int n = (int)cl.size();
        int nextIdx = (currentWaypointIdx + 1) % n;
        int prevIdx = (currentWaypointIdx - 1 + n) % n;
        
        Point toNext = cl[nextIdx] - carPos;
        Point toPrev = cl[prevIdx] - carPos;
        
        float vx = (float)velocity.x;
        float vy = (float)velocity.y;
        float speed = sqrtf(vx*vx + vy*vy);
        
        if (speed > 1.5f) {
            float dotNext = vx * toNext.x + vy * toNext.y;
            float dotPrev = vx * toPrev.x + vy * toPrev.y;
            
            if (dotPrev > dotNext) {
                direction = -1;
                targetWaypointIdx = prevIdx;
                cout << "[PP] Direction: -1 (CCW)" << endl;
            } else {
                direction = 1;
                targetWaypointIdx = nextIdx;
                cout << "[PP] Direction: +1 (CW)" << endl;
            }
        }
    }

    // ══════════════════════════════════════════════════════════
    // UPDATE WAYPOINT — Advance when car reaches target
    // ══════════════════════════════════════════════════════════
    void updateWaypoint(const Point& carPos, const vector<Point>& cl) {
        if (cl.empty() || targetWaypointIdx < 0) return;
        
        int n = (int)cl.size();
        float distToTarget = (float)norm(cl[targetWaypointIdx] - carPos);
        
        if (distToTarget < WAYPOINT_REACH_DIST) {
            currentWaypointIdx = targetWaypointIdx;
            targetWaypointIdx = (currentWaypointIdx + direction + n) % n;
        }
    }

    // ══════════════════════════════════════════════════════════
    // GET LOOKAHEAD TARGET
    // ══════════════════════════════════════════════════════════
    Point getLookaheadTarget(const Point& carPos, const vector<Point>& cl, 
                             float lookaheadDist, int* outIdx = nullptr) {
        if (cl.empty() || targetWaypointIdx < 0) {
            if (outIdx) *outIdx = 0;
            return carPos;
        }
        
        int n = (int)cl.size();
        float accumDist = 0.0f;
        int idx = targetWaypointIdx;
        Point prevPt = carPos;
        
        for (int step = 0; step < LOOKAHEAD_POINTS * 3; step++) {
            accumDist += (float)norm(cl[idx] - prevPt);
            if (accumDist >= lookaheadDist) break;
            prevPt = cl[idx];
            idx = (idx + direction + n) % n;
        }
        
        if (outIdx) *outIdx = idx;
        return cl[idx];
    }

    // ══════════════════════════════════════════════════════════
    // COMPUTE CURVATURE — κ = (2 * sin(α)) / Ld
    // ══════════════════════════════════════════════════════════
    float computeCurvature(Point2f car_pos, Point2f target, float yaw) const {
        float dx = target.x - car_pos.x;
        float dy = target.y - car_pos.y;
        float target_angle = atan2f(dy, dx);
        float alpha = target_angle - yaw;

        while (alpha > (float)M_PI)  alpha -= 2.0f * (float)M_PI;
        while (alpha < -(float)M_PI) alpha += 2.0f * (float)M_PI;

        float Ld = sqrtf(dx * dx + dy * dy);
        if (Ld < 1e-5f) return 0.0f;

        return (2.0f * sinf(alpha)) / Ld;
    }

    // ══════════════════════════════════════════════════════════
    // ESTIMATE YAW from velocity
    // ══════════════════════════════════════════════════════════
    float estimateYaw() const {
        float vx = (float)(predVel.x != 0 ? predVel.x : velocity.x);
        float vy = (float)(predVel.y != 0 ? predVel.y : velocity.y);
        float speed = sqrtf(vx * vx + vy * vy);
        if (speed > 1.0f) return atan2f(vy, vx);
        return 0.0f;
    }

    // ══════════════════════════════════════════════════════════
    // ADAPTIVE LOOKAHEAD
    // ══════════════════════════════════════════════════════════
    float adaptiveLookahead() const {
        float speed = sqrtf((float)(velocity.x * velocity.x + velocity.y * velocity.y));
        float Ld = MIN_LOOKAHEAD + speed * 3.0f;
        return std::clamp(Ld, MIN_LOOKAHEAD, MAX_LOOKAHEAD);
    }

    // ══════════════════════════════════════════════════════════
    // CROSS-TRACK ERROR
    // ══════════════════════════════════════════════════════════
    float crossTrackError(const Point& pos, const vector<Point>& cl) const {
        if (cl.empty() || currentWaypointIdx < 0) return 0.0f;
        
        int n = (int)cl.size();
        int idx = currentWaypointIdx;
        int nxt = (idx + direction + n) % n;
        int prv = (idx - direction + n) % n;

        float dx = (float)(cl[nxt].x - cl[prv].x);
        float dy = (float)(cl[nxt].y - cl[prv].y);
        float len = sqrtf(dx * dx + dy * dy);
        if (len < 1.0f) return 0.0f;

        float ux = dx / len, uy = dy / len;
        float ex = (float)(pos.x - cl[idx].x);
        float ey = (float)(pos.y - cl[idx].y);

        return ux * ey - uy * ex;
    }

    // ══════════════════════════════════════════════════════════
    // DEBUG INFO
    // ══════════════════════════════════════════════════════════
    struct DebugInfo {
        float curvature = 0;
        float Ld = 0;
        float cte = 0;
        float yaw = 0;
        int   currentIdx = 0;
        int   targetIdx = 0;
        int   lookaheadIdx = 0;
        int   direction = 1;
    };

    // ══════════════════════════════════════════════════════════
    // MAIN STEERING COMPUTATION
    // ══════════════════════════════════════════════════════════
    int computeSteering(const Point& carPos, const Point& predPos,
                        const vector<Point>& cl, DebugInfo& dbg) {
        if (cl.empty()) return STEER_CENTER;

        // Initialize if needed
        if (!initialized) {
            initializeAtPosition(carPos, cl);
        }

        // Determine direction from velocity
        float speed = sqrtf((float)(velocity.x * velocity.x + velocity.y * velocity.y));
        static bool directionLocked = false;
        if (speed > 2.5f && !directionLocked) {
            determineDirection(carPos, cl);
            directionLocked = true;
        }

        // Update waypoint progression
        updateWaypoint(carPos, cl);

        // Get lookahead target
        float Ld = adaptiveLookahead();
        int lookaheadIdx = 0;
        Point target = getLookaheadTarget(predPos, cl, Ld, &lookaheadIdx);

        // Compute steering
        float yaw = estimateYaw();
        Point2f carPosF((float)predPos.x, (float)predPos.y);
        Point2f targetF((float)target.x, (float)target.y);
        float kappa = computeCurvature(carPosF, targetF, yaw);

        float delta = atanf(kappa * PP_WHEELBASE);
        float steerOffset = delta / ((float)M_PI * 0.5f) * (float)STEER_RANGE;
        float steerFloat = (float)STEER_CENTER + steerOffset;
        steerFloat = std::clamp(steerFloat, (float)STEER_RIGHT, (float)STEER_LEFT);

        // Smooth steering
        smoothSteer = STEER_SMOOTH_ALPHA * smoothSteer + (1.0f - STEER_SMOOTH_ALPHA) * steerFloat;

        // Fill debug
        dbg.curvature = kappa;
        dbg.Ld = Ld;
        dbg.cte = crossTrackError(carPos, cl);
        dbg.yaw = yaw;
        dbg.currentIdx = currentWaypointIdx;
        dbg.targetIdx = targetWaypointIdx;
        dbg.lookaheadIdx = lookaheadIdx;
        dbg.direction = direction;

        return (int)std::clamp(smoothSteer, (float)STEER_RIGHT, (float)STEER_LEFT);
    }

    int computeSteering(const Point& carPos, const vector<Point>& cl) {
        DebugInfo dbg;
        Point predPos = (predVel.x != 0 || predVel.y != 0)
            ? Point(carPos.x + predVel.x * 2, carPos.y + predVel.y * 2)
            : carPos;
        return computeSteering(carPos, predPos, cl, dbg);
    }

    enum class Cmd { STRAIGHT, LEFT, RIGHT };
    Cmd classify(int s) const {
        if (s < STEER_CENTER - 15) return Cmd::RIGHT;
        if (s > STEER_CENTER + 15) return Cmd::LEFT;
        return Cmd::STRAIGHT;
    }
};
