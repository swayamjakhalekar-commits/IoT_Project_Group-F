// ============================================================
// AUTONOMOUS RACING  v8  —  Waypoint Progression
//
// CONCEPT:
//   1. Car detected → find closest centerline point = START
//   2. Point behind car = where we came from
//   3. Point ahead = TARGET (where we're going)
//   4. As car approaches target → advance to NEXT point
//
// NO calibration — direction auto-detected from velocity
// ============================================================

#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <filesystem>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>

#include "ble_manager.hpp"
#include "centerline_module.hpp"
#include "tracking_module.hpp"
#include "pure_pursuit_controller.hpp"
#include "safety_monitor.hpp"

using namespace cv;
using namespace std;
using namespace chrono;

static constexpr int LOST_TOL_MAIN     = 40;
static constexpr int CAR_LOST_GRACE_MS = 2000;
static constexpr int SEARCH_FWD_MS     = 300;
static constexpr int SEARCH_STOP_MS    = 500;
static constexpr int SEARCH_CYCLE_MS   = SEARCH_FWD_MS + SEARCH_STOP_MS;
static constexpr float EDGE_CTE_THRESH = 55.0f;
static constexpr int EDGE_FRAMES       = 8;
static constexpr int RECOVERY_BACK_MS  = 350;
static constexpr int RECOVERY_STOP_MS  = 150;
static constexpr int MIN_THROTTLE      = 4;

enum class AppState { SEARCHING, LOCKING, RUNNING, CAR_LOST, SAFETY_HOLD };
string stateName(AppState s) {
    switch(s) {
        case AppState::SEARCHING:   return "SEARCH";
        case AppState::LOCKING:     return "LOCK";
        case AppState::RUNNING:     return "RUN";
        case AppState::CAR_LOST:    return "LOST";
        case AppState::SAFETY_HOLD: return "SAFE";
        default: return "?";
    }
}

atomic<bool> systemRunning{false};
static struct termios g_origTerm;
static bool g_termSaved = false;
void saveTerminal()    { if(tcgetattr(STDIN_FILENO,&g_origTerm)==0) g_termSaved=true; }
void restoreTerminal() { if(g_termSaved) tcsetattr(STDIN_FILENO,TCSANOW,&g_origTerm); }

enum class Zone { MAX_ACCEL, ACCEL_OUT, SOFT_TURN, BRAKE, SHARP_TURN };
Zone classifyZone(float k) {
    if (k < 0.003f) return Zone::MAX_ACCEL;
    if (k < 0.006f) return Zone::ACCEL_OUT;
    if (k < 0.012f) return Zone::SOFT_TURN;
    if (k < 0.020f) return Zone::BRAKE;
    return Zone::SHARP_TURN;
}
Scalar zoneColor(Zone z) {
    switch(z){
        case Zone::MAX_ACCEL:  return Scalar(255,80,0);
        case Zone::ACCEL_OUT:  return Scalar(0,200,0);
        case Zone::SOFT_TURN:  return Scalar(0,200,255);
        case Zone::BRAKE:      return Scalar(0,100,255);
        case Zone::SHARP_TURN: return Scalar(0,0,220);
        default: return Scalar(200,200,200);
    }
}
int zoneThrottle(Zone z) {
    switch(z){
        case Zone::MAX_ACCEL: return 8;
        case Zone::ACCEL_OUT: return 7;
        case Zone::SOFT_TURN: return 6;
        case Zone::BRAKE:     return 5;
        case Zone::SHARP_TURN:return 4;
        default: return 6;
    }
}

void hudBox(Mat& f, Rect r, double a=0.22) {
    r.x=max(0,r.x); r.y=max(0,r.y);
    r.width=min(r.width,f.cols-r.x); r.height=min(r.height,f.rows-r.y);
    if(r.width<=0||r.height<=0) return;
    Mat roi=f(r), blk=Mat::zeros(roi.size(),roi.type());
    addWeighted(roi,a,blk,1.0-a,0,roi);
}

void drawCar(Mat& f, const CarState& st) {
    if (!st.detected) return;
    if (st.bbox.width>0) rectangle(f,st.bbox,st.locked?Scalar(0,255,0):Scalar(0,180,80),2);
    if (st.position.x>=0) {
        circle(f,st.position,8,Scalar(0,255,80),-1);
        if (st.velocity.x!=0||st.velocity.y!=0) {
            Point tip(st.position.x+st.velocity.x*4, st.position.y+st.velocity.y*4);
            arrowedLine(f,st.position,tip,Scalar(0,200,255),2,LINE_AA,0,0.4);
        }
    }
}

void drawZoneCenterline(Mat& f, const vector<CenterlinePoint>& pts) {
    int n=(int)pts.size();
    for(int i=1;i<n;i++) line(f,pts[i-1].pt,pts[i].pt,zoneColor(classifyZone(pts[i].curvature)),3,LINE_AA);
    if(n>2) line(f,pts.back().pt,pts.front().pt,zoneColor(classifyZone(pts.back().curvature)),3,LINE_AA);
}

void sendBLE(BLEManager& ble, int thr, int steer, int torque=32) {
    thr = max(MIN_THROTTLE, thr);
    auto f=ble.BASE_FRAME;
    f[BLEManager::THROTTLE_BYTE]=thr;
    f[BLEManager::STEER_BYTE]=steer;
    f[BLEManager::TORQUE_BYTE]=torque;
    ble.sendFrame(f);
}

inline void feedController(PurePursuitController& pp, const CarState& st) {
    pp.velocity = st.velocity;
    pp.predVel  = st.predVel;
}

void runLoop(TrackingModule& tracker, CenterlineModule& cl, PurePursuitController& pp,
             BLEManager& ble, SafetyMonitor& safety)
{
    namedWindow("AUTO", WINDOW_NORMAL);
    resizeWindow("AUTO", 960, 720);

    AppState state = AppState::SEARCHING;
    auto phaseStart = steady_clock::now();
    int noCarFrames = 0;
    auto carLostTime = steady_clock::now();

    enum class RunMode { NORMAL, EDGE_BACK, EDGE_STOP };
    RunMode runMode = RunMode::NORMAL;
    int edgeCount = 0;
    auto recoveryStart = steady_clock::now();
    float prevSteer = 120.0f;

    while (systemRunning) {
        int key = waitKey(1);
        if (key=='q'||key=='Q') { ble.stopImmediate(); systemRunning=false; break; }

        if (safety.safetyTripped && state != AppState::SAFETY_HOLD) state = AppState::SAFETY_HOLD;

        Mat disp;
        if (!tracker.getFrame(disp)) { this_thread::sleep_for(milliseconds(3)); continue; }
        drawZoneCenterline(disp, cl.centerlineFull);
        CarState st = tracker.getState();

        switch (state) {
        case AppState::SEARCHING: {
            int elapsed = (int)duration_cast<milliseconds>(steady_clock::now()-phaseStart).count();
            int cyc = elapsed % SEARCH_CYCLE_MS;
            sendBLE(ble, cyc < SEARCH_FWD_MS ? 5 : 0, 120, cyc < SEARCH_FWD_MS ? 32 : 0);
            if (st.detected) { ble.stopImmediate(); phaseStart=steady_clock::now(); state=AppState::LOCKING; }
            drawCar(disp,st);
            hudBox(disp,Rect(5,5,300,40));
            putText(disp,"v8 SEARCHING...",Point(10,32),FONT_HERSHEY_SIMPLEX,0.7,Scalar(0,200,255),2);
            break;
        }
        case AppState::LOCKING: {
            sendBLE(ble,0,120,0);
            if (!st.detected) { phaseStart=steady_clock::now(); state=AppState::SEARCHING; break; }
            if (st.locked) {
                pp.reset(); feedController(pp,st);
                noCarFrames=0; runMode=RunMode::NORMAL; edgeCount=0; prevSteer=120.0f;
                state=AppState::RUNNING;
            }
            drawCar(disp,st);
            hudBox(disp,Rect(5,5,300,40));
            putText(disp,"LOCKING...",Point(10,32),FONT_HERSHEY_SIMPLEX,0.7,Scalar(0,255,200),2);
            break;
        }
        case AppState::RUNNING: {
            if (st.detected) {
                noCarFrames=0;
                float nearK=0; { double b=1e9; for(auto& cp:cl.centerlineFull){double d=norm(cp.pt-st.position);if(d<b){b=d;nearK=cp.curvature;}}}
                Zone z=classifyZone(nearK);
                int thr=max(MIN_THROTTLE,zoneThrottle(z));

                switch(runMode) {
                case RunMode::NORMAL: {
                    feedController(pp,st);
                    Point pred = st.predicted.x>=0 ? st.predicted : st.position;
                    PurePursuitController::DebugInfo dbg;
                    int ppS = pp.computeSteering(st.position, pred, cl.centerline, dbg);
                    
                    // Smooth
                    float sm = 0.7f * prevSteer + 0.3f * (float)ppS;
                    prevSteer = sm;
                    int steer = (int)std::clamp(sm, 40.0f, 200.0f);

                    // Edge?
                    if (fabsf(dbg.cte) > EDGE_CTE_THRESH) {
                        if (++edgeCount >= EDGE_FRAMES) {
                            ble.stopImmediate(); recoveryStart=steady_clock::now();
                            runMode=RunMode::EDGE_BACK; edgeCount=0; break;
                        }
                    } else edgeCount=0;

                    sendBLE(ble, thr, steer);

                    // Draw
                    drawCar(disp,st);
                    if(dbg.currentIdx>=0 && dbg.currentIdx<(int)cl.centerline.size())
                        circle(disp,cl.centerline[dbg.currentIdx],6,Scalar(0,0,255),-1);
                    if(dbg.targetIdx>=0 && dbg.targetIdx<(int)cl.centerline.size())
                        circle(disp,cl.centerline[dbg.targetIdx],8,Scalar(0,255,0),-1);
                    if(dbg.lookaheadIdx>=0 && dbg.lookaheadIdx<(int)cl.centerline.size()) {
                        circle(disp,cl.centerline[dbg.lookaheadIdx],10,Scalar(0,220,255),2);
                        arrowedLine(disp,st.position,cl.centerline[dbg.lookaheadIdx],Scalar(0,200,255),2,LINE_AA,0,0.3);
                    }

                    hudBox(disp,Rect(5,5,400,130));
                    string dir = dbg.direction>0?"CW":"CCW";
                    putText(disp,"v8 RUN "+dir,Point(10,28),FONT_HERSHEY_SIMPLEX,0.6,Scalar(0,255,200),2);
                    putText(disp,"WP:"+to_string(dbg.currentIdx)+"->"+to_string(dbg.targetIdx)+"->"+to_string(dbg.lookaheadIdx),
                            Point(10,52),FONT_HERSHEY_SIMPLEX,0.5,Scalar(0,255,180),1);
                    putText(disp,"k="+to_string(dbg.curvature).substr(0,6)+" Ld="+to_string((int)dbg.Ld)+" s="+to_string(steer),
                            Point(10,74),FONT_HERSHEY_SIMPLEX,0.45,Scalar(200,200,100),1);
                    putText(disp,"CTE="+to_string((int)dbg.cte)+" thr="+to_string(thr),
                            Point(10,94),FONT_HERSHEY_SIMPLEX,0.45,Scalar(150,150,150),1);
                    putText(disp,"'q'=stop",Point(10,114),FONT_HERSHEY_SIMPLEX,0.4,Scalar(100,100,100),1);
                    break;
                }
                case RunMode::EDGE_BACK: {
                    int el=(int)duration_cast<milliseconds>(steady_clock::now()-recoveryStart).count();
                    if(el<RECOVERY_BACK_MS){
                        auto f=ble.BASE_FRAME; f[7]=215; f[11]=120; f[12]=32; ble.sendFrame(f);
                    } else { ble.stopImmediate(); recoveryStart=steady_clock::now(); runMode=RunMode::EDGE_STOP; }
                    drawCar(disp,st);
                    hudBox(disp,Rect(5,5,250,40));
                    putText(disp,"EDGE BACK",Point(10,32),FONT_HERSHEY_SIMPLEX,0.7,Scalar(0,100,255),2);
                    break;
                }
                case RunMode::EDGE_STOP: {
                    int el=(int)duration_cast<milliseconds>(steady_clock::now()-recoveryStart).count();
                    if(el>=RECOVERY_STOP_MS){ pp.reset(); feedController(pp,st); prevSteer=120; runMode=RunMode::NORMAL; }
                    drawCar(disp,st);
                    break;
                }
                default: break;
                }
            } else {
                noCarFrames++;
                if(noCarFrames>LOST_TOL_MAIN){ ble.stopImmediate(); carLostTime=steady_clock::now(); state=AppState::CAR_LOST; }
            }
            break;
        }
        case AppState::CAR_LOST: {
            int el=(int)duration_cast<milliseconds>(steady_clock::now()-carLostTime).count();
            if(st.detected){ noCarFrames=0; state=AppState::RUNNING; break; }
            if(el>CAR_LOST_GRACE_MS){ pp.reset(); phaseStart=steady_clock::now(); state=AppState::SEARCHING; }
            hudBox(disp,Rect(5,5,200,40));
            putText(disp,"LOST",Point(10,32),FONT_HERSHEY_SIMPLEX,0.7,Scalar(0,0,255),2);
            break;
        }
        case AppState::SAFETY_HOLD: {
            hudBox(disp,Rect(5,5,200,40));
            putText(disp,"SAFETY",Point(10,32),FONT_HERSHEY_SIMPLEX,0.7,Scalar(0,0,255),2);
            break;
        }
        }

        imshow("AUTO", disp);
        this_thread::sleep_for(milliseconds(3));
    }
}

int main() {
    filesystem::create_directories("data/raw");
    filesystem::create_directories("data/processed");
    saveTerminal();

    BLEManager ble;
    VideoCapture cap;
    CenterlineModule cl;
    TrackingModule tracker;
    PurePursuitController pp;
    SafetyMonitor safety;

    cout << "\n=== AUTONOMOUS RACING v8 — Waypoint Progression ===\n\n";

    cout << "[1] BLE... ";
    if (!ble.connect(5)) { cerr << "FAIL\n"; restoreTerminal(); return -1; }
    cout << ble.getActiveCarInfo() << " OK\n";

    cout << "[2] Camera... ";
    cap.open(0, cv::CAP_V4L2);
    if (!cap.isOpened()) { cerr << "FAIL\n"; restoreTerminal(); return -1; }
    cap.set(CAP_PROP_FRAME_WIDTH,800); cap.set(CAP_PROP_FRAME_HEIGHT,600); cap.set(CAP_PROP_FPS,30);
    cout << "OK\n";

    cout << "[3] Centerline... ";
    if (!cl.initialize(cap)) { cerr << "FAIL\n"; restoreTerminal(); return -1; }
    cout << cl.centerline.size() << " pts OK\n";

    cout << "[4] Threads... ";
    systemRunning = true;
    tracker.running = true;
    safety.running = true;
    tracker.setTrackROI(cl.getTrackROI());
    tracker.setCenterline(cl.centerline);
    thread t1(&TrackingModule::run, &tracker, ref(cap));
    thread t2(&SafetyMonitor::run, &safety, ref(ble), ref(tracker), cl.ready);
    ble.startSenderThread();
    { Mat tmp; int w=0; while(!tracker.getFrame(tmp)&&w<80){this_thread::sleep_for(milliseconds(50));w++;} }
    cout << "OK\n\n";

    cout << "[READY] Place car, 'q' to stop\n\n";
    runLoop(tracker, cl, pp, ble, safety);

    ble.stopImmediate();
    this_thread::sleep_for(milliseconds(200));
    systemRunning = false;
    tracker.running = false;
    safety.running = false;
    ble.stopSenderThread();
    t1.join(); t2.join();
    cap.release();
    destroyAllWindows();
    restoreTerminal();
    cout << "\nDone.\n";
    return 0;
}
