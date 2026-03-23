#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/tracking.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iostream>

using namespace cv;
using namespace std;

// ============================================================
// TRACKING MODULE  v8  —  6-state Kalman + Kinematic Lag Comp
//
// v8 changes vs v7:
//
// 1. LAG COMPENSATION: kinematics not matrix power
//    Old: for(s<lagSteps) stFwd = F * stFwd  (3× matrix mul)
//    Problem: 6×6 matrix with acceleration terms compounds
//    noise. After 3 steps, ax/ay errors are ×4.5 amplified.
//    New: explicit kinematic equations (same math, no drift):
//      x_f = x + vx*L + 0.5*ax*L²
//      y_f = y + vy*L + 0.5*ay*L²  (L = lag in frames)
//    Acceleration values are clamped (±2 px/frame²) before use.
//
// 2. MEASURED FRAME INTERVAL (dt_ms)
//    Actual measured time between tracker loop iterations.
//    Published in CarState so the controller can compute
//    lagFrames = LAG_COMP_MS / dt_ms accurately.
//
// 3. CarState additions:
//    float dt_ms   — measured frame interval (default 25ms)
//    (predicted/predVel kept, now computed by kinematics)
// ============================================================

static constexpr int LAG_COMP_MS = 75;  // total pipeline lag estimate

struct CarState {
    Point  position{-1,-1};
    Point  velocity{0,0};
    Point  predicted{-1,-1};   // kinematic prediction at t+LAG_COMP_MS
    Point  predVel{0,0};       // predicted velocity at t+LAG_COMP_MS
    Rect   bbox;
    bool   detected  = false;
    bool   locked    = false;
    double timestamp = 0.0;
    int    windingSign = 0;
    float  dt_ms = 25.0f;      // measured frame interval (ms)
};

class TrackingModule {
public:
    atomic<bool>   running{false};
    atomic<double> lastDetectionTime{0.0};
    atomic<int>    consecHitsPublic{0};

    static constexpr int WARMUP_FRAMES = 30;
    static constexpr int LOCK_FRAMES   = 15;
    static constexpr int LOST_TOL      = 12;
    static constexpr int LOCK_LOST     = 25;

    mutex    stateMutex;
    CarState latestState;
    mutex    frameMutex;
    Mat      frontFrame;
    Rect     trackROI;
    bool     hasROI = false;

    vector<Point> centerlineRef;
    void setCenterline(const vector<Point>& cl) { centerlineRef = cl; }
    void setTrackROI(const Rect& roi) {
        trackROI = roi; hasROI = true;
        cout << "[Tracker] ROI: " << roi << "\n";
    }

    CarState getState() { lock_guard<mutex> lk(stateMutex); return latestState; }
    bool getFrame(Mat& out) {
        lock_guard<mutex> lk(frameMutex);
        if (frontFrame.empty()) return false;
        out = frontFrame.clone(); return true;
    }

    atomic<bool> resetRequested{false};
    void resetLock() { resetRequested = true; }

    void run(VideoCapture& cap) {
        cout << "[Tracker] Starting — MOG2 + Kalman 6-state.\n";

        auto mog2 = createBackgroundSubtractorMOG2(500, 25, false);

        // ── 6-state Kalman [x, y, vx, vy, ax, ay] ────────────
        KalmanFilter KF(6, 2, 0, CV_32F);
        KF.transitionMatrix = (Mat_<float>(6,6) <<
            1,0,1,0,0.5f,0,
            0,1,0,1,0,0.5f,
            0,0,1,0,1,0,
            0,0,0,1,0,1,
            0,0,0,0,1,0,
            0,0,0,0,0,1);

        KF.measurementMatrix = Mat::zeros(2, 6, CV_32F);
        KF.measurementMatrix.at<float>(0,0) = 1.0f;
        KF.measurementMatrix.at<float>(1,1) = 1.0f;

        // Tuned process noise: tight on position, loose on accel
        Mat Q = Mat::zeros(6,6,CV_32F);
        Q.at<float>(0,0)=0.5f; Q.at<float>(1,1)=0.5f;  // pos
        Q.at<float>(2,2)=1.5f; Q.at<float>(3,3)=1.5f;  // vel
        Q.at<float>(4,4)=3.0f; Q.at<float>(5,5)=3.0f;  // accel
        KF.processNoiseCov = Q;
        setIdentity(KF.measurementNoiseCov, Scalar::all(9.0f));
        setIdentity(KF.errorCovPost,        Scalar::all(1.0f));

        Mat meas = Mat::zeros(2,1,CV_32F);
        bool kalmanInit = false;
        int  lostFrames=0, consecHits=0, frameIdx=0;
        bool locked=false;
        int  windingSign=0;
        vector<Point2f> posHistory;
        posHistory.reserve(12);

        Mat kOpen  = getStructuringElement(MORPH_ELLIPSE,Size(3,3));
        Mat kClose = getStructuringElement(MORPH_ELLIPSE,Size(7,7));
        Mat kDilate= getStructuringElement(MORPH_RECT,   Size(5,5));

        cout << "[Tracker] MOG2 warmup...\n";
        { Mat wf,wm;
          for(int i=0;i<WARMUP_FRAMES;i++){
              cap>>wf; if(wf.empty()) continue;
              resize(wf,wf,Size(800,600));
              mog2->apply(wf,wm,0.1);
              {lock_guard<mutex> lk(frameMutex); wf.copyTo(frontFrame);}
              this_thread::sleep_for(chrono::milliseconds(10));
          }
        }
        cout << "[Tracker] Warmup done.\n";

        auto loopT = chrono::steady_clock::now();

        while (running) {
            if (resetRequested.exchange(false)) {
                kalmanInit=false; lostFrames=0; consecHits=0;
                locked=false; windingSign=0; posHistory.clear();
                consecHitsPublic=0;
                cout << "[Tracker] Reset.\n";
            }

            auto t0 = chrono::steady_clock::now();
            // Measure actual dt
            float dt_ms_meas = (float)chrono::duration_cast<chrono::microseconds>(
                t0 - loopT).count() / 1000.0f;
            dt_ms_meas = std::clamp(dt_ms_meas, 15.0f, 50.0f);
            loopT = t0;

            Mat frame;
            cap >> frame;
            if (frame.empty()) {
                this_thread::sleep_for(chrono::milliseconds(5));
                continue;
            }
            resize(frame, frame, Size(800,600));
            frameIdx++;

            Mat fgMask;
            mog2->apply(frame, fgMask, -1);
            threshold(fgMask, fgMask, 127, 255, THRESH_BINARY);
            medianBlur(fgMask, fgMask, 5);
            morphologyEx(fgMask, fgMask, MORPH_OPEN,  kOpen);
            morphologyEx(fgMask, fgMask, MORPH_CLOSE, kClose);
            dilate(fgMask, fgMask, kDilate);

            vector<vector<Point>> contours;
            findContours(fgMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            Mat pred = KF.predict();
            Point predPt = kalmanInit
                ? Point((int)pred.at<float>(0),(int)pred.at<float>(1))
                : Point(400,300);

            Point  bestCtr; Rect bestBox;
            double bestScore=-1.0; bool carFound=false;

            for (auto& cnt : contours) {
                double area = contourArea(cnt);
                if (area<20.0||area>30000.0) continue;
                Rect box = boundingRect(cnt);
                if (box.width<5||box.width>500) continue;
                if (box.height<5||box.height>500) continue;
                float asp = (float)box.width/max(box.height,1);
                if (asp<0.2f||asp>5.0f) continue;
                Point ctr(box.x+box.width/2, box.y+box.height/2);
                if (hasROI && !trackROI.contains(ctr)) continue;
                double dist  = norm(ctr-predPt);
                double score = area/(1.0+(kalmanInit?dist*0.05:0.0));
                if (score>bestScore) {
                    bestScore=score; bestCtr=ctr; bestBox=box; carFound=true;
                }
            }

            if (carFound) {
                if (!kalmanInit) {
                    KF.statePost.at<float>(0)=(float)bestCtr.x;
                    KF.statePost.at<float>(1)=(float)bestCtr.y;
                    for(int i=2;i<6;i++) KF.statePost.at<float>(i)=0.0f;
                    kalmanInit=true;
                    cout<<"[Tracker] First detection ("<<bestCtr.x<<","<<bestCtr.y<<")\n";
                }
                meas.at<float>(0)=(float)bestCtr.x;
                meas.at<float>(1)=(float)bestCtr.y;
                KF.correct(meas);

                lastDetectionTime = chrono::duration<double>(
                    chrono::steady_clock::now().time_since_epoch()).count();
                lostFrames=0; consecHits++;
                consecHitsPublic=consecHits;

                if (!locked && consecHits>=LOCK_FRAMES) {
                    locked=true;
                    cout<<"[Tracker] *** LOCKED ***\n";
                }

                // Winding detection
                if (locked && windingSign==0) {
                    posHistory.push_back(Point2f((float)bestCtr.x,(float)bestCtr.y));
                    if ((int)posHistory.size()>=8 && !centerlineRef.empty()) {
                        auto clIdx=[&](Point2f p)->int{
                            double b=1e9; int id=0;
                            for(int i=0;i<(int)centerlineRef.size();i++){
                                double d=norm(Point2f(centerlineRef[i])-p);
                                if(d<b){b=d;id=i;}}
                            return id;};
                        int io=clIdx(posHistory.front());
                        int in_=clIdx(posHistory.back());
                        int n=(int)centerlineRef.size();
                        int dF=(in_-io+n)%n, dB=(io-in_+n)%n;
                        if(dF!=dB){
                            windingSign=(dF<=dB)?+1:-1;
                            cout<<"[Tracker] Winding: "<<windingSign<<"\n";
                        }
                        posHistory.clear();
                    }
                }
            } else {
                lostFrames++; consecHits=0; consecHitsPublic=0;
                if(locked && lostFrames>LOCK_LOST){locked=false;cout<<"[Tracker] Lock lost.\n";}
            }

            // ── State publish: corrected + kinematic prediction ──
            Point corrPos(-1,-1), corrVel(0,0);
            Point futrPos(-1,-1), futrVel(0,0);

            if (kalmanInit) {
                // Corrected state
                float cx = KF.statePost.at<float>(0);
                float cy = KF.statePost.at<float>(1);
                float vx = KF.statePost.at<float>(2);
                float vy = KF.statePost.at<float>(3);
                // Clamp acceleration before using (noise rejection)
                float ax = std::clamp(KF.statePost.at<float>(4), -2.0f, 2.0f);
                float ay = std::clamp(KF.statePost.at<float>(5), -2.0f, 2.0f);

                corrPos = Point((int)cx, (int)cy);
                corrVel = Point((int)vx, (int)vy);

                // Kinematic forward projection by lagFrames
                float lagFrames = (float)LAG_COMP_MS / max(dt_ms_meas, 1.0f);
                lagFrames = std::clamp(lagFrames, 1.0f, 6.0f);

                float fx = cx + vx*lagFrames + 0.5f*ax*lagFrames*lagFrames;
                float fy = cy + vy*lagFrames + 0.5f*ay*lagFrames*lagFrames;
                float fvx = vx + ax*lagFrames;
                float fvy = vy + ay*lagFrames;

                // Clamp to frame
                fx = std::clamp(fx, 0.0f, 799.0f);
                fy = std::clamp(fy, 0.0f, 599.0f);

                futrPos = Point((int)fx, (int)fy);
                futrVel = Point((int)fvx, (int)fvy);
            }

            {
                lock_guard<mutex> lk(stateMutex);
                latestState.position    = corrPos;
                latestState.velocity    = corrVel;
                latestState.predicted   = futrPos;
                latestState.predVel     = futrVel;
                latestState.bbox        = carFound ? bestBox : Rect();
                latestState.detected    = kalmanInit && (lostFrames<=LOST_TOL);
                latestState.locked      = locked;
                latestState.windingSign = windingSign;
                latestState.dt_ms       = dt_ms_meas;
                latestState.timestamp   = chrono::duration<double>(
                    chrono::steady_clock::now().time_since_epoch()).count();
            }
            { lock_guard<mutex> lk(frameMutex); frame.copyTo(frontFrame); }

            if (frameIdx%30==0) {
                Mat dbg; cvtColor(fgMask,dbg,COLOR_GRAY2BGR);
                if(carFound){rectangle(dbg,bestBox,Scalar(0,255,0),2);
                             circle(dbg,bestCtr,5,Scalar(0,0,255),-1);}
                imwrite("data/processed/fg_mask.png",    dbg);
                imwrite("data/processed/live_frame.png", frame);
            }

            auto elapsed = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now()-t0).count();
            this_thread::sleep_for(chrono::milliseconds(max(0LL,25LL-elapsed)));
        }
        cout << "[Tracker] Stopped.\n";
    }
};
