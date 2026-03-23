#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <filesystem>
#include <cmath>

using namespace cv;
using namespace std;

// ============================================================
// CENTERLINE MODULE  v8  —  Dense path (original density)
//
// Reverted from 9-waypoint sparse path back to full density.
// Sampling: every 20th inner contour point → ~25-35 raw midpoints
// Three-pass Gaussian smooth → clean continuous curve
//
// The snake-motion fix is now handled in the CONTROLLER
// (Stanley + directional hold + IIR filter), not in the path.
// Dense waypoints give accurate curvature and tangent data.
// ============================================================

struct CenterlinePoint {
    Point pt;
    float curvature = 0.0f;
    float throttle  = 0.0f;
};

struct Seg {
    Point  p1, p2;
    Scalar color;
};

class CenterlineModule {
public:
    vector<Point>           centerline;
    vector<CenterlinePoint> centerlineFull;
    vector<Seg>             segments;
    bool ready = false;

    static constexpr float THROTTLE_DRIVE = 10.0f;
    static constexpr float THROTTLE_MIN   =  6.0f;
    static constexpr float CURV_SCALE     = 10.0f;
    static constexpr float RAMP_START     =  5.0f;
    static constexpr int   RAMP_STEPS     = 25;

    int  rampCounter  = 0;
    bool rampComplete = false;

    static float menger(const Point& A, const Point& B, const Point& C) {
        float a=(float)norm(A-B), b=(float)norm(B-C), c=(float)norm(C-A);
        if(a<1e-5f||b<1e-5f||c<1e-5f) return 0.0f;
        float area=fabsf((float)((B.x-A.x)*(C.y-A.y)-(C.x-A.x)*(B.y-A.y)))*0.5f;
        return (2.0f*area)/(a*b*c);
    }

    static float curvToThrottle(float k) {
        return max(THROTTLE_MIN, min(THROTTLE_DRIVE,
               THROTTLE_DRIVE / (1.0f + k * CURV_SCALE)));
    }

    // Gaussian smooth — wraps around (closed loop track)
    static vector<Point> gaussSmooth(const vector<Point>& pts, int window, float sigma) {
        int n=(int)pts.size();
        if(n<window) return pts;
        int half=window/2;
        vector<float> w(window);
        float sum=0.0f;
        for(int i=0;i<window;i++){
            float x=(float)(i-half);
            w[i]=expf(-x*x/(2.f*sigma*sigma));
            sum+=w[i];
        }
        for(auto& v:w) v/=sum;
        vector<Point> out(n);
        for(int i=0;i<n;i++){
            float fx=0,fy=0;
            for(int j=0;j<window;j++){
                int idx=(i-half+j+n)%n;
                fx+=w[j]*(float)pts[idx].x;
                fy+=w[j]*(float)pts[idx].y;
            }
            out[i]=Point((int)roundf(fx),(int)roundf(fy));
        }
        return out;
    }

    Rect getTrackROI() const {
        if(centerline.empty()) return Rect(0,0,800,600);
        int x0=800,y0=600,x1=0,y1=0;
        for(const Point& p:centerline){
            x0=min(x0,p.x); y0=min(y0,p.y);
            x1=max(x1,p.x); y1=max(y1,p.y);
        }
        int mg=90;
        return Rect(max(0,x0-mg),max(0,y0-mg),
                    min(800,x1+mg)-max(0,x0-mg),
                    min(600,y1+mg)-max(0,y0-mg));
    }

    void drawOnFrame(Mat& frame) const {
        for(const auto& s:segments)
            line(frame,s.p1,s.p2,s.color,2,LINE_AA);
        for(const auto& cp:centerlineFull)
            circle(frame,cp.pt,2,Scalar(0,255,255),-1);
    }

    bool extractBorders(const Mat& frame) {
        CV_Assert(frame.cols==800 && frame.rows==600);

        Mat hsv;
        cvtColor(frame,hsv,COLOR_BGR2HSV);

        Mat r1,r2,wh;
        inRange(hsv,Scalar(0,  55,40), Scalar(12, 255,255),r1);
        inRange(hsv,Scalar(168,55,40), Scalar(180,255,255),r2);
        inRange(hsv,Scalar(0,0,185),   Scalar(180,55,255), wh);
        Mat barMask=r1|r2|wh;

        Mat dk=getStructuringElement(MORPH_RECT,Size(11,11));
        dilate(barMask,barMask,dk);
        morphologyEx(barMask,barMask,MORPH_CLOSE,
                     getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

        vector<vector<Point>> contours;
        findContours(barMask,contours,RETR_TREE,CHAIN_APPROX_NONE);

        if((int)contours.size()<3){
            cerr<<"[Centerline] Only "<<contours.size()<<" contours (need >=3).\n";
            return false;
        }

        vector<pair<double,int>> arcs;
        for(int i=0;i<(int)contours.size();i++)
            arcs.push_back({arcLength(contours[i],false),i});
        sort(arcs.rbegin(),arcs.rend());

        cout<<"[Centerline] Arc lengths: "
            <<(int)arcs[0].first<<"  "
            <<(int)arcs[1].first<<"  "
            <<(int)arcs[2].first<<"\n";

        vector<Point>& inner=contours[arcs[1].second];
        vector<Point>& outer=contours[arcs[2].second];

        // Sample every 20th point on inner border → ~25-35 raw midpoints
        vector<Point> raw;
        for(size_t i=0;i<inner.size();i+=20){
            Point pi=inner[i];
            double bd=1e9; Point bo=outer[0];
            for(const Point& po:outer){
                double d=norm(pi-po);
                if(d<bd){bd=d;bo=po;}
            }
            raw.push_back(Point((pi.x+bo.x)/2,(pi.y+bo.y)/2));
        }

        if((int)raw.size()<8){
            cerr<<"[Centerline] Too few raw midpoints ("<<raw.size()<<").\n";
            return false;
        }

        // Three-pass Gaussian smooth → clean dense centerline
        vector<Point> s1=gaussSmooth(raw,9,3.0f);
        vector<Point> s2=gaussSmooth(s1, 7,2.5f);
        vector<Point> s3=gaussSmooth(s2, 5,2.0f);

        // ── ODD-INDEX SELECTION ───────────────────────────────
        // Use only points at odd indices (1, 3, 5, 7 ...) from
        // the smooth curve. This doubles the spacing between
        // active waypoints (~40-50px apart instead of ~20-25px)
        // which forces the controller to hold each steering
        // direction longer — eliminating snake/zigzag motion.
        // Even-indexed points are kept only in centerlineFull
        // for display and curvature reference, NOT for steering.
        // The controller (closestIdx, lookahead walk) operates
        // entirely on 'centerline' which contains only odd pts.
        // Every 3rd point: indices 0, 3, 6, 9 ...
        // Spacing ~60-75px — car holds direction ~20 frames before next target
        vector<Point> oddPts;
        for(int i=0;i<(int)s3.size();i+=3)
            oddPts.push_back(s3[i]);

        if((int)oddPts.size()<4){
            oddPts = s3;
        }

        centerline.clear();
        centerlineFull.clear();
        for(const Point& p:oddPts){
            centerline.push_back(p);
            centerlineFull.push_back({p,0.0f,THROTTLE_DRIVE});
        }

        cout<<"[Centerline] s3="<<s3.size()<<" pts → odd-select="
            <<centerline.size()<<" active waypoints\n";

        // Curvature pass on the selected waypoints
        int n=(int)centerlineFull.size();
        for(int i=0;i<n;i++){
            int pr=(i-1+n)%n, nx=(i+1)%n;
            float k=menger(centerlineFull[pr].pt,
                           centerlineFull[i].pt,
                           centerlineFull[nx].pt);
            centerlineFull[i].curvature=k;
            centerlineFull[i].throttle =curvToThrottle(k);
        }

        segments.clear();
        for(int i=1;i<n;i++){
            float t=min(1.0f,centerlineFull[i].curvature*120.0f);
            Scalar col((int)(255*(1-t)),0,(int)(255*t));
            segments.push_back({centerlineFull[i-1].pt,centerlineFull[i].pt,col});
        }
        if(n>2)
            segments.push_back({centerlineFull.back().pt,centerlineFull.front().pt,
                                 segments.empty()?Scalar(255,0,0):segments.back().color});

        ready=true;
        cout<<"[Centerline] "<<n<<" waypoints ready.\n";
        return true;
    }

    bool initialize(VideoCapture& cap) {
        cout<<"[Centerline] Warm-up (30 frames)...\n";
        Mat frame;
        for(int i=0;i<30;i++) cap>>frame;
        if(frame.empty()){cerr<<"[Centerline] Empty frame.\n";return false;}
        resize(frame,frame,Size(800,600));

        filesystem::create_directories("data/raw");
        filesystem::create_directories("data/processed");
        imwrite("data/raw/calibration.png",frame);

        if(!extractBorders(frame)){
            cerr<<"[Centerline] extractBorders failed.\n";return false;
        }

        float kMin=1e9f,kMax=0.0f;
        for(auto& cp:centerlineFull){
            kMax=max(kMax,cp.curvature);
            kMin=min(kMin,cp.curvature);
        }
        printf("[Centerline] %d pts  curv %.4f->%.4f  thr %.1f->%.1f\n",
               (int)centerline.size(),kMin,kMax,
               curvToThrottle(kMax),curvToThrottle(kMin));

        Mat preview=frame.clone();
        {
            Mat hsv2;cvtColor(frame,hsv2,COLOR_BGR2HSV);
            Mat r1,r2,wh;
            inRange(hsv2,Scalar(0,55,40),  Scalar(12,255,255), r1);
            inRange(hsv2,Scalar(168,55,40),Scalar(180,255,255),r2);
            inRange(hsv2,Scalar(0,0,185),  Scalar(180,55,255), wh);
            Mat msk=r1|r2|wh;
            Mat dk=getStructuringElement(MORPH_RECT,Size(11,11));
            dilate(msk,msk,dk);
            vector<vector<Point>> ctrs;
            findContours(msk,ctrs,RETR_TREE,CHAIN_APPROX_NONE);
            if((int)ctrs.size()>=3){
                vector<pair<double,int>> av;
                for(int i=0;i<(int)ctrs.size();i++)
                    av.push_back({arcLength(ctrs[i],false),i});
                sort(av.rbegin(),av.rend());
                drawContours(preview,ctrs,av[1].second,Scalar(0,255,0),1);
                drawContours(preview,ctrs,av[2].second,Scalar(255,80,0),1);
            }
        }

        drawOnFrame(preview);
        rectangle(preview,getTrackROI(),Scalar(0,255,255),1);

        putText(preview,
                "GREEN=inner  BLUE=outer  YELLOW=ROI  |  Press 'S' to confirm",
                Point(8,preview.rows-8),FONT_HERSHEY_SIMPLEX,0.46,Scalar(255,255,255),1,LINE_AA);

        imwrite("data/processed/centerline_preview.png",preview);
        namedWindow("Centerline - Press S",WINDOW_NORMAL);
        resizeWindow("Centerline - Press S",960,720);
        imshow("Centerline - Press S",preview);
        cout<<"[Centerline] Check path then press 'S'.\n";
        while(true){int k=waitKey(30);if(k=='s'||k=='S') break;}
        destroyWindow("Centerline - Press S");
        return true;
    }

    float getThrottleAtPos(const Point& pos) {
        if(centerlineFull.empty()) return RAMP_START;
        double best=1e9; float tgt=THROTTLE_DRIVE;
        for(auto& cp:centerlineFull){
            double d=norm(cp.pt-pos);
            if(d<best){best=d;tgt=cp.throttle;}
        }
        if(!rampComplete){
            float t=(float)rampCounter/(float)RAMP_STEPS;
            float r=RAMP_START+t*(tgt-RAMP_START);
            if(++rampCounter>=RAMP_STEPS){rampComplete=true;cout<<"[Centerline] Ramp done.\n";}
            return max(RAMP_START,min(tgt,r));
        }
        return tgt;
    }

    void resetRamp(){rampCounter=0;rampComplete=false;}

    Point nearest(const Point& pos) const {
        if(centerline.empty()) return pos;
        double best=1e9; Point bp=centerline[0];
        for(const Point& p:centerline){
            double d=norm(p-pos);
            if(d<best){best=d;bp=p;}
        }
        return bp;
    }
};
