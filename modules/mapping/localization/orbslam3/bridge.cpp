// SPDX-License-Identifier: GPL-3.0-or-later
// Video transport and read-only map export around the official ORB-SLAM3 library.
#include <System.h>
#include <Atlas.h>
#include <Map.h>
#include <KeyFrame.h>
#include <MapPoint.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <csignal>
#include <execinfo.h>
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <deque>
#include <set>
#include <sstream>
#include <thread>
#include <sys/stat.h>
#include "rolling_statistics.hpp"

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;
using namespace ORB_SLAM3;

static void fatal_signal(int signal) {
    const char message[] = "ORB_FATAL_SIGNAL\n";
    ::write(STDERR_FILENO, message, sizeof(message)-1);
    void* stack[40];
    const int count = backtrace(stack, 40);
    backtrace_symbols_fd(stack, count, STDERR_FILENO);
    _exit(128 + signal);
}

static double seconds(Clock::time_point start) {
    return std::chrono::duration<double>(Clock::now()-start).count();
}
static void replace_file(const std::string& from,const std::string& to) {
    // Windows bind mounts can briefly deny rename while a host reader has the file open.
    for(int i=0;i<40;++i) {
        if(!std::rename(from.c_str(),to.c_str())) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("output_publish_failed");
}
static void atomic_json(const std::string& path, const json& data) {
    { std::ofstream out(path+".tmp"); out << data.dump(); if (!out) throw std::runtime_error("output_write_failed"); }
    replace_file(path+".tmp",path);
}
static json matrix(const Sophus::SE3f& pose) {
    json result=json::array(); const auto m=pose.matrix();
    for(int r=0;r<4;++r) result.push_back({m(r,0),m(r,1),m(r,2),m(r,3)});
    return result;
}

struct FrameInfo { std::string name; double pts; std::string clock; cv::Mat image; };

class MapPublisher {
    std::function<bool(int,bool)> publish;
    std::atomic<bool> running{true};
    std::thread worker;
public:
    std::atomic<int> revision{0};
    std::atomic<bool> failed{false};
    std::string error;
    explicit MapPublisher(std::function<bool(int,bool)> callback):publish(callback) {
        worker=std::thread([this]{
            try {
                while(running) {
                    if(publish(revision+1,false)) ++revision;
                    // Map geometry is slower than tracking; preview/status have their own 10 Hz path.
                    for(int i=0;i<20 && running;++i) std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } catch(const std::exception& e) {error=e.what();failed=true;running=false;}
        });
    }
    void stop() {running=false;if(worker.joinable()) worker.join();}
    void finalize() {stop();if(failed) throw std::runtime_error(error);if(publish(revision+1,true)) ++revision;}
    ~MapPublisher() {stop();}
};

class PreviewPublisher {
    struct Job { cv::Mat image; json progress; };
    const std::string root;
    std::mutex mutex;
    std::condition_variable condition;
    bool running=true;
    std::unique_ptr<Job> pending;
    std::thread worker;
public:
    std::atomic<bool> failed{false};
    std::string error;
    explicit PreviewPublisher(std::string directory):root(std::move(directory)) {
        worker=std::thread([this]{
            std::deque<std::string> retained;
            try {
                while(true) {
                    std::unique_ptr<Job> job;
                    {
                        std::unique_lock<std::mutex> lock(mutex);
                        condition.wait(lock,[this]{return pending || !running;});
                        if(!pending) break;
                        job=std::move(pending);
                    }
                    const auto path=root+"/previews/"+job->progress["frame_id"].get<std::string>();
                    if(!cv::imwrite(path,job->image,{cv::IMWRITE_JPEG_QUALITY,85}))
                        throw std::runtime_error("preview_write_failed");
                    atomic_json(root+"/tracking.json",job->progress);
                    retained.push_back(path);
                    while(retained.size()>8 && !std::remove(retained.front().c_str())) retained.pop_front();
                }
            } catch(const std::exception& e) {error=e.what();failed=true;}
        });
    }
    void submit(const cv::Mat& image, const json& progress) {
        std::lock_guard<std::mutex> lock(mutex);
        // image is the immutable retained frame, not VideoCapture's reused buffer.
        pending.reset(new Job{image,progress});
        condition.notify_one();
    }
    void stop() {
        {std::lock_guard<std::mutex> lock(mutex);running=false;}
        condition.notify_one();
        if(worker.joinable()) worker.join();
    }
    ~PreviewPublisher() {stop();}
};

static bool export_map(System& slam, const std::string& root,
                       const std::map<double,FrameInfo>& frames, std::set<std::string>& stored, int revision, bool final) {
    Map* map=slam.GetAtlasForExport()->GetCurrentMap();
    if(!map || map->KeyFramesInMap()<2 || map->MapPointsInMap()<20) return false;
    struct Point { unsigned long id; Eigen::Vector3f xyz; int observations; std::vector<unsigned char> descriptor; };
    struct Observation { unsigned long id; cv::Point2f xy; float sigma2; };
    struct Key { unsigned long id; FrameInfo source; Sophus::SE3f pose; std::vector<Observation> observations; };
    std::vector<Point> point_copy;
    std::vector<Key> key_copy;
    json keyframes=json::array(), points=json::array();
    std::vector<std::pair<std::string,cv::Mat>> images;
    // Copy only native values under the tracking lock. JSON assembly and all I/O
    // happen after releasing it; a busy tracker takes priority over an export.
    std::unique_lock<std::mutex> lock(map->mMutexMapUpdate,std::defer_lock);
    if(final) lock.lock(); else if(!lock.try_lock()) return false;
    const auto lock_start=Clock::now();
    const auto map_id=map->GetId();
    const auto change=map->GetMapChangeIndex(), big_change=map->GetLastBigChangeIdx();
    auto keys=map->GetAllKeyFrames();
    std::sort(keys.begin(),keys.end(),[](KeyFrame* a,KeyFrame* b){return a->mTimeStamp<b->mTimeStamp;});
    std::set<unsigned long> available;
    for(auto* p:map->GetAllMapPoints()) {
        if(!p || p->isBad() || p->Observations()<2) continue;
        const auto xyz=p->GetWorldPos();
        if(!xyz.allFinite()) continue;
        const cv::Mat descriptor=p->GetDescriptor();
        std::vector<unsigned char> bytes;
        if(!descriptor.empty() && descriptor.type()==CV_8U && descriptor.total()==32)
            bytes.assign(descriptor.ptr<unsigned char>(),descriptor.ptr<unsigned char>()+32);
        point_copy.push_back({p->mnId,xyz,p->Observations(),std::move(bytes)});
        available.insert(p->mnId);
    }
    for(auto* k:keys) {
        if(!k || k->isBad() || k->GetMap()!=map) continue;
        const auto pose = k->GetPose();
        if (!pose.matrix().allFinite()) continue;
        const auto source=frames.find(k->mTimeStamp);
        if(source==frames.end()) continue;
        if(!stored.count(source->second.name)) {
            if(source->second.image.empty()) continue;
            images.emplace_back(source->second.name,source->second.image);
        }
        std::vector<Observation> observations; auto matches=k->GetMapPointMatches();
        for(size_t i=0;i<matches.size() && i<k->mvKeysUn.size();++i) {
            auto* p=matches[i]; if(!p || p->isBad() || !available.count(p->mnId)) continue;
            const auto xy=k->mvKeys[i].pt;
            observations.push_back({p->mnId,xy,k->mvLevelSigma2[k->mvKeysUn[i].octave]});
        }
        key_copy.push_back({k->mnId,source->second,pose,std::move(observations)});
    }
    const double lock_ms=seconds(lock_start)*1000;
    lock.unlock();
    for(const auto& p:point_copy)
        points.push_back({{"id",p.id},{"xyz",{p.xyz.x(),p.xyz.y(),p.xyz.z()}},
            {"observations",p.observations},{"descriptor",p.descriptor}});
    for(const auto& k:key_copy) {
        json observations=json::array();
        for(const auto& o:k.observations)
            observations.push_back({{"point_id",o.id},{"xy",{o.xy.x,o.xy.y}},{"sigma2",o.sigma2}});
        keyframes.push_back({{"id",k.id},{"frame_id",k.source.name},
            {"timestamp_s",k.source.pts},{"timestamp_source",k.source.clock},
            {"T_cw",matrix(k.pose)},{"observations",observations}});
    }
    json value={{"schema_version","orb-slam3/1.0"},{"revision",revision},
        {"map_id",map_id},{"map_change_index",change},{"big_change_index",big_change},
        {"keyframes",keyframes},{"points",points},{"final",final},{"export_lock_ms",lock_ms}};
    if(keyframes.size()<2 || points.size()<20) return false;
    for(const auto& image:images) {
        if(!cv::imwrite(root+"/frames/"+image.first,image.second,{cv::IMWRITE_JPEG_QUALITY,85}))
            throw std::runtime_error("keyframe_write_failed");
        stored.insert(image.first);
    }
    // This revision was computed now, not when its oldest surviving keyframe was captured.
    if(!frames.empty()) value["timestamp_s"]=frames.rbegin()->first;
    atomic_json(root+"/native.json",value);
    return true;
}

int main(int argc,char** argv) {
    for (int signal : {SIGABRT, SIGSEGV, SIGFPE}) std::signal(signal, fatal_signal);
    if(argc!=6) { std::cerr << "orb_video video camera.yaml output replay_rate max_seconds\n"; return 2; }
    const std::string root=argv[3];
    try {
        cv::setNumThreads(1);
        cv::FileStorage calibration(argv[2],cv::FileStorage::READ);
        const int width=int(calibration["Camera.width"]), height=int(calibration["Camera.height"]);
        const double configured_fps=double(calibration["Camera.fps"]);
        const double rate=std::stod(argv[4]), max_seconds=std::stod(argv[5]);
        const bool streaming=std::string(argv[1])=="-";
        cv::VideoCapture capture;
        if(!streaming) capture.open(argv[1]);
        if((!streaming && !capture.isOpened()) || width<=0 || height<=0 || configured_fps<=0) throw std::runtime_error("invalid_video_or_calibration");
        const double fps=capture.get(cv::CAP_PROP_FPS)>0?capture.get(cv::CAP_PROP_FPS):configured_fps;
        System slam("/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt",argv[2],System::MONOCULAR,false);
        atomic_json(root+"/ready.json",{{"ready",true},{"transport",streaming?"jpeg_stdin":"file"}});
        mkdir((root+"/frames").c_str(),0755);
        mkdir((root+"/previews").c_str(),0755);
        const auto start=Clock::now();
        std::ofstream journal(root+"/tracking.jsonl");
        std::map<double,FrameInfo> frames;
        std::mutex frames_mutex;
        std::deque<double> retained;
        std::set<std::string> stored;
        MapPublisher publisher([&](int revision,bool final){
            std::map<double,FrameInfo> copy;
            {std::lock_guard<std::mutex> lock(frames_mutex);copy=frames;}
            return export_map(slam,root,copy,stored,revision,final);
        });
        PreviewPublisher preview_publisher(root);
        std::set<unsigned long> previous;
        RollingStatistics timings,lags;
        json progress;
        int count=0, tracked=0, initializing=0, lost=0, source_fallbacks=0;
        unsigned long previous_map=std::numeric_limits<unsigned long>::max();
        double last_pts=-1, first_pts=0, next_preview=0, initialization=-1;
        cv::Mat input,last_image;
        cv::Mat last_good_image;
        double last_good_pts = -1;
        int previous_state = -1, failure_count = 0;
        std::deque<std::string> failure_images;
        std::ofstream failures(root+"/tracking_failures.jsonl");
        mkdir((root+"/diagnostic_frames").c_str(),0755);
        while(true) {
            double pts;
            std::string timestamp_source=streaming?"browser_media_pts":"source_pts";
            if(streaming) {
                // One ASCII header followed by exactly size JPEG bytes. EOF ends a session.
                std::string header;
                if(!std::getline(std::cin,header)) break;
                std::istringstream fields(header); size_t size=0; std::string extra;
                if(!(fields>>size>>pts) || fields>>extra || size<4 || size>2097152 ||
                    !std::isfinite(pts) || pts<0 || pts<=last_pts) throw std::runtime_error("invalid_stream_header");
                std::vector<unsigned char> jpeg(size);
                if(!std::cin.read(reinterpret_cast<char*>(jpeg.data()),size)) throw std::runtime_error("truncated_stream_frame");
                input=cv::imdecode(jpeg,cv::IMREAD_COLOR);
                if(input.empty() || input.cols!=width || input.rows!=height) throw std::runtime_error("stream_dimensions_changed");
            } else {
                if(!capture.read(input)) break;
                pts=capture.get(cv::CAP_PROP_POS_MSEC)/1000.;
            }
            if(!std::isfinite(pts) || pts<=last_pts) {
                pts=std::max(count/fps,last_pts+1./fps);
                timestamp_source="nominal_fps_fallback"; ++source_fallbacks;
            }
            if(count==0) first_pts=pts;
            if(max_seconds>0 && pts-first_pts>max_seconds) break;
            if(rate>0 && !streaming) std::this_thread::sleep_until(start+std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>((pts-first_pts)/rate)));
            const double frame_gap = last_pts < 0 ? 0 : pts-last_pts;
            last_pts=pts;
            cv::Mat image;
            if(input.cols!=width || input.rows!=height) cv::resize(input,image,cv::Size(width,height)); else image=input;
            std::ostringstream name; name << "orb_" << std::setw(8) << std::setfill('0') << count << ".jpg";
            last_image=image.clone();
            {
                std::lock_guard<std::mutex> lock(frames_mutex);
                frames.emplace(pts,FrameInfo{name.str(),pts,timestamp_source,last_image});
                retained.push_back(pts);
                while(retained.size()>180) {frames.at(retained.front()).image.release();retained.pop_front();}
            }
            if(publisher.failed) throw std::runtime_error(publisher.error);
            if(preview_publisher.failed) throw std::runtime_error(preview_publisher.error);
            const auto before=Clock::now();
            const auto pose=slam.TrackMonocular(image,pts,{},name.str());
            const double ms=seconds(before)*1000;
            // Host transport allows at most one unacknowledged frame in Docker's stdin.
            if(streaming) std::cout << "ORB_ACK " << std::setprecision(17) << pts << std::endl;
            const int state=slam.GetTrackingState();
            if (state == 2 && !pose.matrix().allFinite()) throw std::runtime_error("tracker_nonfinite_pose");
            const bool good=state==2 && pose.matrix().allFinite();
            if(state<=1) ++initializing; else if(!good) ++lost;
            const auto map_id=slam.GetAtlasForExport()->GetCurrentMap()->GetId();
            if(map_id!=previous_map) previous.clear();
            previous_map=map_id;
            json features=json::array(); std::set<unsigned long> current;
            int reused=0;
            const auto keys=slam.GetTrackedKeyPointsUn(); const auto points=slam.GetTrackedMapPoints();
            int candidates = 0;
            for (auto* p: points) if (p && !p->isBad() && p->Observations()>0) ++candidates;
            if(previous_state==2 && !good && failure_count<512) {
                const std::string slot="loss_"+std::to_string(failure_count);
                const std::string before="diagnostic_frames/"+slot+"_before.jpg";
                const std::string after="diagnostic_frames/"+slot+"_after.jpg";
                if(!last_good_image.empty()) cv::imwrite(root+"/"+before,last_good_image,{cv::IMWRITE_JPEG_QUALITY,85});
                cv::imwrite(root+"/"+after,image,{cv::IMWRITE_JPEG_QUALITY,85});
                failure_images.push_back(root+"/"+before); failure_images.push_back(root+"/"+after);
                while(failure_images.size()>16) {
                    std::remove(failure_images.front().c_str()); failure_images.pop_front();
                }
                failures << json({{"timestamp_s",pts},{"previous_good_timestamp_s",last_good_pts},
                    {"map_id",map_id},{"tracking_state",state},{"detected_features",keys.size()},
                    {"map_candidates",candidates},{"frame_gap_s",frame_gap},{"tracking_ms",ms},
                    {"before_image",before},{"after_image",after}}).dump() << '\n';
                failures.flush(); ++failure_count;
            }
            previous_state=state;
            if(good) {
                ++tracked;
                last_good_image=last_image; last_good_pts=pts;
                if(initialization<0) initialization=seconds(start);
                for(size_t i=0;i<keys.size() && i<points.size();++i) {
                    auto* p=points[i]; if(!p || p->isBad() || p->Observations()<1) continue;
                    const bool same=previous.count(p->mnId); reused+=same; current.insert(p->mnId);
                    features.push_back({keys[i].pt.x,keys[i].pt.y,p->mnId,same});
                }
            }
            previous=std::move(current); ++count;
            const double lag=rate>0?std::max(0.,seconds(start)-(pts-first_pts)/rate):0.;
            timings.push(ms); lags.push(lag);
            json record={{"frame_id",name.str()},{"timestamp_s",pts},{"tracking_state",state},
                {"tracked_features",features.size()},{"same_previous",reused},{"tracking_ms",ms},{"input_lag_seconds",lag},
                {"detected_features",keys.size()},{"map_candidates",candidates},{"map_id",map_id},{"frame_gap_s",frame_gap}};
            journal << record.dump() << '\n';
            progress={{"backend","orb_slam3_monocular"},{"status","receiving"},{"frames_received",count},
                {"tracked_frames",tracked},{"initializing_frames",initializing},{"lost_tracking_frames",lost},{"frame_drops",0},
                {"source_pts_fallbacks",source_fallbacks},{"source_fps",fps},{"replay_rate",rate},
                {"input",streaming?"browser_camera":"local_file"},
                {"last_map_timestamp_s",pts},{"frame_id",name.str()},{"tracking_state",state},{"map_id",map_id},
                {"frame_tracking",{{"tracked_features",features.size()},{"reused_features",reused},
                    {"detected_features",keys.size()},{"map_candidates",candidates}}},
                {"tracking_ms",ms},{"tracking_p95_ms",timings.percentile(.95)},
                {"statistics_window_frames",timings.size()},
                {"input_lag_seconds",lag},{"input_lag_p95_seconds",lags.percentile(.95)},
                {"initialization_seconds",initialization},{"elapsed_seconds",seconds(start)},
                {"tracking_ratio",double(tracked)/count},{"width",width},{"height",height},
                {"features",features},{"revision",publisher.revision.load()},{"snapshots",publisher.revision.load()}};
            if(good) progress["T_cw"]=matrix(pose);
            if(seconds(start)>=next_preview) {
                preview_publisher.submit(last_image,progress);
                journal.flush();
                next_preview=seconds(start)+.1;
            }
        }
        preview_publisher.stop();
        if(preview_publisher.failed) throw std::runtime_error(preview_publisher.error);
        publisher.stop();
        slam.Shutdown();
        publisher.finalize();
        if(!last_image.empty()) cv::imwrite(root+"/previews/"+progress["frame_id"].get<std::string>(),last_image);
        progress["revision"]=publisher.revision.load(); progress["snapshots"]=publisher.revision.load();
        progress["status"]=publisher.revision?"complete":"failed";
        progress["elapsed_seconds"]=seconds(start);
        progress["performance_target_met"]=count>30 && rate==1 && double(tracked)/count>=.9 &&
            timings.percentile(.95)<=1000./fps && lags.percentile(.95)<=2;
        progress["quality_scope"]="sparse_monocular_tracking; metric_accuracy_and_dense_surface_not_certified";
        atomic_json(root+"/tracking.json",progress);
        auto summary=progress; summary.erase("features"); summary.erase("T_cw");
        std::cout << "ORB_RESULT " << summary.dump() << std::endl;
        return publisher.revision?0:3;
    } catch(const std::exception& e) {
        atomic_json(root+"/tracking.json",{{"status","failed"},{"error",e.what()}});
        std::cerr << e.what() << std::endl; return 2;
    }
}
