// Drives the real ManusTelemetry with fake SDK callbacks and a fake 120 Hz
// publish timer, injecting one fault of each kind at a known time. Prints JSON
// lines: {"telemetry": <batch>, "recv": ns} and {"glove": ..., "recv": ns}.
// Built and checked by tests/test_publisher_telemetry.py (needs MANUS SDK headers).
#include "ManusTelemetry.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct Vec { double x, y, z; };
struct Quat { double x, y, z, w; };
struct Pose { Vec position; Quat orientation; };
struct Node { Pose pose; };
struct FakeGlove { std::vector<Node> raw_nodes; };

static std::mutex g_Out;
static std::vector<std::string> g_Lines;

static int64_t Now() { return ManusTelemetry::UnixNs(); }
static double Elapsed(int64_t p_Start) { return (Now() - p_Start) / 1e9; }

static FakeGlove MakePose(uint32_t p_Glove, uint64_t p_Frame, bool p_Twitch)
{
    FakeGlove t_Msg;
    for (int i = 0; i < 25; i++)
    {
        const double t_Phase = 0.002 * p_Frame + 0.1 * i + p_Glove;
        Node t_Node{{{0.01 * i + 0.001 * std::sin(t_Phase), 0.005 * std::cos(t_Phase), 0.0}, {0, 0, 0, 1}}};
        if (p_Twitch && i == 9)
            t_Node.pose.position.y += 0.08; // index fingertip jumps for one SDK frame
        t_Msg.raw_nodes.push_back(t_Node);
    }
    return t_Msg;
}

static void Emit(const std::string &p_Line)
{
    std::lock_guard<std::mutex> t_Lock(g_Out);
    g_Lines.push_back(p_Line);
}

int main()
{
    ManusTelemetry &t_Tel = ManusTelemetry::Instance();
    t_Tel.Start([](const std::string &p_Batch) {
        Emit("{\"recv\":" + std::to_string(Now() + 200000) + ",\"telemetry\":" + p_Batch + "}");
    }, nullptr, "harness");

    const int64_t t_Start = Now();
    std::atomic<bool> t_Done{false};
    std::mutex t_CacheMutex;
    std::vector<FakeGlove> t_Cache(3);
    std::vector<uint64_t> t_CachePt(3, 0);
    std::vector<bool> t_HaveData(3, false);

    // Fault windows, seconds from start.
    auto In = [&](double a, double b) { double t = Elapsed(t_Start); return t >= a && t < b; };

    std::thread t_Sdk([&] {
        uint64_t t_Frame = 0;
        while (!t_Done)
        {
            const bool t_SdkStall = In(1.2, 1.5);
            for (uint32_t t_Glove = 1; t_Glove <= 2; t_Glove++)
            {
                if (t_SdkStall || (t_Glove == 2 && In(2.0, 2.3)))
                    continue;
                ManusTimestamp t_Pt{static_cast<uint64_t>(t_Start / 1000000) + t_Frame};
                {
                    std::lock_guard<std::mutex> t_Lock(t_CacheMutex);
                    t_Cache[t_Glove] = MakePose(t_Glove, t_Frame, t_Glove == 1 && In(0.9, 0.92));
                    t_CachePt[t_Glove] = t_Pt.time;
                    t_HaveData[t_Glove] = true;
                }
                t_Tel.NoteSkeleton(t_Glove, 25, t_Pt);
                t_Tel.NoteErgonomics(t_Glove, t_Pt);
            }
            if (t_Frame % 30 == 0 || In(2.0, 2.3))
            {
                static Landscape t_Land;
                std::memset(&t_Land, 0, sizeof(t_Land));
                t_Land.gloveDevices.gloveCount = 2;
                t_Land.gloveDevices.dongleCount = 1;
                for (uint32_t i = 0; i < 2; i++)
                {
                    t_Land.gloveDevices.gloves[i].id = i + 1;
                    t_Land.gloveDevices.gloves[i].side = i ? Side_Right : Side_Left;
                    t_Land.gloveDevices.gloves[i].transmissionStrength = -40;
                    t_Land.gloveDevices.gloves[i].pairedState =
                        (i == 1 && In(2.0, 2.3)) ? DevicePairedState_Unpaired : DevicePairedState_Paired;
                }
                t_Tel.NoteLandscape(t_Land);
            }
            t_Frame++;
            std::this_thread::sleep_for(std::chrono::microseconds(16667)); // 60 Hz
        }
    });

    std::thread t_Timer([&] {
        std::vector<uint64_t> t_TopicSeq(3, 0);
        while (!t_Done)
        {
            if (In(0.5, 0.8) || In(1.2, 1.5)) // publisher stall; timer blocked during SDK stall
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            {
                ManusTelemetry::TickScope t_Tick(t_Tel);
                for (uint32_t t_Glove = 1; t_Glove <= 2; t_Glove++)
                {
                    FakeGlove t_Msg;
                    uint64_t t_Pt;
                    {
                        std::lock_guard<std::mutex> t_Lock(t_CacheMutex);
                        if (!t_HaveData[t_Glove])
                            continue;
                        t_Msg = t_Cache[t_Glove];
                        t_Pt = t_CachePt[t_Glove];
                    }
                    const std::string t_Topic = "/manus_glove_" + std::to_string(t_Glove - 1);
                    const int64_t t_PreMono = ManusTelemetry::MonoNs(), t_PreUnix = Now();
                    t_Tel.NotePublish(t_Glove, t_Topic, ManusTimestamp{t_Pt}, ManusTelemetry::PoseHash(t_Msg),
                                      t_PreMono, t_PreUnix, ManusTelemetry::MonoNs());
                    // Transport faults act on delivery only.
                    int64_t t_Recv = t_PreUnix + 300000;
                    if (In(3.1, 3.3))
                        continue; // published, never recorded
                    if (In(2.6, 2.9))
                        t_Recv = t_Start + static_cast<int64_t>(2.9e9) + t_TopicSeq[t_Glove] * 1000; // delivered late
                    t_TopicSeq[t_Glove]++;
                    std::string t_Line = "{\"recv\":" + std::to_string(t_Recv) + ",\"glove\":{\"topic\":\"" + t_Topic +
                                         "\",\"id\":" + std::to_string(t_Glove) + ",\"side\":\"" +
                                         (t_Glove == 1 ? "Left" : "Right") + "\",\"poses\":[";
                    for (size_t i = 0; i < t_Msg.raw_nodes.size(); i++)
                    {
                        const Pose &p = t_Msg.raw_nodes[i].pose;
                        char t_Buf[256];
                        std::snprintf(t_Buf, sizeof(t_Buf), "%s[%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g]",
                                      i ? "," : "", p.position.x, p.position.y, p.position.z, p.orientation.x,
                                      p.orientation.y, p.orientation.z, p.orientation.w);
                        t_Line += t_Buf;
                    }
                    Emit(t_Line + "]}}");
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(8333));
        }
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(3800));
    t_Done = true;
    t_Sdk.join();
    t_Timer.join();
    t_Tel.Stop();
    for (const std::string &t_Line : g_Lines)
        std::printf("%s\n", t_Line.c_str());
    return 0;
}
