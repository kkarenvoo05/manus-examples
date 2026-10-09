// Timing and connection-state telemetry for MANUS manus_data_publisher.
//
// Installed into the MANUS ROS2 package by apply_patch.py (manus-examples).
// The glove topics are unchanged; everything here goes to a separate
// /manus_telemetry topic as batched JSON (std_msgs/String), flushed by its own
// thread so a stalled publish timer or SDK thread still leaves a heartbeat.
//
// Clocks: "mono" is std::chrono::steady_clock (CLOCK_MONOTONIC) and "unix" is
// std::chrono::system_clock, both in nanoseconds, read inside this process.
// "pt" fields are the SDK's own ManusTimestamp (raw) and, when decodable,
// "pt_ms" is that timestamp as Unix milliseconds.
#pragma once

#include "ManusSDKTypes.h"

#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class ManusTelemetry
{
public:
    using Sink = std::function<void(const std::string &)>;
    using TimestampDecoder = SDKReturnCode (*)(ManusTimestamp, ManusTimestampInfo *);

    static ManusTelemetry &Instance();

    void Start(Sink p_Sink, TimestampDecoder p_Decoder, const char *p_SourceSha, int p_FlushMs = 50);
    void Stop();

    // SDK stream hooks, called on SDK threads.
    void NoteSkeleton(uint32_t p_GloveId, uint32_t p_NodeCount, ManusTimestamp p_PublishTime);
    void NoteErgonomics(uint32_t p_GloveId, ManusTimestamp p_PublishTime);
    void NoteRawDevice(uint32_t p_DeviceId, uint32_t p_SensorCount, ManusTimestamp p_PublishTime);
    void NoteLandscape(const Landscape &p_Landscape);
    void NoteRegistration(const char *p_Callback, int p_ReturnCode);

    static void OnConnect(const ManusHost *const p_Host);
    static void OnDisconnect(const ManusHost *const p_Host);
    static void OnLog(LogSeverity p_Severity, const char *const p_Log, uint32_t p_Length);
    static void OnSystem(const SystemMessage *const p_Message);

    // Publish timer hooks. One TickScope per timer callback; it reports the
    // callback's duration and which gloves were published, on every exit path.
    class TickScope
    {
    public:
        explicit TickScope(ManusTelemetry &p_Telemetry);
        ~TickScope();
        TickScope(const TickScope &) = delete;
        TickScope &operator=(const TickScope &) = delete;

    private:
        ManusTelemetry &m_Telemetry;
        uint64_t m_Seq;
        int64_t m_Mono;
        int64_t m_Unix;
    };

    void NotePublish(uint32_t p_GloveId, const std::string &p_Topic, ManusTimestamp p_SkeletonPublishTime,
                     uint64_t p_PoseHash, int64_t p_PreMono, int64_t p_PreUnix, int64_t p_PostMono);

    // FNV-1a 64 over every raw node pose in message order: position x, y, z then
    // orientation x, y, z, w, each as the 8 little-endian bytes of the float64.
    // tools/diagnose_glove_telemetry.py recomputes it from the bag to pair each
    // recorded glove message with its publish record.
    template <class GloveMsg>
    static uint64_t PoseHash(const GloveMsg &p_Msg)
    {
        uint64_t t_Hash = 14695981039346656037ull;
        for (const auto &t_Node : p_Msg.raw_nodes)
        {
            const double t_Values[7] = {t_Node.pose.position.x, t_Node.pose.position.y, t_Node.pose.position.z,
                                        t_Node.pose.orientation.x, t_Node.pose.orientation.y,
                                        t_Node.pose.orientation.z, t_Node.pose.orientation.w};
            for (double t_Value : t_Values)
            {
                uint64_t t_Bits;
                std::memcpy(&t_Bits, &t_Value, sizeof(t_Bits));
                for (int t_Byte = 0; t_Byte < 8; t_Byte++)
                {
                    t_Hash ^= (t_Bits >> (8 * t_Byte)) & 0xffu;
                    t_Hash *= 1099511628211ull;
                }
            }
        }
        return t_Hash;
    }

    static int64_t MonoNs();
    static int64_t UnixNs();

private:
    ManusTelemetry() = default;
    ~ManusTelemetry();

    struct GloveState
    {
        uint64_t skeletonSeq = 0;
        uint64_t skeletonPt = 0;
        int64_t skeletonMono = 0;
        uint64_t ergonomicsSeq = 0;
        uint64_t ergonomicsPt = 0;
        uint64_t rawDeviceSeq = 0;
        bool published = false;
        uint64_t lastPublishedSkeletonSeq = 0;
        uint64_t lastPublishedErgonomicsSeq = 0;
        uint64_t lastPublishedPt = 0;
        uint64_t lastPublishedHash = 0;
        std::map<std::string, uint64_t> topicSeq;
    };

    void Enqueue(std::string p_Record);
    void FlushLoop();
    std::string BuildBatch(std::vector<std::string> &p_Records, uint64_t p_Dropped);
    std::string Timestamp(ManusTimestamp p_Time);
    std::string LandscapeJson(const Landscape &p_Landscape, bool p_IncludeLastSeen);

    std::mutex m_StateMutex;
    std::map<uint32_t, GloveState> m_Gloves;
    std::vector<uint32_t> m_TickPublished;
    uint64_t m_TickSeq = 0;
    std::string m_LastLandscape;
    int64_t m_LastLandscapeMono = 0;
    uint64_t m_LandscapeCallbacks = 0;
    int64_t m_LogWindowMono = 0;
    uint32_t m_LogsInWindow = 0;
    uint64_t m_LogsDropped = 0;

    std::mutex m_QueueMutex;
    std::condition_variable m_QueueCondition;
    std::vector<std::string> m_Queue;
    uint64_t m_Dropped = 0;
    bool m_Running = false;
    bool m_StopRequested = false;
    std::thread m_FlushThread;
    Sink m_Sink;
    TimestampDecoder m_Decoder = nullptr;
    std::string m_Session;
    std::string m_SourceSha;
    std::string m_Identity; // pid, host, boot, start time: repeated in every batch
    std::map<std::string, int> m_Registrations;
    uint64_t m_BatchSeq = 0;
    int m_FlushMs = 50;
};
