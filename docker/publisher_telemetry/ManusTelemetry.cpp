// See ManusTelemetry.hpp. Installed by apply_patch.py (manus-examples).
#include "ManusTelemetry.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <random>
#include <unistd.h>

namespace
{
// Minimal JSON object writer: records are flat apart from a few arrays.
class Json
{
public:
    Json &Key(const char *p_Key)
    {
        m_Out += m_First ? "" : ",";
        m_First = false;
        m_Out += '"';
        m_Out += p_Key;
        m_Out += "\":";
        return *this;
    }
    Json &Str(const char *p_Key, const std::string &p_Value) { Key(p_Key); Quote(p_Value); return *this; }
    Json &U64(const char *p_Key, uint64_t p_Value) { Key(p_Key); m_Out += std::to_string(p_Value); return *this; }
    Json &I64(const char *p_Key, int64_t p_Value) { Key(p_Key); m_Out += std::to_string(p_Value); return *this; }
    Json &Bool(const char *p_Key, bool p_Value) { Key(p_Key); m_Out += p_Value ? "true" : "false"; return *this; }
    Json &Null(const char *p_Key) { Key(p_Key); m_Out += "null"; return *this; }
    Json &Raw(const char *p_Key, const std::string &p_Json) { Key(p_Key); m_Out += p_Json; return *this; }
    std::string Done() { return "{" + m_Out + "}"; }

    void Quote(const std::string &p_Value)
    {
        m_Out += '"';
        for (unsigned char c : p_Value)
        {
            if (c == '"' || c == '\\')
            {
                m_Out += '\\';
                m_Out += static_cast<char>(c);
            }
            else if (c < 0x20)
            {
                char t_Escape[8];
                std::snprintf(t_Escape, sizeof(t_Escape), "\\u%04x", c);
                m_Out += t_Escape;
            }
            else
            {
                m_Out += static_cast<char>(c);
            }
        }
        m_Out += '"';
    }

private:
    std::string m_Out;
    bool m_First = true;
};

std::string Bounded(const char *p_Text, size_t p_Max)
{
    return std::string(p_Text, strnlen(p_Text, p_Max));
}

std::string VersionString(const Version &p_Version)
{
    return std::to_string(p_Version.major) + "." + std::to_string(p_Version.minor) + "." +
           std::to_string(p_Version.patch);
}

const char *PairedStateName(DevicePairedState p_State)
{
    switch (p_State)
    {
    case DevicePairedState_Paired: return "paired";
    case DevicePairedState_Unpaired: return "unpaired";
    case DevicePairedState_Pairing: return "pairing";
    default: return "unknown";
    }
}

const char *SideName(Side p_Side)
{
    switch (p_Side)
    {
    case Side_Left: return "Left";
    case Side_Right: return "Right";
    case Side_Center: return "Center";
    default: return "Invalid";
    }
}

const char *SeverityName(LogSeverity p_Severity)
{
    switch (p_Severity)
    {
    case LogSeverity_Debug: return "debug";
    case LogSeverity_Info: return "info";
    case LogSeverity_Warn: return "warn";
    case LogSeverity_Error: return "error";
    default: return "unknown";
    }
}

// SystemMessageType in SDK 3.1.1 is a prefix of 3.2.1, so one table serves both.
const char *const kSystemMessageNames[] = {
    "Unknown", "LibDebugReplugDongle", "LibDebugRxStall", "LibDebugTxStall",
    "TrackerError", "TrackerOk", "TrackerSystemOutOfDate",
    "GloveSanityErrorPSOCInit", "GloveSanityErrorQCBatV", "GloveSanityErrorQCLRACalib",
    "GloveSanityErrorQCFlexInit", "GloveSanityErrorQCIMUInit", "GloveSanityErrorQCIMUCalib",
    "GloveSanityErrorQCID", "GloveSanityErrorQCInterCPU",
    "SessionConnectionVersionMismatch", "TemporarySkeletonModified",
    "SessionRefusedDueToLicenseIssue", "LaunchDevTools", "MaxSeatMixing", "DeprecatedLicense",
    "LicenseSupportExpired", "NetSeatsExhausted"};

std::string ReadFirstLine(const char *p_Path)
{
    std::ifstream t_File(p_Path);
    std::string t_Line;
    std::getline(t_File, t_Line);
    return t_Line;
}

std::string HexU64(uint64_t p_Value)
{
    char t_Buffer[17];
    std::snprintf(t_Buffer, sizeof(t_Buffer), "%016" PRIx64, p_Value);
    return t_Buffer;
}

constexpr size_t kMaxQueuedRecords = 200000;
constexpr uint32_t kMaxLogsPerSecond = 100;
constexpr int64_t kLandscapeHeartbeatNs = 1000000000;
} // namespace

ManusTelemetry &ManusTelemetry::Instance()
{
    static ManusTelemetry s_Telemetry;
    return s_Telemetry;
}

ManusTelemetry::~ManusTelemetry()
{
    Stop();
}

int64_t ManusTelemetry::MonoNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

int64_t ManusTelemetry::UnixNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch()).count();
}

void ManusTelemetry::Start(Sink p_Sink, TimestampDecoder p_Decoder, const char *p_SourceSha, int p_FlushMs)
{
    {
        std::lock_guard<std::mutex> t_Lock(m_QueueMutex);
        if (m_Running)
            return;
        m_Sink = std::move(p_Sink);
        m_Decoder = p_Decoder;
        m_FlushMs = p_FlushMs > 0 ? p_FlushMs : 50;
        std::random_device t_Random;
        m_Session = HexU64((static_cast<uint64_t>(t_Random()) << 32) ^ t_Random());
        m_Running = true;
        m_StopRequested = false;
    }
    // The recorder usually subscribes after the publisher starts, so identity
    // is repeated in every batch header rather than sent once.
    char t_Host[256] = {0};
    gethostname(t_Host, sizeof(t_Host) - 1);
    {
        std::lock_guard<std::mutex> t_Lock(m_StateMutex);
        m_SourceSha = p_SourceSha ? p_SourceSha : "";
        m_Identity = Json()
                         .I64("pid", getpid())
                         .Str("host", t_Host)
                         .Str("boot_id", ReadFirstLine("/proc/sys/kernel/random/boot_id"))
                         .I64("start_unix", UnixNs())
                         .I64("start_mono", MonoNs())
                         .Done();
    }
    m_FlushThread = std::thread(&ManusTelemetry::FlushLoop, this);
}

void ManusTelemetry::Stop()
{
    {
        std::lock_guard<std::mutex> t_Lock(m_QueueMutex);
        if (!m_Running)
            return;
        m_StopRequested = true;
    }
    m_QueueCondition.notify_all();
    if (m_FlushThread.joinable())
        m_FlushThread.join();
    std::lock_guard<std::mutex> t_Lock(m_QueueMutex);
    m_Running = false;
    m_Sink = nullptr; // Release the ROS publisher before static destruction.
}

void ManusTelemetry::Enqueue(std::string p_Record)
{
    std::lock_guard<std::mutex> t_Lock(m_QueueMutex);
    if (!m_Running)
        return;
    if (m_Queue.size() >= kMaxQueuedRecords)
    {
        m_Dropped++;
        return;
    }
    m_Queue.push_back(std::move(p_Record));
}

std::string ManusTelemetry::BuildBatch(std::vector<std::string> &p_Records, uint64_t p_Dropped)
{
    std::string t_Records = "[";
    for (size_t i = 0; i < p_Records.size(); i++)
    {
        if (i)
            t_Records += ',';
        t_Records += p_Records[i];
    }
    t_Records += ']';
    uint64_t t_LogsDropped;
    std::string t_Identity, t_SourceSha;
    Json t_Registrations;
    {
        std::lock_guard<std::mutex> t_Lock(m_StateMutex);
        t_LogsDropped = m_LogsDropped;
        t_Identity = m_Identity;
        t_SourceSha = m_SourceSha;
        for (const auto &t_Entry : m_Registrations)
            t_Registrations.I64(t_Entry.first.c_str(), t_Entry.second);
    }
    return Json()
        .Str("schema", "manus.publisher_telemetry.v1")
        .Str("session", m_Session)
        .Str("source_sha", t_SourceSha)
        .I64("flush_ms", m_FlushMs)
        .Raw("process", t_Identity)
        .Raw("registrations", t_Registrations.Done())
        .U64("batch", m_BatchSeq++)
        .I64("mono", MonoNs())
        .I64("unix", UnixNs())
        .U64("dropped_records", p_Dropped)
        .U64("dropped_logs", t_LogsDropped)
        .Raw("records", t_Records)
        .Done();
}

void ManusTelemetry::FlushLoop()
{
    // An empty batch is still sent every period: it is the heartbeat that
    // separates "this process stopped" from "nothing happened".
    std::unique_lock<std::mutex> t_Lock(m_QueueMutex);
    while (true)
    {
        const bool t_Stopping = m_QueueCondition.wait_for(t_Lock, std::chrono::milliseconds(m_FlushMs),
                                                          [this] { return m_StopRequested; });
        std::vector<std::string> t_Records;
        t_Records.swap(m_Queue);
        const uint64_t t_Dropped = m_Dropped;
        Sink t_Sink = m_Sink;
        t_Lock.unlock();
        const std::string t_Batch = BuildBatch(t_Records, t_Dropped);
        try
        {
            if (t_Sink)
                t_Sink(t_Batch);
        }
        catch (...)
        {
            // Publishing fails once rclcpp has shut down; telemetry must never crash the publisher.
        }
        t_Lock.lock();
        if (t_Stopping)
            return;
    }
}

std::string ManusTelemetry::Timestamp(ManusTimestamp p_Time)
{
    if (m_Decoder == nullptr || p_Time.time == 0)
        return "null";
    ManusTimestampInfo t_Info;
    std::memset(&t_Info, 0, sizeof(t_Info));
    if (m_Decoder(p_Time, &t_Info) != SDKReturnCode_Success || t_Info.timecode || t_Info.year == 0)
        return "null";
    std::tm t_Tm{};
    t_Tm.tm_year = static_cast<int>(t_Info.year) - 1900;
    t_Tm.tm_mon = t_Info.month - 1;
    t_Tm.tm_mday = t_Info.day;
    t_Tm.tm_hour = t_Info.hour;
    t_Tm.tm_min = t_Info.minute;
    t_Tm.tm_sec = t_Info.second;
    return std::to_string(static_cast<int64_t>(timegm(&t_Tm)) * 1000 + t_Info.fraction);
}

void ManusTelemetry::NoteSkeleton(uint32_t p_GloveId, uint32_t p_NodeCount, ManusTimestamp p_PublishTime)
{
    const int64_t t_Mono = MonoNs();
    const int64_t t_Unix = UnixNs();
    uint64_t t_Seq;
    {
        std::lock_guard<std::mutex> t_Lock(m_StateMutex);
        GloveState &t_State = m_Gloves[p_GloveId];
        t_Seq = ++t_State.skeletonSeq;
        t_State.skeletonPt = p_PublishTime.time;
        t_State.skeletonMono = t_Mono;
    }
    Enqueue(Json()
                .Str("t", "skel")
                .I64("mono", t_Mono)
                .I64("unix", t_Unix)
                .U64("glove", p_GloveId)
                .U64("nodes", p_NodeCount)
                .U64("seq", t_Seq)
                .U64("pt", p_PublishTime.time)
                .Raw("pt_ms", Timestamp(p_PublishTime))
                .Done());
}

void ManusTelemetry::NoteErgonomics(uint32_t p_GloveId, ManusTimestamp p_PublishTime)
{
    const int64_t t_Mono = MonoNs();
    const int64_t t_Unix = UnixNs();
    uint64_t t_Seq;
    {
        std::lock_guard<std::mutex> t_Lock(m_StateMutex);
        GloveState &t_State = m_Gloves[p_GloveId];
        t_Seq = ++t_State.ergonomicsSeq;
        t_State.ergonomicsPt = p_PublishTime.time;
    }
    Enqueue(Json()
                .Str("t", "ergo")
                .I64("mono", t_Mono)
                .I64("unix", t_Unix)
                .U64("glove", p_GloveId)
                .U64("seq", t_Seq)
                .U64("pt", p_PublishTime.time)
                .Raw("pt_ms", Timestamp(p_PublishTime))
                .Done());
}

void ManusTelemetry::NoteRawDevice(uint32_t p_DeviceId, uint32_t p_SensorCount, ManusTimestamp p_PublishTime)
{
    const int64_t t_Mono = MonoNs();
    const int64_t t_Unix = UnixNs();
    uint64_t t_Seq;
    {
        std::lock_guard<std::mutex> t_Lock(m_StateMutex);
        t_Seq = ++m_Gloves[p_DeviceId].rawDeviceSeq;
    }
    Enqueue(Json()
                .Str("t", "raw")
                .I64("mono", t_Mono)
                .I64("unix", t_Unix)
                .U64("glove", p_DeviceId)
                .U64("sensors", p_SensorCount)
                .U64("seq", t_Seq)
                .U64("pt", p_PublishTime.time)
                .Raw("pt_ms", Timestamp(p_PublishTime))
                .Done());
}

std::string ManusTelemetry::LandscapeJson(const Landscape &p_Landscape, bool p_IncludeLastSeen)
{
    const DeviceLandscape &t_Devices = p_Landscape.gloveDevices;
    std::string t_Dongles = "[";
    for (uint32_t i = 0; i < t_Devices.dongleCount && i < MAX_NUMBER_OF_DONGLES; i++)
    {
        const DongleLandscapeData &t_Dongle = t_Devices.dongles[i];
        Json t_Json;
        t_Json.U64("id", t_Dongle.id)
            .I64("channel", t_Dongle.channel)
            .U64("charging", t_Dongle.chargingState)
            .I64("update", t_Dongle.updateStatus)
            .Str("fw", VersionString(t_Dongle.firmwareVersion))
            .U64("left", t_Dongle.leftGloveID)
            .U64("right", t_Dongle.rightGloveID);
        if (p_IncludeLastSeen)
            t_Json.U64("last_seen", t_Dongle.lastSeen.time).Raw("last_seen_ms", Timestamp(t_Dongle.lastSeen));
        t_Dongles += (i ? "," : "") + t_Json.Done();
    }
    t_Dongles += "]";

    std::string t_Gloves = "[";
    for (uint32_t i = 0; i < t_Devices.gloveCount && i < MAX_NUMBER_OF_GLOVES; i++)
    {
        const GloveLandscapeData &t_Glove = t_Devices.gloves[i];
        std::string t_Imu = "[";
        for (int j = 0; j < MAX_NUM_IMUS_ON_GLOVE; j++)
        {
            const IMUCalibrationInfo &t_Cal = t_Glove.iMUCalibrationInfo[j];
            t_Imu += (j ? ",[" : "[") + std::to_string(t_Cal.mag) + "," + std::to_string(t_Cal.acc) + "," +
                     std::to_string(t_Cal.gyr) + "," + std::to_string(t_Cal.sys) + "]";
        }
        t_Imu += "]";
        Json t_Json;
        t_Json.U64("id", t_Glove.id)
            .Str("side", SideName(t_Glove.side))
            .Str("paired", PairedStateName(t_Glove.pairedState))
            .U64("dongle", t_Glove.dongleID)
            .U64("battery", t_Glove.batteryPercentage)
            .I64("tx_strength", t_Glove.transmissionStrength)
            .Bool("excluded", t_Glove.excluded)
            .I64("update", t_Glove.updateStatus)
            .Str("fw", VersionString(t_Glove.firmwareVersion))
            .Raw("imu_cal", t_Imu);
        if (p_IncludeLastSeen)
            t_Json.U64("last_seen", t_Glove.lastSeen.time).Raw("last_seen_ms", Timestamp(t_Glove.lastSeen));
        t_Gloves += (i ? "," : "") + t_Json.Done();
    }
    t_Gloves += "]";
    return Json().Raw("dongles", t_Dongles).Raw("gloves", t_Gloves).Done();
}

void ManusTelemetry::NoteLandscape(const Landscape &p_Landscape)
{
    // Snapshot on any change (lastSeen excluded, it ticks constantly) and at
    // least once per second so signal strength and battery form a time series.
    const int64_t t_Mono = MonoNs();
    const std::string t_Signature = LandscapeJson(p_Landscape, false);
    bool t_Changed;
    uint64_t t_Callbacks;
    {
        std::lock_guard<std::mutex> t_Lock(m_StateMutex);
        t_Callbacks = ++m_LandscapeCallbacks;
        t_Changed = t_Signature != m_LastLandscape;
        if (!t_Changed && t_Mono - m_LastLandscapeMono < kLandscapeHeartbeatNs)
            return;
        m_LastLandscape = t_Signature;
        m_LastLandscapeMono = t_Mono;
    }
    Enqueue(Json()
                .Str("t", "land")
                .I64("mono", t_Mono)
                .I64("unix", UnixNs())
                .U64("callbacks", t_Callbacks)
                .Bool("changed", t_Changed)
                .U64("dongle_count", p_Landscape.gloveDevices.dongleCount)
                .U64("glove_count", p_Landscape.gloveDevices.gloveCount)
                .Raw("devices", LandscapeJson(p_Landscape, true))
                .Done());
}

void ManusTelemetry::NoteRegistration(const char *p_Callback, int p_ReturnCode)
{
    {
        std::lock_guard<std::mutex> t_Lock(m_StateMutex);
        m_Registrations[p_Callback] = p_ReturnCode;
    }
    Enqueue(Json()
                .Str("t", "reg")
                .I64("mono", MonoNs())
                .I64("unix", UnixNs())
                .Str("callback", p_Callback)
                .I64("code", p_ReturnCode)
                .Done());
}

void ManusTelemetry::OnConnect(const ManusHost *const p_Host)
{
    Json t_Json;
    t_Json.Str("t", "connect").I64("mono", MonoNs()).I64("unix", UnixNs());
    if (p_Host)
        t_Json.Str("host", Bounded(p_Host->hostName, MAX_NUM_CHARS_IN_HOST_NAME))
            .Str("ip", Bounded(p_Host->ipAddress, MAX_NUM_CHARS_IN_IP_ADDRESS))
            .Str("core", VersionString(p_Host->manusCoreVersion));
    Instance().Enqueue(t_Json.Done());
}

void ManusTelemetry::OnDisconnect(const ManusHost *const p_Host)
{
    Json t_Json;
    t_Json.Str("t", "disconnect").I64("mono", MonoNs()).I64("unix", UnixNs());
    if (p_Host)
        t_Json.Str("host", Bounded(p_Host->hostName, MAX_NUM_CHARS_IN_HOST_NAME))
            .Str("ip", Bounded(p_Host->ipAddress, MAX_NUM_CHARS_IN_IP_ADDRESS))
            .Str("core", VersionString(p_Host->manusCoreVersion));
    Instance().Enqueue(t_Json.Done());
}

void ManusTelemetry::OnLog(LogSeverity p_Severity, const char *const p_Log, uint32_t p_Length)
{
    ManusTelemetry &t_Self = Instance();
    const int64_t t_Mono = MonoNs();
    {
        std::lock_guard<std::mutex> t_Lock(t_Self.m_StateMutex);
        if (t_Mono - t_Self.m_LogWindowMono >= 1000000000)
        {
            t_Self.m_LogWindowMono = t_Mono;
            t_Self.m_LogsInWindow = 0;
        }
        if (++t_Self.m_LogsInWindow > kMaxLogsPerSecond)
        {
            t_Self.m_LogsDropped++;
            return;
        }
    }
    const size_t t_Length = p_Log ? strnlen(p_Log, p_Length < 2048 ? p_Length : 2048) : 0;
    t_Self.Enqueue(Json()
                       .Str("t", "log")
                       .I64("mono", t_Mono)
                       .I64("unix", UnixNs())
                       .Str("sev", SeverityName(p_Severity))
                       .Str("text", std::string(p_Log ? p_Log : "", t_Length))
                       .Done());
}

void ManusTelemetry::OnSystem(const SystemMessage *const p_Message)
{
    if (p_Message == nullptr)
        return;
    const int t_Type = static_cast<int>(p_Message->type);
    const int t_Known = static_cast<int>(sizeof(kSystemMessageNames) / sizeof(kSystemMessageNames[0]));
    const std::string t_Name = (t_Type >= 0 && t_Type < t_Known) ? kSystemMessageNames[t_Type]
                                                                 : "Unknown(" + std::to_string(t_Type) + ")";
    Instance().Enqueue(Json()
                           .Str("t", "sys")
                           .I64("mono", MonoNs())
                           .I64("unix", UnixNs())
                           .Str("type", t_Name)
                           .I64("code", t_Type)
                           .Str("info", Bounded(p_Message->infoString, MAX_NUM_CHARS_IN_SYSTEM_ERROR_MESSAGE))
                           .U64("uint", p_Message->infoUInt)
                           .Done());
}

ManusTelemetry::TickScope::TickScope(ManusTelemetry &p_Telemetry)
    : m_Telemetry(p_Telemetry), m_Mono(MonoNs()), m_Unix(UnixNs())
{
    std::lock_guard<std::mutex> t_Lock(m_Telemetry.m_StateMutex);
    m_Seq = ++m_Telemetry.m_TickSeq;
    m_Telemetry.m_TickPublished.clear();
}

ManusTelemetry::TickScope::~TickScope()
{
    const int64_t t_End = MonoNs();
    std::string t_Published = "[";
    {
        std::lock_guard<std::mutex> t_Lock(m_Telemetry.m_StateMutex);
        for (size_t i = 0; i < m_Telemetry.m_TickPublished.size(); i++)
            t_Published += (i ? "," : "") + std::to_string(m_Telemetry.m_TickPublished[i]);
    }
    t_Published += "]";
    m_Telemetry.Enqueue(Json()
                            .Str("t", "tick")
                            .U64("seq", m_Seq)
                            .I64("mono", m_Mono)
                            .I64("unix", m_Unix)
                            .I64("dur_ns", t_End - m_Mono)
                            .Raw("pub", t_Published)
                            .Done());
}

void ManusTelemetry::NotePublish(uint32_t p_GloveId, const std::string &p_Topic, ManusTimestamp p_SkeletonPublishTime,
                                 uint64_t p_PoseHash, int64_t p_PreMono, int64_t p_PreUnix, int64_t p_PostMono)
{
    Json t_Json;
    {
        std::lock_guard<std::mutex> t_Lock(m_StateMutex);
        m_TickPublished.push_back(p_GloveId);
        GloveState &t_State = m_Gloves[p_GloveId];
        const uint64_t t_TopicSeq = t_State.topicSeq[p_Topic]++;
        // "fresh": the skeleton in this message carries a different SDK publish
        // time than the previous message for this glove, i.e. it is not a
        // re-send of the cached pose by the 120 Hz timer.
        const bool t_Fresh = !t_State.published || p_SkeletonPublishTime.time != t_State.lastPublishedPt;
        t_Json.Str("t", "pub")
            .U64("glove", p_GloveId)
            .Str("topic", p_Topic)
            .U64("topic_seq", t_TopicSeq)
            .I64("mono", p_PreMono)
            .I64("unix", p_PreUnix)
            .I64("pub_ns", p_PostMono - p_PreMono)
            .U64("pt", p_SkeletonPublishTime.time)
            .Bool("fresh", t_Fresh)
            .Bool("same_pose", t_State.published && p_PoseHash == t_State.lastPublishedHash)
            .U64("skel_seq", t_State.skeletonSeq)
            .U64("new_skel", t_State.skeletonSeq - t_State.lastPublishedSkeletonSeq)
            .U64("ergo_seq", t_State.ergonomicsSeq)
            .U64("new_ergo", t_State.ergonomicsSeq - t_State.lastPublishedErgonomicsSeq)
            .U64("ergo_pt", t_State.ergonomicsPt)
            .Str("hash", HexU64(p_PoseHash));
        if (p_SkeletonPublishTime.time == t_State.skeletonPt && t_State.skeletonMono != 0)
            t_Json.I64("skel_age_ns", p_PreMono - t_State.skeletonMono);
        else
            t_Json.Null("skel_age_ns");
        t_State.published = true;
        t_State.lastPublishedPt = p_SkeletonPublishTime.time;
        t_State.lastPublishedHash = p_PoseHash;
        t_State.lastPublishedSkeletonSeq = t_State.skeletonSeq;
        t_State.lastPublishedErgonomicsSeq = t_State.ergonomicsSeq;
    }
    t_Json.Raw("pt_ms", Timestamp(p_SkeletonPublishTime));
    Enqueue(t_Json.Done());
}
