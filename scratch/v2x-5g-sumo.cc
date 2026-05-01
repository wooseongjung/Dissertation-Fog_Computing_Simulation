// V2X 5G + SUMO (ns-2 trace) baseline scenario for NS-3.46

#include "ns3/applications-module.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/simulator.h"
#include "ns3/tag.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("V2x5gSumo");

struct Ipv4Hash
{
    std::size_t operator()(const Ipv4Address& a) const
    {
        return a.Get();
    }
};

struct CellRntiKey
{
    uint16_t cellId;
    uint16_t rnti;

    bool operator==(const CellRntiKey& other) const
    {
        return cellId == other.cellId && rnti == other.rnti;
    }
};

struct CellRntiKeyHash
{
    std::size_t operator()(const CellRntiKey& key) const
    {
        return (static_cast<std::size_t>(key.cellId) << 16) ^ key.rnti;
    }
};

enum NodeType
{
    NODE_TYPE_RSU = 0, // stationary fog node (CFN / RSU)
    NODE_TYPE_BUS = 1, // VFN
    NODE_TYPE_CAR = 2
};

static std::unordered_map<uint32_t, NodeType> g_nodeTypes;
static std::unordered_map<uint32_t, uint32_t> g_fogQueueSizes;
static std::unordered_map<uint32_t, Ptr<Node>> g_nodesById;
static std::unordered_map<uint32_t, Ipv4Address> g_nodeIpv4;
static std::unordered_map<uint64_t, uint32_t> g_imsiToNodeId;
static std::unordered_map<uint64_t, uint16_t> g_lastCellByImsi;
static std::unordered_map<uint64_t, Time> g_handoverStartByImsi;
static std::unordered_map<uint16_t, uint32_t> g_cellIdToGnbNodeId;
static std::unordered_map<uint32_t, uint16_t> g_gnbNodeToCellId;
static std::unordered_map<uint32_t, uint32_t> g_gnbDeviceIndexByNodeId;
static std::unordered_map<uint32_t, uint32_t> g_vehicleDeviceIndexByNodeId;
static std::unordered_map<uint32_t, uint16_t> g_vehicleCellByNodeId;
static std::unordered_map<uint32_t, uint16_t> g_vehicleRntiByNodeId;
static std::unordered_map<CellRntiKey, uint32_t, CellRntiKeyHash> g_vehicleNodeByCellRnti;
static std::unordered_set<std::string> g_downlinkTraceAttachmentKeys;
static std::unordered_map<uint32_t, NodeType> g_vehicleServingFogType;
static std::unordered_map<uint32_t, double> g_vehicleServingRelVelKmh;
static std::ofstream g_handoverLog;
static std::ofstream g_methodologyLog;
static std::ofstream g_responsePathLog;
static std::ofstream g_downlinkRadioLog;
static std::ofstream g_assocDiagnosticsLog;
static double g_bandwidthHz = 100e6;
static std::unordered_map<uint32_t, Time> g_vehicleFirstAssocTime;

// Per-architecture task counters. Each Poisson tick increments
// offered_tasks before any association check; tasks dropped because no
// fog is associated go into no_association_drops, transmitted ones into
// sent_tasks. The split lets the aggregated run schema show whether
// VFN and CFN see the same offered load.
struct VehicleAppCounters
{
    uint64_t offered_tasks{0};
    uint64_t sent_tasks{0};
    uint64_t no_association_drops{0};
};
static VehicleAppCounters g_counters;
static std::unordered_map<uint32_t, NodeType> g_vehicleSelectedFogType;
static constexpr double kPi = 3.141592653589793;
// CRE bias disabled so association is on raw radio score.
static constexpr double kVfnCreBiasDb = 0.0;

static double
CalculateRelativeVelocity(Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
    if (!a || !b)
    {
        return 0.0;
    }
    Vector va = a->GetVelocity();
    Vector vb = b->GetVelocity();
    Vector dv(va.x - vb.x, va.y - vb.y, va.z - vb.z);
    return std::sqrt(dv.x * dv.x + dv.y * dv.y + dv.z * dv.z);
}

static double
DistanceSquared2d(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

static std::vector<Vector>
SelectSpreadAnchors(const std::vector<Vector>& candidates, uint32_t count, uint32_t seedOffset = 0)
{
    std::vector<Vector> anchors;
    if (candidates.empty() || count == 0)
    {
        return anchors;
    }

    const uint32_t n = static_cast<uint32_t>(candidates.size());
    anchors.reserve(count);

    std::vector<bool> chosen(n, false);
    std::vector<double> minDist2(n, std::numeric_limits<double>::infinity());

    uint32_t first = seedOffset % n;
    anchors.push_back(candidates[first]);
    chosen[first] = true;
    for (uint32_t i = 0; i < n; ++i)
    {
        minDist2[i] = DistanceSquared2d(candidates[i], candidates[first]);
    }

    while (anchors.size() < std::min<uint32_t>(count, n))
    {
        uint32_t bestIdx = n;
        double bestScore = -1.0;
        for (uint32_t i = 0; i < n; ++i)
        {
            if (chosen[i])
            {
                continue;
            }
            if (minDist2[i] > bestScore)
            {
                bestScore = minDist2[i];
                bestIdx = i;
            }
        }

        if (bestIdx == n)
        {
            break;
        }

        anchors.push_back(candidates[bestIdx]);
        chosen[bestIdx] = true;

        for (uint32_t i = 0; i < n; ++i)
        {
            if (chosen[i])
            {
                continue;
            }
            minDist2[i] =
                std::min(minDist2[i], DistanceSquared2d(candidates[i], candidates[bestIdx]));
        }
    }

    while (anchors.size() < count)
    {
        anchors.push_back(anchors[anchors.size() % n]);
    }

    return anchors;
}

static Vector
OffsetAlongVelocityNormal(const Vector& basePos,
                          const Vector& velocity,
                          double lateralOffsetMeters,
                          double z)
{
    const double speed = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y);
    Vector lateral(0.0, 1.0, 0.0);
    if (speed > 1e-6)
    {
        lateral.x = -velocity.y / speed;
        lateral.y = velocity.x / speed;
    }

    return Vector(basePos.x + lateralOffsetMeters * lateral.x,
                  basePos.y + lateralOffsetMeters * lateral.y,
                  z);
}

static void
UpdateBusFollowerMobility(Ptr<Node> busNode,
                          Ptr<Node> referenceVehicleNode,
                          double busHeight,
                          double lateralOffsetMeters,
                          Time updatePeriod)
{
    if (!busNode || !referenceVehicleNode)
    {
        return;
    }

    Ptr<MobilityModel> refMob = referenceVehicleNode->GetObject<MobilityModel>();
    Ptr<ConstantVelocityMobilityModel> busMob = busNode->GetObject<ConstantVelocityMobilityModel>();
    if (!refMob || !busMob)
    {
        return;
    }

    const Vector refPos = refMob->GetPosition();
    const Vector refVel = refMob->GetVelocity();
    busMob->SetPosition(OffsetAlongVelocityNormal(refPos, refVel, lateralOffsetMeters, busHeight));
    busMob->SetVelocity(Vector(refVel.x, refVel.y, 0.0));

    Simulator::Schedule(updatePeriod,
                        &UpdateBusFollowerMobility,
                        busNode,
                        referenceVehicleNode,
                        busHeight,
                        lateralOffsetMeters,
                        updatePeriod);
}

static void
RegisterKnownNode(Ptr<Node> node)
{
    if (!node)
    {
        return;
    }
    g_nodesById[node->GetId()] = node;
}

static void
RegisterNodeType(Ptr<Node> node, NodeType type)
{
    if (!node)
    {
        return;
    }
    RegisterKnownNode(node);
    g_nodeTypes[node->GetId()] = type;
}

static NodeType
GetNodeType(uint32_t nodeId)
{
    auto it = g_nodeTypes.find(nodeId);
    if (it == g_nodeTypes.end())
    {
        return NODE_TYPE_CAR;
    }
    return it->second;
}

static const char*
NodeTypeLabel(NodeType type)
{
    switch (type)
    {
    case NODE_TYPE_RSU:
        return "CFN";
    case NODE_TYPE_BUS:
        return "VFN";
    default:
        return "CAR";
    }
}

class TaskResponseTag : public Tag
{
  public:
    TaskResponseTag() = default;

    explicit TaskResponseTag(uint32_t taskId)
        : m_taskId(taskId)
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::TaskResponseTag").SetParent<Tag>().AddConstructor<TaskResponseTag>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return sizeof(m_taskId);
    }

    void Serialize(TagBuffer i) const override
    {
        i.WriteU32(m_taskId);
    }

    void Deserialize(TagBuffer i) override
    {
        m_taskId = i.ReadU32();
    }

    void Print(std::ostream& os) const override
    {
        os << "taskId=" << m_taskId;
    }

    void SetTaskId(uint32_t taskId)
    {
        m_taskId = taskId;
    }

    uint32_t GetTaskId() const
    {
        return m_taskId;
    }

  private:
    uint32_t m_taskId{0};
};

static bool
ExtractTaskResponseId(Ptr<const Packet> packet, uint32_t* taskId)
{
    if (!packet)
    {
        return false;
    }

    TaskResponseTag tag;
    if (!packet->PeekPacketTag(tag))
    {
        return false;
    }

    if (taskId)
    {
        *taskId = tag.GetTaskId();
    }
    return true;
}

static void
LogMethodologyRow(double timeSeconds,
                  uint32_t vehicleId,
                  NodeType connectedType,
                  double relVelKmh,
                  const char* packetState,
                  uint32_t handoverEvent,
                  uint32_t fogReassociationEvent)
{
    if (!g_methodologyLog.is_open())
    {
        return;
    }

    g_methodologyLog << timeSeconds << ',' << vehicleId << ',' << NodeTypeLabel(connectedType)
                     << ',' << relVelKmh << ',' << packetState << ',' << handoverEvent << ','
                     << fogReassociationEvent << '\n';
}

static void
UpdateFogQueueSize(uint32_t nodeId, uint32_t size)
{
    g_fogQueueSizes[nodeId] = size;
}

static void
RegisterNodeAddress(Ptr<Node> node, Ipv4Address address)
{
    if (!node)
    {
        return;
    }
    RegisterKnownNode(node);
    g_nodeIpv4[node->GetId()] = address;
}

static Ipv4Address
LookupNodeAddress(uint32_t nodeId)
{
    auto it = g_nodeIpv4.find(nodeId);
    if (it == g_nodeIpv4.end())
    {
        return Ipv4Address::GetZero();
    }
    return it->second;
}

static void
UpdateVehicleBearerMapping(uint32_t vehicleNodeId, uint16_t cellId, uint16_t rnti)
{
    auto oldCellIt = g_vehicleCellByNodeId.find(vehicleNodeId);
    auto oldRntiIt = g_vehicleRntiByNodeId.find(vehicleNodeId);
    if (oldCellIt != g_vehicleCellByNodeId.end() && oldRntiIt != g_vehicleRntiByNodeId.end())
    {
        g_vehicleNodeByCellRnti.erase(CellRntiKey{oldCellIt->second, oldRntiIt->second});
    }

    g_vehicleCellByNodeId[vehicleNodeId] = cellId;
    g_vehicleRntiByNodeId[vehicleNodeId] = rnti;
    g_vehicleNodeByCellRnti[CellRntiKey{cellId, rnti}] = vehicleNodeId;
}

static void NotifyDlTaskRlcTx(uint32_t gnbNodeId,
                              uint32_t vehicleNodeId,
                              uint16_t cellId,
                              uint16_t rnti,
                              uint8_t lcid,
                              Ptr<const Packet> packet);
static void NotifyDlTaskRlcTxDrop(uint32_t gnbNodeId,
                                  uint32_t vehicleNodeId,
                                  uint16_t cellId,
                                  uint16_t rnti,
                                  uint8_t lcid,
                                  Ptr<const Packet> packet);
static void NotifyDlTaskRlcRx(uint32_t gnbNodeId,
                              uint32_t vehicleNodeId,
                              uint16_t cellId,
                              uint16_t rnti,
                              uint8_t lcid,
                              Ptr<const Packet> packet,
                              uint64_t delayNs);

static void
LogDownlinkRadioEvent(const char* stage,
                      uint32_t taskId,
                      uint32_t gnbNodeId,
                      uint32_t vehicleNodeId,
                      uint16_t cellId,
                      uint16_t rnti,
                      uint8_t lcid,
                      uint32_t bytes,
                      double delaySeconds)
{
    if (!g_downlinkRadioLog.is_open())
    {
        return;
    }

    g_downlinkRadioLog << Simulator::Now().GetSeconds() << ',' << stage << ',' << taskId << ','
                       << gnbNodeId << ',' << vehicleNodeId << ',' << cellId << ',' << rnti << ','
                       << static_cast<uint32_t>(lcid) << ',' << bytes << ',' << delaySeconds
                       << '\n';
}

static void
ConnectDownlinkTaskTracesForVehicle(uint32_t vehicleNodeId,
                                    uint16_t cellId,
                                    uint16_t rnti,
                                    uint32_t attempt = 0)
{
    if (GetNodeType(vehicleNodeId) != NODE_TYPE_CAR)
    {
        return;
    }

    auto gnbNodeIt = g_cellIdToGnbNodeId.find(cellId);
    auto gnbDevIndexIt = (gnbNodeIt != g_cellIdToGnbNodeId.end())
                             ? g_gnbDeviceIndexByNodeId.find(gnbNodeIt->second)
                             : g_gnbDeviceIndexByNodeId.end();
    auto ueDevIndexIt = g_vehicleDeviceIndexByNodeId.find(vehicleNodeId);
    if (gnbNodeIt == g_cellIdToGnbNodeId.end() || gnbDevIndexIt == g_gnbDeviceIndexByNodeId.end() ||
        ueDevIndexIt == g_vehicleDeviceIndexByNodeId.end())
    {
        return;
    }

    auto vehicleNodeIt = g_nodesById.find(vehicleNodeId);
    auto gnbNodeObjIt = g_nodesById.find(gnbNodeIt->second);
    if (vehicleNodeIt == g_nodesById.end() || gnbNodeObjIt == g_nodesById.end())
    {
        if (attempt < 20)
        {
            Simulator::Schedule(MilliSeconds(50),
                                &ConnectDownlinkTaskTracesForVehicle,
                                vehicleNodeId,
                                cellId,
                                rnti,
                                attempt + 1);
        }
        return;
    }

    Ptr<NrUeNetDevice> ueNetDev =
        DynamicCast<NrUeNetDevice>(vehicleNodeIt->second->GetDevice(ueDevIndexIt->second));
    Ptr<NrGnbNetDevice> gnbNetDev =
        DynamicCast<NrGnbNetDevice>(gnbNodeObjIt->second->GetDevice(gnbDevIndexIt->second));
    Ptr<NrUeRrc> ueRrc = ueNetDev ? ueNetDev->GetRrc() : nullptr;
    Ptr<NrGnbRrc> gnbRrc = gnbNetDev ? gnbNetDev->GetRrc() : nullptr;
    if (!ueRrc || !gnbRrc)
    {
        if (attempt < 20)
        {
            Simulator::Schedule(MilliSeconds(50),
                                &ConnectDownlinkTaskTracesForVehicle,
                                vehicleNodeId,
                                cellId,
                                rnti,
                                attempt + 1);
        }
        return;
    }

    ObjectMapValue gnbUeMap;
    gnbRrc->GetAttribute("UeMap", gnbUeMap);
    Ptr<Object> gnbUeManagerObj;
    for (auto it = gnbUeMap.Begin(); it != gnbUeMap.End(); ++it)
    {
        UintegerValue rntiValue;
        it->second->GetAttribute("C-RNTI", rntiValue);
        if (static_cast<uint16_t>(rntiValue.Get()) == rnti)
        {
            gnbUeManagerObj = it->second;
            break;
        }
    }

    ObjectMapValue ueBearerMap;
    ueRrc->GetAttribute("DataRadioBearerMap", ueBearerMap);
    ObjectMapValue gnbBearerMap;
    if (gnbUeManagerObj)
    {
        gnbUeManagerObj->GetAttribute("DataRadioBearerMap", gnbBearerMap);
    }

    if (!gnbUeManagerObj || ueBearerMap.GetN() == 0 || gnbBearerMap.GetN() == 0)
    {
        if (attempt < 20)
        {
            Simulator::Schedule(MilliSeconds(50),
                                &ConnectDownlinkTaskTracesForVehicle,
                                vehicleNodeId,
                                cellId,
                                rnti,
                                attempt + 1);
        }
        return;
    }

    std::unordered_map<uint8_t, Ptr<NrRlc>> gnbRlcsByLcid;
    for (auto it = gnbBearerMap.Begin(); it != gnbBearerMap.End(); ++it)
    {
        UintegerValue lcidValue;
        it->second->GetAttribute("logicalChannelIdentity", lcidValue);
        PointerValue rlcValue;
        it->second->GetAttribute("NrRlc", rlcValue);
        Ptr<NrRlc> rlc = rlcValue.Get<NrRlc>();
        if (rlc)
        {
            gnbRlcsByLcid[static_cast<uint8_t>(lcidValue.Get())] = rlc;
        }
    }

    bool attachedAny = false;
    for (auto it = ueBearerMap.Begin(); it != ueBearerMap.End(); ++it)
    {
        UintegerValue lcidValue;
        it->second->GetAttribute("logicalChannelIdentity", lcidValue);
        const uint8_t lcid = static_cast<uint8_t>(lcidValue.Get());

        auto gnbRlcIt = gnbRlcsByLcid.find(lcid);
        if (gnbRlcIt == gnbRlcsByLcid.end())
        {
            continue;
        }

        PointerValue ueRlcValue;
        it->second->GetAttribute("NrRlc", ueRlcValue);
        Ptr<NrRlc> ueRlc = ueRlcValue.Get<NrRlc>();
        if (!ueRlc)
        {
            continue;
        }

        const std::string attachmentKey = std::to_string(vehicleNodeId) + ":" +
                                          std::to_string(static_cast<uint32_t>(cellId)) + ":" +
                                          std::to_string(static_cast<uint32_t>(rnti)) + ":" +
                                          std::to_string(static_cast<uint32_t>(lcid));
        if (!g_downlinkTraceAttachmentKeys.insert(attachmentKey).second)
        {
            attachedAny = true;
            continue;
        }

        bool foundTx = gnbRlcIt->second->TraceConnectWithoutContext(
            "TxPDUWithPacket",
            MakeBoundCallback(&NotifyDlTaskRlcTx, gnbNodeIt->second, vehicleNodeId, cellId));
        bool foundTxDrop =
            gnbRlcIt->second->TraceConnectWithoutContext("TxDrop",
                                                         MakeBoundCallback(&NotifyDlTaskRlcTxDrop,
                                                                           gnbNodeIt->second,
                                                                           vehicleNodeId,
                                                                           cellId,
                                                                           rnti,
                                                                           lcid));
        bool foundRx = ueRlc->TraceConnectWithoutContext(
            "RxPDUWithPacket",
            MakeBoundCallback(&NotifyDlTaskRlcRx, gnbNodeIt->second, vehicleNodeId, cellId));

        if (foundTx && foundTxDrop && foundRx)
        {
            attachedAny = true;
            continue;
        }

        g_downlinkTraceAttachmentKeys.erase(attachmentKey);
    }

    if (!attachedAny && attempt < 20)
    {
        Simulator::Schedule(MilliSeconds(50),
                            &ConnectDownlinkTaskTracesForVehicle,
                            vehicleNodeId,
                            cellId,
                            rnti,
                            attempt + 1);
    }
}

static void
ConnectAllDownlinkTaskTraces()
{
    for (const auto& [vehicleNodeId, cellId] : g_vehicleCellByNodeId)
    {
        auto rntiIt = g_vehicleRntiByNodeId.find(vehicleNodeId);
        if (rntiIt == g_vehicleRntiByNodeId.end())
        {
            continue;
        }

        ConnectDownlinkTaskTracesForVehicle(vehicleNodeId, cellId, rntiIt->second);
    }
}

static void
RememberVehicleFogSelection(uint32_t vehicleId, NodeType fogType, double relVelKmh)
{
    g_vehicleServingFogType[vehicleId] = fogType;
    g_vehicleServingRelVelKmh[vehicleId] = relVelKmh;
}

static double
ComputeVehicleToCellRelativeVelocityKmh(uint32_t vehicleNodeId, uint16_t cellId)
{
    auto vehicleIt = g_nodesById.find(vehicleNodeId);
    auto gnbIt = g_cellIdToGnbNodeId.find(cellId);
    if (vehicleIt == g_nodesById.end() || gnbIt == g_cellIdToGnbNodeId.end())
    {
        return 0.0;
    }

    auto gnbNodeIt = g_nodesById.find(gnbIt->second);
    if (gnbNodeIt == g_nodesById.end())
    {
        return 0.0;
    }

    Ptr<MobilityModel> vehicleMob = vehicleIt->second->GetObject<MobilityModel>();
    Ptr<MobilityModel> gnbMob = gnbNodeIt->second->GetObject<MobilityModel>();
    return CalculateRelativeVelocity(vehicleMob, gnbMob) * 3.6;
}

static void
NotifyDlTaskRlcTx(uint32_t gnbNodeId,
                  uint32_t vehicleNodeId,
                  uint16_t cellId,
                  uint16_t rnti,
                  uint8_t lcid,
                  Ptr<const Packet> packet)
{
    uint32_t taskId = 0;
    if (!ExtractTaskResponseId(packet, &taskId))
    {
        return;
    }
    LogDownlinkRadioEvent("RLC_DL_TX",
                          taskId,
                          gnbNodeId,
                          vehicleNodeId,
                          cellId,
                          rnti,
                          lcid,
                          packet->GetSize(),
                          0.0);
}

static void
NotifyDlTaskRlcTxDrop(uint32_t gnbNodeId,
                      uint32_t vehicleNodeId,
                      uint16_t cellId,
                      uint16_t rnti,
                      uint8_t lcid,
                      Ptr<const Packet> packet)
{
    uint32_t taskId = 0;
    if (!ExtractTaskResponseId(packet, &taskId))
    {
        return;
    }
    LogDownlinkRadioEvent("RLC_DL_TX_DROP",
                          taskId,
                          gnbNodeId,
                          vehicleNodeId,
                          cellId,
                          rnti,
                          lcid,
                          packet->GetSize(),
                          0.0);
}

static void
NotifyDlTaskRlcRx(uint32_t gnbNodeId,
                  uint32_t vehicleNodeId,
                  uint16_t cellId,
                  uint16_t rnti,
                  uint8_t lcid,
                  Ptr<const Packet> packet,
                  uint64_t delayNs)
{
    uint32_t taskId = 0;
    if (!ExtractTaskResponseId(packet, &taskId))
    {
        return;
    }
    LogDownlinkRadioEvent("RLC_DL_RX",
                          taskId,
                          gnbNodeId,
                          vehicleNodeId,
                          cellId,
                          rnti,
                          lcid,
                          packet->GetSize(),
                          static_cast<double>(delayNs) / 1e9);
}

static void
NotifyUeHandoverStart(std::string context,
                      uint64_t imsi,
                      uint16_t cellId,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    (void)context;
    (void)cellId;
    (void)rnti;
    (void)targetCellId;

    auto nodeIt = g_imsiToNodeId.find(imsi);
    if (nodeIt == g_imsiToNodeId.end())
    {
        return;
    }

    if (GetNodeType(nodeIt->second) != NODE_TYPE_CAR)
    {
        return;
    }

    g_lastCellByImsi[imsi] = cellId;
    g_handoverStartByImsi[imsi] = Simulator::Now();
}

static void
NotifyUeHandoverEndOk(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    (void)context;
    (void)rnti;

    auto nodeIt = g_imsiToNodeId.find(imsi);
    if (nodeIt == g_imsiToNodeId.end())
    {
        return;
    }

    const uint32_t vehicleNodeId = nodeIt->second;
    if (GetNodeType(vehicleNodeId) != NODE_TYPE_CAR)
    {
        g_lastCellByImsi[imsi] = cellId;
        return;
    }

    UpdateVehicleBearerMapping(vehicleNodeId, cellId, rnti);
    ConnectDownlinkTaskTracesForVehicle(vehicleNodeId, cellId, rnti);

    const Time now = Simulator::Now();
    const uint16_t oldCellId = g_lastCellByImsi.count(imsi) ? g_lastCellByImsi[imsi] : UINT16_MAX;
    if (oldCellId == UINT16_MAX || oldCellId == cellId)
    {
        g_lastCellByImsi[imsi] = cellId;
        return;
    }

    double interruption = 0.0;
    auto hoStartIt = g_handoverStartByImsi.find(imsi);
    if (hoStartIt != g_handoverStartByImsi.end())
    {
        interruption = (now - hoStartIt->second).GetSeconds();
        g_handoverStartByImsi.erase(hoStartIt);
    }

    if (g_handoverLog.is_open())
    {
        g_handoverLog << now.GetSeconds() << ',' << vehicleNodeId << ',' << oldCellId << ','
                      << cellId << ',' << interruption << '\n';
    }

    NodeType servingType = NODE_TYPE_RSU;
    auto fogTypeIt = g_vehicleServingFogType.find(vehicleNodeId);
    if (fogTypeIt != g_vehicleServingFogType.end())
    {
        servingType = fogTypeIt->second;
    }

    double relVelKmh = ComputeVehicleToCellRelativeVelocityKmh(vehicleNodeId, cellId);
    if (relVelKmh <= 0.0)
    {
        auto relIt = g_vehicleServingRelVelKmh.find(vehicleNodeId);
        if (relIt != g_vehicleServingRelVelKmh.end())
        {
            relVelKmh = relIt->second;
        }
    }

    LogMethodologyRow(now.GetSeconds(), vehicleNodeId, servingType, relVelKmh, "Handover", 1, 0);

    g_lastCellByImsi[imsi] = cellId;
}

static void
NotifyUeConnectionReconfiguration(std::string context,
                                  uint64_t imsi,
                                  uint16_t cellId,
                                  uint16_t rnti)
{
    auto nodeIt = g_imsiToNodeId.find(imsi);
    if (nodeIt != g_imsiToNodeId.end() && GetNodeType(nodeIt->second) == NODE_TYPE_CAR)
    {
        UpdateVehicleBearerMapping(nodeIt->second, cellId, rnti);
        ConnectDownlinkTaskTracesForVehicle(nodeIt->second, cellId, rnti);
    }
    NotifyUeHandoverEndOk(context, imsi, cellId, rnti);
}

static double
EstimateSnrDb(double distanceMeters, double txPowerDbm, double bandwidthHz, double noiseFigureDb)
{
    // Friis pathloss (free-space) in dB
    const double c = 3e8;
    double lambda = c / 28e9; // fixed for this scenario
    double lossDb = 20.0 * std::log10(4 * kPi * std::max(distanceMeters, 1.0) / lambda);
    double rxDbm = txPowerDbm - lossDb;
    double noiseDbm = -174.0 + 10.0 * std::log10(bandwidthHz) + noiseFigureDb;
    return rxDbm - noiseDbm;
}

class TaskHeader : public Header
{
  public:
    TaskHeader() = default;

    static TypeId GetTypeId()
    {
        static TypeId tid =
            TypeId("ns3::TaskHeader").SetParent<Header>().AddConstructor<TaskHeader>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    void SetTaskId(uint32_t id)
    {
        m_taskId = id;
    }

    void SetSrcId(uint32_t id)
    {
        m_srcId = id;
    }

    void SetDstId(uint32_t id)
    {
        m_dstId = id;
    }

    void SetSendTime(Time t)
    {
        m_sendTimeNs = static_cast<uint64_t>(t.GetNanoSeconds());
    }

    void SetDeadline(Time t)
    {
        m_deadlineNs = static_cast<uint64_t>(t.GetNanoSeconds());
    }

    void SetMigrationDelay(Time t)
    {
        m_migrationDelayNs = static_cast<uint64_t>(t.GetNanoSeconds());
    }

    uint32_t GetTaskId() const
    {
        return m_taskId;
    }

    uint32_t GetSrcId() const
    {
        return m_srcId;
    }

    uint32_t GetDstId() const
    {
        return m_dstId;
    }

    Time GetSendTime() const
    {
        return NanoSeconds(m_sendTimeNs);
    }

    Time GetDeadline() const
    {
        return NanoSeconds(m_deadlineNs);
    }

    Time GetMigrationDelay() const
    {
        return NanoSeconds(m_migrationDelayNs);
    }

    uint32_t GetSerializedSize() const override
    {
        return 4 + 4 + 4 + 8 + 8 + 8;
    }

    void Serialize(Buffer::Iterator start) const override
    {
        start.WriteHtonU32(m_taskId);
        start.WriteHtonU32(m_srcId);
        start.WriteHtonU32(m_dstId);
        start.WriteHtonU64(m_sendTimeNs);
        start.WriteHtonU64(m_deadlineNs);
        start.WriteHtonU64(m_migrationDelayNs);
    }

    uint32_t Deserialize(Buffer::Iterator start) override
    {
        m_taskId = start.ReadNtohU32();
        m_srcId = start.ReadNtohU32();
        m_dstId = start.ReadNtohU32();
        m_sendTimeNs = start.ReadNtohU64();
        m_deadlineNs = start.ReadNtohU64();
        m_migrationDelayNs = start.ReadNtohU64();
        return GetSerializedSize();
    }

    void Print(std::ostream& os) const override
    {
        os << "taskId=" << m_taskId << " srcId=" << m_srcId << " dstId=" << m_dstId
           << " sendNs=" << m_sendTimeNs << " deadlineNs=" << m_deadlineNs
           << " migrationNs=" << m_migrationDelayNs;
    }

  private:
    uint32_t m_taskId{0};
    uint32_t m_srcId{0};
    uint32_t m_dstId{0};
    uint64_t m_sendTimeNs{0};
    uint64_t m_deadlineNs{0};
    uint64_t m_migrationDelayNs{0};
};

class TaskLogger : public Object
{
  public:
    static TypeId GetTypeId();

    void Open(const std::string& tasksPath)
    {
        m_tasks.open(tasksPath.c_str(), std::ios::out);
        m_tasks << "task_id,src_vehicle_id,fog_id,serving_node_type,send_time_s,recv_time_s,"
                   "deadline_s,queue_delay_s,service_time_s,completion_time_s,completion_delay_s,"
                   "success,packet_size_bytes,migration_delay_s\n";
        m_tasks << std::fixed << std::setprecision(6);
    }

    void LogTask(uint32_t taskId,
                 uint32_t srcId,
                 uint32_t fogId,
                 NodeType servingType,
                 Time sendTime,
                 Time recvTime,
                 Time deadline,
                 Time queueDelay,
                 Time serviceTime,
                 Time completionTime,
                 uint32_t packetSizeBytes,
                 Time migrationDelay,
                 bool success)
    {
        m_totalTasks++;
        if (success)
        {
            m_successTasks++;
        }
        m_sumQueueDelay += queueDelay.GetSeconds();
        m_sumMigrationDelay += migrationDelay.GetSeconds();
        m_sumCompletionDelay += (completionTime - sendTime).GetSeconds();

        m_tasks << taskId << ',' << srcId << ',' << fogId << ','
                << static_cast<uint32_t>(servingType) << ',' << sendTime.GetSeconds() << ','
                << recvTime.GetSeconds() << ',' << deadline.GetSeconds() << ','
                << queueDelay.GetSeconds() << ',' << serviceTime.GetSeconds() << ','
                << completionTime.GetSeconds() << ',' << (completionTime - sendTime).GetSeconds()
                << ',' << (success ? 1 : 0) << ',' << packetSizeBytes << ','
                << migrationDelay.GetSeconds() << '\n';
    }

    void WriteSummary(const std::string& summaryPath)
    {
        std::ofstream summary(summaryPath.c_str(), std::ios::out);
        summary << "total_tasks,success_tasks,success_rate,avg_queue_delay_s,avg_migration_delay_s,"
                   "avg_completion_delay_s\n";
        summary << std::fixed << std::setprecision(6);
        double successRate =
            m_totalTasks > 0 ? static_cast<double>(m_successTasks) / m_totalTasks : 0.0;
        double avgQueue = m_totalTasks > 0 ? m_sumQueueDelay / m_totalTasks : 0.0;
        double avgMigration = m_totalTasks > 0 ? m_sumMigrationDelay / m_totalTasks : 0.0;
        double avgCompletion = m_totalTasks > 0 ? m_sumCompletionDelay / m_totalTasks : 0.0;
        summary << m_totalTasks << ',' << m_successTasks << ',' << successRate << ',' << avgQueue
                << ',' << avgMigration << ',' << avgCompletion << '\n';
    }

    uint64_t GetTotalTasks() const
    {
        return m_totalTasks;
    }

    uint64_t GetSuccessTasks() const
    {
        return m_successTasks;
    }

    uint64_t GetDeadlineMissTasks() const
    {
        return m_totalTasks >= m_successTasks ? m_totalTasks - m_successTasks : 0;
    }

    void Close()
    {
        if (m_tasks.is_open())
        {
            m_tasks.close();
        }
    }

  private:
    std::ofstream m_tasks;
    uint64_t m_totalTasks{0};
    uint64_t m_successTasks{0};
    double m_sumQueueDelay{0.0};
    double m_sumMigrationDelay{0.0};
    double m_sumCompletionDelay{0.0};
};

NS_OBJECT_ENSURE_REGISTERED(TaskLogger);

TypeId
TaskLogger::GetTypeId()
{
    static TypeId tid = TypeId("TaskLogger").SetParent<Object>().AddConstructor<TaskLogger>();
    return tid;
}

class FogServerApp : public Application
{
  public:
    static TypeId GetTypeId();
    FogServerApp() = default;

    void Setup(uint16_t port, Time serviceTime, Ptr<TaskLogger> logger)
    {
        m_port = port;
        m_serviceTime = serviceTime;
        m_logger = logger;
    }

  private:
    void StartApplication() override
    {
        if (!m_socket)
        {
            m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            InetSocketAddress local(Ipv4Address::GetAny(), m_port);
            m_socket->Bind(local);
            m_socket->SetRecvCallback(MakeCallback(&FogServerApp::HandleRead, this));
        }
        m_nextAvailable = Simulator::Now();
    }

    void StopApplication() override
    {
        if (m_socket)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    void HandleRead(Ptr<Socket> socket)
    {
        Address from;
        while (auto packet = socket->RecvFrom(from))
        {
            TaskHeader header;
            if (packet->GetSize() < header.GetSerializedSize())
            {
                NS_LOG_WARN("Dropping task upload without TaskHeader on fog node "
                            << GetNode()->GetId() << " size " << packet->GetSize());
                continue;
            }
            packet->RemoveHeader(header);

            Time now = Simulator::Now();
            Time serviceStart = std::max(now, m_nextAvailable);
            Time queueDelay = serviceStart - now;
            Time completion = serviceStart + m_serviceTime;
            m_nextAvailable = completion;
            uint32_t queueSize = 0;
            if (m_serviceTime.GetSeconds() > 0)
            {
                double waiting = (serviceStart - now).GetSeconds();
                queueSize =
                    waiting > 0
                        ? static_cast<uint32_t>(std::ceil(waiting / m_serviceTime.GetSeconds()))
                        : 0;
            }
            UpdateFogQueueSize(GetNode()->GetId(), queueSize);

            Time sendTime = header.GetSendTime();
            Time deadline = header.GetDeadline();
            Time migrationDelay = header.GetMigrationDelay();

            bool success = (completion - sendTime) <= deadline;

            uint32_t totalSize = packet->GetSize() + header.GetSerializedSize();
            if (m_logger)
            {
                m_logger->LogTask(header.GetTaskId(),
                                  header.GetSrcId(),
                                  header.GetDstId(),
                                  GetNodeType(GetNode()->GetId()),
                                  sendTime,
                                  now,
                                  deadline,
                                  queueDelay,
                                  m_serviceTime,
                                  completion,
                                  totalSize,
                                  migrationDelay,
                                  success);
            }

            TaskHeader respHeader = header;
            respHeader.SetSrcId(GetNode()->GetId());
            respHeader.SetDstId(header.GetSrcId());
            respHeader.SetSendTime(completion);
            Ptr<Packet> resp = Create<Packet>(0);
            resp->AddHeader(respHeader);
            resp->AddPacketTag(TaskResponseTag(header.GetTaskId()));

            Ipv4Address observedSrc = Ipv4Address::GetZero();
            uint16_t observedPort = 0;
            if (InetSocketAddress::IsMatchingType(from))
            {
                InetSocketAddress remote = InetSocketAddress::ConvertFrom(from);
                observedSrc = remote.GetIpv4();
                observedPort = remote.GetPort();
            }

            Ipv4Address resolvedDst = LookupNodeAddress(header.GetSrcId());
            if (resolvedDst == Ipv4Address::GetZero())
            {
                resolvedDst = observedSrc;
            }
            if (resolvedDst == Ipv4Address::GetZero() || observedPort == 0)
            {
                NS_LOG_WARN("Unable to resolve response path for task "
                            << header.GetTaskId() << " on fog node " << GetNode()->GetId()
                            << " (src=" << header.GetSrcId() << ", observed=" << observedSrc << ':'
                            << observedPort << ')');
                continue;
            }

            Simulator::Schedule(completion - now,
                                &FogServerApp::SendResponse,
                                this,
                                resp,
                                InetSocketAddress(resolvedDst, observedPort),
                                observedSrc,
                                header.GetTaskId(),
                                header.GetSrcId());
        }
    }

    void SendResponse(Ptr<Packet> packet,
                      InetSocketAddress to,
                      Ipv4Address observedSrc,
                      uint32_t taskId,
                      uint32_t vehicleId)
    {
        if (!m_socket)
        {
            return;
        }
        const int sent = m_socket->SendTo(packet, 0, to);
        if (g_responsePathLog.is_open())
        {
            g_responsePathLog << Simulator::Now().GetSeconds() << ',' << taskId << ','
                              << GetNode()->GetId() << ',' << vehicleId << ',' << observedSrc << ','
                              << to.GetIpv4() << ',' << to.GetPort() << ',' << sent << '\n';
        }
    }

    Ptr<Socket> m_socket;
    uint16_t m_port{9000};
    Time m_serviceTime{MilliSeconds(5)};
    Time m_nextAvailable{Seconds(0)};
    Ptr<TaskLogger> m_logger;
};

NS_OBJECT_ENSURE_REGISTERED(FogServerApp);

TypeId
FogServerApp::GetTypeId()
{
    static TypeId tid =
        TypeId("FogServerApp").SetParent<Application>().AddConstructor<FogServerApp>();
    return tid;
}

class VehicleApp : public Application
{
  public:
    static TypeId GetTypeId();
    VehicleApp() = default;
    ~VehicleApp() override = default;

    struct TaskInfo
    {
        uint32_t taskId;
        uint32_t fogNodeId;
        NodeType fogType;
        double relVelKmh;
        bool received{false};
    };

    void Setup(std::vector<Ptr<Node>> fogNodes,
               std::vector<Ipv4Address> fogAddresses,
               uint16_t fogPort,
               double eventRate,
               uint32_t taskSizeBytes,
               Time deadline,
               Time assocInterval,
               uint32_t testMode,
               uint32_t queueLimit,
               double sinrThresholdDb,
               double busTxPowerDbm,
               double rsuTxPowerDbm)
    {
        m_fogNodes = std::move(fogNodes);
        m_fogAddresses = std::move(fogAddresses);
        m_fogPort = fogPort;
        m_eventRate = eventRate;
        m_taskSizeBytes = taskSizeBytes;
        m_deadline = deadline;
        m_assocInterval = assocInterval;
        m_testMode = testMode;
        m_queueLimit = queueLimit;
        m_noiseFigureDb = 7.0;
        m_sinrThresholdDb = sinrThresholdDb;
        m_bandwidthHz = g_bandwidthHz;
        m_busTxPowerDbm = busTxPowerDbm;
        m_rsuTxPowerDbm = rsuTxPowerDbm;
    }

  private:
    void StartApplication() override
    {
        if (!m_socket)
        {
            m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
            m_socket->Bind();
            m_socket->SetRecvCallback(MakeCallback(&VehicleApp::ReceiveTaskResponse, this));
        }

        m_eventRv = CreateObject<ExponentialRandomVariable>();
        if (m_eventRate > 0.0)
        {
            m_eventRv->SetAttribute("Mean", DoubleValue(1.0 / m_eventRate));
        }

        UpdateAssociation();
        ScheduleNextEvent();
    }

    void StopApplication() override
    {
        // Log drops for outstanding tasks
        for (auto& [id, info] : m_tasks)
        {
            if (!info.received)
            {
                LogMethodologyRow(Simulator::Now().GetSeconds(),
                                  GetNode()->GetId(),
                                  info.fogType,
                                  info.relVelKmh,
                                  "Dropped",
                                  0,
                                  0);
            }
        }
        if (m_sendEvent.IsPending())
        {
            Simulator::Cancel(m_sendEvent);
        }
        if (m_assocEvent.IsPending())
        {
            Simulator::Cancel(m_assocEvent);
        }
        if (m_socket)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    void ScheduleNextEvent()
    {
        if (m_eventRate <= 0.0)
        {
            return;
        }
        Time next = Seconds(m_eventRv->GetValue());
        m_sendEvent = Simulator::Schedule(next, &VehicleApp::SendTask, this);
    }

    void ScheduleAssocUpdate()
    {
        if (m_assocInterval.IsZero())
        {
            return;
        }
        m_assocEvent = Simulator::Schedule(m_assocInterval, &VehicleApp::UpdateAssociation, this);
    }

    void UpdateAssociation()
    {
        if (m_fogNodes.empty())
        {
            return;
        }

        Ptr<MobilityModel> selfMob = GetNode()->GetObject<MobilityModel>();
        if (!selfMob)
        {
            return;
        }

        std::optional<uint32_t> bestIndex = GetBestFogIndex(selfMob);
        if (!bestIndex.has_value())
        {
            return;
        }

        Ptr<Node> selectedFog = m_fogNodes[bestIndex.value()];
        NodeType selectedFogType = GetNodeType(selectedFog->GetId());
        Ptr<MobilityModel> selectedFogMob = selectedFog->GetObject<MobilityModel>();
        double relVelKmh = CalculateRelativeVelocity(selfMob, selectedFogMob) * 3.6;
        const bool firstAssociation = !m_hasAssociation;
        const bool migration = m_hasAssociation && bestIndex.value() != m_currentFogIndex;
        bool fogTypeSwitched = false;
        if (!firstAssociation)
        {
            NodeType previousFogType = GetNodeType(m_fogNodes[m_currentFogIndex]->GetId());
            fogTypeSwitched = previousFogType != selectedFogType;
        }

        double selectedDistanceM = 0.0;
        double selectedSnrDb = 0.0;
        if (selectedFogMob)
        {
            selectedDistanceM = selfMob->GetDistanceFrom(selectedFogMob);
            double txPowerDbm =
                (selectedFogType == NODE_TYPE_BUS) ? m_busTxPowerDbm : m_rsuTxPowerDbm;
            selectedSnrDb =
                EstimateSnrDb(selectedDistanceM, txPowerDbm, m_bandwidthHz, m_noiseFigureDb);
        }
        uint32_t selectedQueueSize = 0;
        auto queueIt = g_fogQueueSizes.find(selectedFog->GetId());
        if (queueIt != g_fogQueueSizes.end())
        {
            selectedQueueSize = queueIt->second;
        }
        if (g_assocDiagnosticsLog.is_open())
        {
            g_assocDiagnosticsLog << "association_event," << Simulator::Now().GetSeconds() << ','
                                  << GetNode()->GetId() << ",,," << selectedFog->GetId() << ','
                                  << static_cast<uint32_t>(selectedFogType) << ','
                                  << selectedDistanceM << ',' << selectedSnrDb << ','
                                  << selectedQueueSize << ',' << (firstAssociation ? 1 : 0) << ','
                                  << (migration ? 1 : 0) << '\n';
        }

        if (firstAssociation)
        {
            // First association is not a migration event.
            m_currentFogIndex = bestIndex.value();
            m_lastAssocChange = Simulator::Now();
            m_hasAssociation = true;
            g_vehicleFirstAssocTime.emplace(GetNode()->GetId(), Simulator::Now());
        }
        else if (migration)
        {
            m_currentFogIndex = bestIndex.value();
            m_lastAssocChange = Simulator::Now();
            m_pendingMigration = true;
        }

        RememberVehicleFogSelection(GetNode()->GetId(), selectedFogType, relVelKmh);
        if (fogTypeSwitched)
        {
            LogMethodologyRow(Simulator::Now().GetSeconds(),
                              GetNode()->GetId(),
                              selectedFogType,
                              relVelKmh,
                              "FogReassociation",
                              0,
                              1);
        }

        ScheduleAssocUpdate();
    }

    void SendTask()
    {
        // Every Poisson tick counts as an offered task whether or not
        // the vehicle is currently associated. This is the denominator
        // used for offered-load reliability.
        ++g_counters.offered_tasks;

        if (!m_hasAssociation || m_fogAddresses.empty())
        {
            ++g_counters.no_association_drops;
            ScheduleNextEvent();
            return;
        }
        ++g_counters.sent_tasks;

        TaskHeader header;
        header.SetTaskId(m_taskId++);
        header.SetSrcId(GetNode()->GetId());
        Ptr<Node> dstNode = m_fogNodes[m_currentFogIndex];
        header.SetDstId(dstNode->GetId());
        header.SetSendTime(Simulator::Now());
        header.SetDeadline(m_deadline);

        Time migrationDelay = Seconds(0);
        if (m_pendingMigration)
        {
            migrationDelay = Simulator::Now() - m_lastAssocChange;
            m_pendingMigration = false;
        }
        header.SetMigrationDelay(migrationDelay);

        uint32_t headerSize = header.GetSerializedSize();
        uint32_t payloadSize = 0;
        if (m_taskSizeBytes > headerSize)
        {
            payloadSize = m_taskSizeBytes - headerSize;
        }
        Ptr<Packet> packet = Create<Packet>(payloadSize);
        packet->AddHeader(header);

        InetSocketAddress dst(m_fogAddresses[m_currentFogIndex], m_fogPort);
        m_socket->SendTo(packet, 0, dst);

        Ptr<MobilityModel> selfMob = GetNode()->GetObject<MobilityModel>();
        Ptr<MobilityModel> fogMob = dstNode->GetObject<MobilityModel>();
        double relVel = CalculateRelativeVelocity(selfMob, fogMob) * 3.6; // km/h
        NodeType fogType = GetNodeType(dstNode->GetId());
        m_tasks[header.GetTaskId()] = {header.GetTaskId(), dstNode->GetId(), fogType, relVel};
        LogMethodologyRow(Simulator::Now().GetSeconds(),
                          GetNode()->GetId(),
                          fogType,
                          relVel,
                          "Tx",
                          0,
                          0);

        ScheduleNextEvent();
    }

    std::optional<uint32_t> GetBestFogIndex(Ptr<MobilityModel> selfMob)
    {
        if (!selfMob)
        {
            return std::nullopt;
        }

        std::optional<uint32_t> bestGnb;
        double bestGnbSnr = -1e9;
        std::optional<uint32_t> bestVfn;
        double bestVfnScoreDb = -1e9;

        for (uint32_t i = 0; i < m_fogNodes.size(); ++i)
        {
            Ptr<Node> fogNode = m_fogNodes[i];
            Ptr<MobilityModel> fogMob = fogNode->GetObject<MobilityModel>();
            if (!fogMob)
            {
                continue;
            }
            NodeType type = GetNodeType(fogNode->GetId());
            auto queueIt = g_fogQueueSizes.find(fogNode->GetId());
            if (queueIt != g_fogQueueSizes.end() && queueIt->second >= m_queueLimit)
            {
                continue;
            }
            double dist = selfMob->GetDistanceFrom(fogMob);
            // Score each fog at the TX power its PHY actually uses
            // (both 23 dBm under the equalised-power setup).
            double txPowerDbm =
                (type == NODE_TYPE_BUS) ? m_busTxPowerDbm : m_rsuTxPowerDbm;
            double snrDb = EstimateSnrDb(dist, txPowerDbm, m_bandwidthHz, m_noiseFigureDb);

            if (snrDb < m_sinrThresholdDb)
            {
                continue; // below SINR gate
            }

            if (type == NODE_TYPE_BUS)
            {
                // CRE bias applies only to the association score, not
                // to the physical link budget.
                double associationScoreDb = snrDb;
                if (m_testMode == 1)
                {
                    associationScoreDb += kVfnCreBiasDb;
                }
                if (associationScoreDb > bestVfnScoreDb)
                {
                    bestVfnScoreDb = associationScoreDb;
                    bestVfn = i;
                }
            }
            else // RSU/CFN infrastructure
            {
                if (snrDb > bestGnbSnr)
                {
                    bestGnbSnr = snrDb;
                    bestGnb = i;
                }
            }
        }

        if (bestVfn.has_value() && (!bestGnb.has_value() || bestVfnScoreDb > bestGnbSnr))
        {
            return bestVfn;
        }
        return bestGnb;
    }

    void ReceiveTaskResponse(Ptr<Socket> socket)
    {
        Address from;
        while (auto packet = socket->RecvFrom(from))
        {
            TaskHeader header;
            if (packet->GetSize() < header.GetSerializedSize())
            {
                NS_LOG_WARN("Dropping task response without TaskHeader on vehicle "
                            << GetNode()->GetId() << " size " << packet->GetSize());
                continue;
            }
            packet->RemoveHeader(header);
            uint32_t senderId = header.GetSrcId();
            Time now = Simulator::Now();

            auto it = m_tasks.find(header.GetTaskId());
            NodeType senderType = GetNodeType(senderId);
            double relVel = 0.0;
            if (it != m_tasks.end())
            {
                it->second.received = true;
                relVel = it->second.relVelKmh;
            }
            else
            {
                Ptr<MobilityModel> selfMob = GetNode()->GetObject<MobilityModel>();
                Ptr<MobilityModel> fogMob = nullptr;
                auto fogIt = g_nodesById.find(senderId);
                if (fogIt != g_nodesById.end())
                {
                    fogMob = fogIt->second->GetObject<MobilityModel>();
                }
                relVel = CalculateRelativeVelocity(selfMob, fogMob) * 3.6;
            }
            LogMethodologyRow(now.GetSeconds(), GetNode()->GetId(), senderType, relVel, "Rx", 0, 0);
        }
    }

    Ptr<Socket> m_socket;
    std::vector<Ptr<Node>> m_fogNodes;
    std::vector<Ipv4Address> m_fogAddresses;
    Ptr<ExponentialRandomVariable> m_eventRv;
    EventId m_sendEvent;
    EventId m_assocEvent;

    uint16_t m_fogPort{9000};
    double m_eventRate{0.1};
    uint32_t m_taskSizeBytes{600};
    Time m_deadline{MilliSeconds(100)};
    Time m_assocInterval{MilliSeconds(500)};
    uint32_t m_testMode{0};
    uint32_t m_queueLimit{50};

    uint32_t m_taskId{0};
    uint32_t m_currentFogIndex{0};
    bool m_hasAssociation{false};
    bool m_pendingMigration{false};
    Time m_lastAssocChange{Seconds(0)};
    std::unordered_map<uint32_t, TaskInfo> m_tasks;
    double m_noiseFigureDb{7.0};
    double m_sinrThresholdDb{0.0};
    double m_bandwidthHz{100e6};
    double m_busTxPowerDbm{23.0};
    double m_rsuTxPowerDbm{23.0};
};

NS_OBJECT_ENSURE_REGISTERED(VehicleApp);

TypeId
VehicleApp::GetTypeId()
{
    static TypeId tid = TypeId("VehicleApp").SetParent<Application>().AddConstructor<VehicleApp>();
    return tid;
}

class BroadcastSenderApp : public Application
{
  public:
    static TypeId GetTypeId();

    BroadcastSenderApp() = default;

    void Setup(std::vector<Ipv4Address> vehicleAddresses,
               uint16_t port,
               Time interval,
               uint32_t packetSize)
    {
        m_vehicleAddresses = std::move(vehicleAddresses);
        m_port = port;
        m_interval = interval;
        m_packetSize = packetSize;
    }

  private:
    void StartApplication() override
    {
        if (!m_socket)
        {
            m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        }
        ScheduleNext();
    }

    void StopApplication() override
    {
        if (m_sendEvent.IsPending())
        {
            Simulator::Cancel(m_sendEvent);
        }
        if (m_socket)
        {
            m_socket->Close();
            m_socket = nullptr;
        }
    }

    void ScheduleNext()
    {
        if (m_interval.IsZero())
        {
            return;
        }
        m_sendEvent = Simulator::Schedule(m_interval, &BroadcastSenderApp::SendBurst, this);
    }

    void SendBurst()
    {
        for (const auto& addr : m_vehicleAddresses)
        {
            Ptr<Packet> packet = Create<Packet>(m_packetSize);
            InetSocketAddress dst(addr, m_port);
            m_socket->SendTo(packet, 0, dst);
        }
        ScheduleNext();
    }

    Ptr<Socket> m_socket;
    EventId m_sendEvent;
    std::vector<Ipv4Address> m_vehicleAddresses;
    uint16_t m_port{9100};
    Time m_interval{MilliSeconds(100)};
    uint32_t m_packetSize{200};
};

NS_OBJECT_ENSURE_REGISTERED(BroadcastSenderApp);

TypeId
BroadcastSenderApp::GetTypeId()
{
    static TypeId tid =
        TypeId("BroadcastSenderApp").SetParent<Application>().AddConstructor<BroadcastSenderApp>();
    return tid;
}

/**
 * Parse the NS-2 trace file to extract initial "$node_(i) set X_/Y_" positions.
 * This is called BEFORE the simulation runs, so we cannot rely on MobilityModel
 * which only gets populated when the event loop processes the trace commands.
 */
static std::unordered_map<uint32_t, Vector>
ParseTracePositions(const std::string& traceFile)
{
    std::unordered_map<uint32_t, Vector> positions;
    std::unordered_set<uint32_t> hasX;
    std::unordered_set<uint32_t> hasY;

    std::ifstream in(traceFile);
    if (!in.is_open())
    {
        NS_LOG_WARN("ParseTracePositions: cannot open " << traceFile);
        return {};
    }

    std::string line;
    while (std::getline(in, line))
    {
        // Match: $node_(N) set X_ VALUE  or  $node_(N) set Y_ VALUE
        if (line.find("$node_(") == std::string::npos || line.find(" set ") == std::string::npos)
        {
            continue;
        }

        auto parenStart = line.find('(');
        auto parenEnd = line.find(')');
        if (parenStart == std::string::npos || parenEnd == std::string::npos)
        {
            continue;
        }

        uint32_t nodeIndex = 0;
        try
        {
            nodeIndex = static_cast<uint32_t>(
                std::stoul(line.substr(parenStart + 1, parenEnd - parenStart - 1)));
        }
        catch (...)
        {
            continue;
        }

        auto setPos = line.find(" set ");
        std::string rest = line.substr(setPos + 5);

        double value = 0.0;
        if (rest.substr(0, 3) == "X_ ")
        {
            try
            {
                value = std::stod(rest.substr(3));
            }
            catch (...)
            {
                continue;
            }
            positions[nodeIndex].x = value;
            hasX.insert(nodeIndex);
        }
        else if (rest.substr(0, 3) == "Y_ ")
        {
            try
            {
                value = std::stod(rest.substr(3));
            }
            catch (...)
            {
                continue;
            }
            positions[nodeIndex].y = value;
            hasY.insert(nodeIndex);
        }
    }

    std::unordered_map<uint32_t, Vector> result;
    for (const auto& [nodeIndex, position] : positions)
    {
        if (hasX.find(nodeIndex) != hasX.end() && hasY.find(nodeIndex) != hasY.end())
        {
            result[nodeIndex] = position;
        }
    }

    NS_LOG_UNCOND("ParseTracePositions: extracted " << result.size() << " initial positions from "
                                                    << traceFile);
    return result;
}

int
main(int argc, char* argv[])
{
    std::string outputDir = "./";
    std::string simTag = "v2x";
    std::string animFile = "";
    std::string traceFile = "scratch/manchester_city_ns3.tcl";

    // RNG seeding. SetSeed picks the global seed; SetRun picks the
    // substream. For CI sweeps, hold rngSeed and vary rngRun.
    uint32_t rngSeed = 1;
    uint32_t rngRun = 1;

    uint32_t numVehicles = 100;
    uint32_t numRsus = 2;
    uint32_t numBuses = 1;
    uint32_t numGnbs = 3;

    double gnbDistance = 300.0;
    double gnbHeight = 10.0;
    double fogOffsetY = 50.0;
    double rsuHeight = 3.0;
    double busHeight = 2.0;
    double fogSpacing = 200.0;
    double rsuTxPowerDbm = 23.0;
    double busTxPowerDbm = 23.0;
    double sinrThresholdDb = 0.0;

    double roadLength = 1200.0;
    double vehicleSpacing = 20.0;
    double vehicleSpeed = 15.0; // m/s
    double busSpeed = 15.0;     // m/s (override via CLI)

    double centralFrequency = 28e9;
    double bandwidth = 100e6;
    double totalTxPowerDbm = 40.0;
    uint16_t numerology = 2; // mu=2 (60 kHz) for stability with scheduler

    double eventRate = 2; // events per second per vehicle
    uint32_t taskSizeBytes = 800;
    double taskDeadlineMs = 100.0;
    double serviceTimeMs = 5.0;
    double assocIntervalMs = 500.0;
    uint32_t queueLimit = 50;

    uint16_t fogPort = 9000;
    uint16_t downlinkPort = 9100;
    double downlinkIntervalMs = 0.0; // disabled: broadcast not needed for reliability analysis
    uint32_t downlinkPacketSize = 200;

    Time simTime = Seconds(100.0);

    uint32_t testMode = 0; // 0 baseline, 1 hybrid

    CommandLine cmd(__FILE__);
    cmd.AddValue("rngSeed", "RngSeedManager::SetSeed value", rngSeed);
    cmd.AddValue("rngRun", "RngSeedManager::SetRun value (sweep for CIs)", rngRun);
    cmd.AddValue("outputDir", "Output directory for CSV logs", outputDir);
    cmd.AddValue("simTag", "Tag for output files", simTag);
    cmd.AddValue("animFile", "NetAnim output XML file (empty to disable)", animFile);
    cmd.AddValue("traceFile", "NS-2 mobility trace file path", traceFile);
    cmd.AddValue("numVehicles", "Number of vehicle UEs", numVehicles);
    cmd.AddValue("numRsus", "Number of roadside unit fog nodes", numRsus);
    cmd.AddValue("numBuses", "Number of bus (VFN) fog nodes", numBuses);
    cmd.AddValue("numGnbs", "Number of gNBs", numGnbs);
    cmd.AddValue("gnbDistance", "Inter-site distance for gNBs (meters)", gnbDistance);
    cmd.AddValue("gnbHeight", "gNB antenna height (meters)", gnbHeight);
    cmd.AddValue("fogOffsetY", "Fog node offset on Y axis (meters)", fogOffsetY);
    cmd.AddValue("rsuHeight", "RSU antenna height (meters)", rsuHeight);
    cmd.AddValue("busHeight", "Bus antenna height (meters)", busHeight);
    cmd.AddValue("fogSpacing", "Fog node spacing along X axis (meters)", fogSpacing);
    cmd.AddValue("sinrThresholdDb", "Minimum SNR gate for fog association (dB)", sinrThresholdDb);
    cmd.AddValue("roadLength", "Length of straight road (meters)", roadLength);
    cmd.AddValue("vehicleSpacing", "Spacing between vehicles (meters)", vehicleSpacing);
    cmd.AddValue("vehicleSpeed", "Vehicle speed along +X (m/s)", vehicleSpeed);
    cmd.AddValue("busSpeed", "Bus speed along +X (m/s)", busSpeed);
    cmd.AddValue("centralFrequency", "Carrier frequency (Hz)", centralFrequency);
    cmd.AddValue("bandwidth", "System bandwidth (Hz)", bandwidth);
    cmd.AddValue("totalTxPowerDbm", "gNB total TX power (dBm)", totalTxPowerDbm);
    cmd.AddValue("numerology", "NR numerology", numerology);
    cmd.AddValue("eventRate", "Event rate (events/sec per vehicle)", eventRate);
    cmd.AddValue("taskSizeBytes", "Task packet size in bytes", taskSizeBytes);
    cmd.AddValue("taskDeadlineMs", "Task deadline in milliseconds", taskDeadlineMs);
    cmd.AddValue("serviceTimeMs", "Fog service time in milliseconds", serviceTimeMs);
    cmd.AddValue("assocIntervalMs", "Fog association update interval (ms)", assocIntervalMs);
    cmd.AddValue("queueLimit", "Maximum fog queue depth considered attachable", queueLimit);
    cmd.AddValue("fogPort", "UDP port used for uploads", fogPort);
    cmd.AddValue("downlinkPort", "UDP port used for downlink broadcast", downlinkPort);
    cmd.AddValue("downlinkIntervalMs", "Downlink broadcast interval (ms)", downlinkIntervalMs);
    cmd.AddValue("downlinkPacketSize", "Downlink packet size (bytes)", downlinkPacketSize);
    cmd.AddValue("simTime", "Simulation time", simTime);
    cmd.AddValue("testMode",
                 "Compatibility flag (CRE bias is disabled in this build)",
                 testMode);

    cmd.Parse(argc, argv);
    g_bandwidthHz = bandwidth;

    // Seed before any RandomVariableStream is created. NrHelper,
    // Ns2MobilityHelper, the VehicleApp event RV, and the bus-follow
    // offset all draw from the global RNG.
    RngSeedManager::SetSeed(rngSeed);
    RngSeedManager::SetRun(rngRun);
    NS_LOG_UNCOND("RNG: seed=" << rngSeed << " run=" << rngRun);

    Time taskDeadline = MilliSeconds(taskDeadlineMs);
    Time serviceTime = MilliSeconds(serviceTimeMs);
    Time assocInterval = MilliSeconds(assocIntervalMs);
    Time downlinkInterval = MilliSeconds(downlinkIntervalMs);

    NodeContainer gnbNodes;
    gnbNodes.Create(numGnbs);

    NodeContainer vehicleNodes;
    vehicleNodes.Create(numVehicles);

    NodeContainer rsuNodes;
    rsuNodes.Create(numRsus);

    NodeContainer busNodes;
    busNodes.Create(numBuses);

    NodeContainer fogNodes;
    fogNodes.Add(rsuNodes);
    fogNodes.Add(busNodes);

    NodeContainer mobileNodes;
    mobileNodes.Add(vehicleNodes);
    mobileNodes.Add(fogNodes);

    // Tag nodes by type
    for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i)
    {
        RegisterNodeType(vehicleNodes.Get(i), NODE_TYPE_CAR);
    }
    for (uint32_t i = 0; i < rsuNodes.GetN(); ++i)
    {
        RegisterNodeType(rsuNodes.Get(i), NODE_TYPE_RSU);
    }
    for (uint32_t i = 0; i < busNodes.GetN(); ++i)
    {
        RegisterNodeType(busNodes.Get(i), NODE_TYPE_BUS);
    }

    // Vehicle mobility: ns-2 trace when available, else simple straight road.
    const bool useTraceDrivenPlacement = !traceFile.empty();
    if (useTraceDrivenPlacement)
    {
        Ns2MobilityHelper ns2(traceFile);
        ns2.Install(vehicleNodes.Begin(), vehicleNodes.End());
    }
    else
    {
        MobilityHelper vehicleMobility;
        vehicleMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        Ptr<ListPositionAllocator> vehPos = CreateObject<ListPositionAllocator>();
        double x = 0.0;
        for (uint32_t i = 0; i < numVehicles; ++i)
        {
            vehPos->Add(Vector(x, 0.0, 0.0));
            x += vehicleSpacing;
            if (x > roadLength)
            {
                x = 0.0;
            }
        }
        vehicleMobility.SetPositionAllocator(vehPos);
        vehicleMobility.Install(vehicleNodes);
        for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i)
        {
            Ptr<ConstantVelocityMobilityModel> cv =
                vehicleNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
            cv->SetVelocity(Vector(vehicleSpeed, 0.0, 0.0));
        }
    }

    // Read initial positions straight from the trace file. The
    // MobilityModel does not have valid coordinates before
    // Simulator::Run() because Ns2MobilityHelper schedules position
    // updates as events.
    std::vector<Vector> vehicleInitialPositions;
    if (useTraceDrivenPlacement)
    {
        auto tracePositionsById = ParseTracePositions(traceFile);

        // Hard-fail rather than padding with stationary phantom UEs
        // when the trace does not contain enough distinct vehicle IDs.
        NS_ABORT_MSG_IF(tracePositionsById.size() < numVehicles,
                        "Trace defines only " << tracePositionsById.size()
                        << " distinct vehicles but numVehicles=" << numVehicles
                        << ". Regenerate the SUMO trace with at least "
                        << numVehicles << " IDs alive for the simulation duration, "
                        << "or reduce --numVehicles.");
        vehicleInitialPositions.reserve(numVehicles);
        for (uint32_t i = 0; i < numVehicles; ++i)
        {
            auto posIt = tracePositionsById.find(i);
            NS_ABORT_MSG_IF(posIt == tracePositionsById.end(),
                            "Trace is missing requested vehicle ID " << i
                            << ". Regenerate the trace with contiguous IDs 0.."
                            << (numVehicles - 1) << " or change the subsetting rule.");
            vehicleInitialPositions.push_back(posIt->second);
        }
    }
    if (vehicleInitialPositions.empty())
    {
        // Fallback for the non-trace path.
        vehicleInitialPositions.reserve(vehicleNodes.GetN());
        for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i)
        {
            Ptr<MobilityModel> vm = vehicleNodes.Get(i)->GetObject<MobilityModel>();
            if (vm)
            {
                vehicleInitialPositions.push_back(vm->GetPosition());
            }
        }
    }

    // Place gNBs near the active vehicle area when a trace is in use.
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> gnbPositions = CreateObject<ListPositionAllocator>();
    if (useTraceDrivenPlacement && !vehicleInitialPositions.empty())
    {
        auto gnbAnchors = SelectSpreadAnchors(vehicleInitialPositions, numGnbs, 0u);
        for (const auto& anchor : gnbAnchors)
        {
            gnbPositions->Add(Vector(anchor.x, anchor.y, gnbHeight));
        }
    }
    else
    {
        for (uint32_t i = 0; i < numGnbs; ++i)
        {
            gnbPositions->Add(Vector(i * gnbDistance, 0.0, gnbHeight));
        }
    }
    gnbMobility.SetPositionAllocator(gnbPositions);
    gnbMobility.Install(gnbNodes);
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        RegisterKnownNode(gnbNodes.Get(i));
    }

    // RSUs are stationary but placed close to the road activity.
    std::vector<Vector> rsuAnchors;
    if (useTraceDrivenPlacement && !vehicleInitialPositions.empty())
    {
        rsuAnchors = SelectSpreadAnchors(vehicleInitialPositions,
                                         numRsus,
                                         static_cast<uint32_t>(vehicleInitialPositions.size() / 3));
    }
    for (uint32_t i = 0; i < rsuNodes.GetN(); ++i)
    {
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
        if (!rsuAnchors.empty())
        {
            // Tiny deterministic offset so RSU and gNB markers do not overlap.
            const double angle = (2.0 * kPi * i) / std::max<uint32_t>(numRsus, 1u);
            const double radial = 8.0;
            const Vector base = rsuAnchors[i];
            mob->SetPosition(Vector(base.x + radial * std::cos(angle),
                                    base.y + radial * std::sin(angle),
                                    rsuHeight));
        }
        else
        {
            mob->SetPosition(Vector(i * fogSpacing, fogOffsetY, rsuHeight));
        }
        rsuNodes.Get(i)->AggregateObject(mob);
    }

    // VFN buses follow real vehicle trajectories.
    MobilityHelper busMobility;
    busMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    Ptr<ListPositionAllocator> busPos = CreateObject<ListPositionAllocator>();
    double bx = 0.0;
    for (uint32_t i = 0; i < busNodes.GetN(); ++i)
    {
        if (useTraceDrivenPlacement && vehicleNodes.GetN() > 0)
        {
            uint32_t refIndex = (i * vehicleNodes.GetN()) / std::max<uint32_t>(busNodes.GetN(), 1u);
            refIndex = std::min(refIndex, vehicleNodes.GetN() - 1);
            Ptr<MobilityModel> refMob = vehicleNodes.Get(refIndex)->GetObject<MobilityModel>();
            Vector refPos = refMob ? refMob->GetPosition() : Vector(0.0, 0.0, 0.0);
            Vector refVel = refMob ? refMob->GetVelocity() : Vector(0.0, 0.0, 0.0);
            // Lateral offset cycles 6 / 8 / 10 / 12 m to span the
            // 6-12 m range stated in the methodology.
            double lateralOffset = 6.0 + 2.0 * (i % 4);
            busPos->Add(OffsetAlongVelocityNormal(refPos, refVel, lateralOffset, busHeight));
        }
        else
        {
            busPos->Add(Vector(bx, fogOffsetY / 2.0, busHeight));
            bx += fogSpacing;
        }
    }
    busMobility.SetPositionAllocator(busPos);
    busMobility.Install(busNodes);
    for (uint32_t i = 0; i < busNodes.GetN(); ++i)
    {
        Ptr<ConstantVelocityMobilityModel> cv =
            busNodes.Get(i)->GetObject<ConstantVelocityMobilityModel>();
        if (useTraceDrivenPlacement && vehicleNodes.GetN() > 0)
        {
            uint32_t refIndex = (i * vehicleNodes.GetN()) / std::max<uint32_t>(busNodes.GetN(), 1u);
            refIndex = std::min(refIndex, vehicleNodes.GetN() - 1);
            double lateralOffset = 6.0 + 2.0 * (i % 4);
            Simulator::ScheduleNow(&UpdateBusFollowerMobility,
                                   busNodes.Get(i),
                                   vehicleNodes.Get(refIndex),
                                   busHeight,
                                   lateralOffset,
                                   MilliSeconds(20));
        }
        else
        {
            cv->SetVelocity(Vector(busSpeed, 0.0, 0.0));
        }
    }

    // Make sure every mobile node has a MobilityModel before NR devices go on.
    auto ensureMobility = [](Ptr<Node> n) {
        if (!n->GetObject<MobilityModel>())
        {
            Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
            mob->SetPosition(Vector(0.0, 0.0, 0.0));
            n->AggregateObject(mob);
        }
    };
    for (uint32_t i = 0; i < mobileNodes.GetN(); ++i)
    {
        ensureMobility(mobileNodes.Get(i));
    }

    // 5G NR setup
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(beamformingHelper);
    // Use a concrete scheduler type that derives from NrMacSchedulerNs3
    nrHelper->SetSchedulerTypeId(TypeId::LookupByName("ns3::NrMacSchedulerTdmaRR"));
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(true));
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(2));

    CcBwpCreator ccBwpCreator;
    const uint8_t numCcs = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCcs);
    bandConf.m_numBwp = 1;

    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled",
                                        BooleanValue(false)); // flat-earth, no blockage/shadowing
    channelHelper->AssignChannelsToBands({band}, NrChannelHelper::INIT_PROPAGATION);

    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(mobileNodes, allBwps);

    NS_ABORT_MSG_IF(ueDevs.GetN() != mobileNodes.GetN(),
                    "UE device count does not match mobile node count");

    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbPhy> gnbPhy = NrHelper::GetGnbPhy(gnbDevs.Get(i), 0);
        Ptr<NrGnbMac> gnbMac = NrHelper::GetGnbMac(gnbDevs.Get(i), 0);
        NS_ABORT_MSG_IF(!gnbPhy, "Missing gNB PHY at index " << i);
        NS_ABORT_MSG_IF(!gnbMac, "Missing gNB MAC at index " << i);
        gnbPhy->SetAttribute("Numerology", UintegerValue(numerology));
        gnbPhy->SetAttribute("TxPower", DoubleValue(totalTxPowerDbm));
        // Trim DL control symbols to avoid HARQ overrun.
        uint8_t dlCtrl = gnbMac->GetDlCtrlSyms();
        if (dlCtrl > 1)
        {
            gnbMac->SetAttribute("DlCtrlSyms", UintegerValue(1));
        }
    }

    // Enable X2 and handover
    nrHelper->AddX2Interface(gnbNodes);
    nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(1.5));
    nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(128)));

    // Internet stack on mobile nodes
    InternetStackHelper internet;
    internet.Install(mobileNodes);

    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueDevs);
    Ipv4StaticRoutingHelper routingHelper;
    for (uint32_t i = 0; i < mobileNodes.GetN(); ++i)
    {
        RegisterNodeAddress(mobileNodes.Get(i), ueIpIface.GetAddress(i));
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            routingHelper.GetStaticRouting(mobileNodes.Get(i)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Check mobility and PHY before the attach loop.
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<Node> ueNode = mobileNodes.Get(i);
        Ptr<MobilityModel> mm = ueNode->GetObject<MobilityModel>();
        NS_ABORT_MSG_IF(!mm, "Missing MobilityModel on mobile node " << ueNode->GetId());

        Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(ueDevs.Get(i), 0);
        NS_ABORT_MSG_IF(!uePhy, "Missing UE PHY on mobile node " << ueNode->GetId());
        NodeType type = GetNodeType(ueNode->GetId());
        if (type == NODE_TYPE_RSU)
        {
            uePhy->SetAttribute("TxPower", DoubleValue(rsuTxPowerDbm));
        }
        else if (type == NODE_TYPE_BUS)
        {
            uePhy->SetAttribute("TxPower", DoubleValue(busTxPowerDbm));
        }

        Ptr<NrSpectrumPhy> sp = uePhy->GetSpectrumPhy();
        NS_ABORT_MSG_IF(!sp, "Missing UE SpectrumPhy on mobile node " << ueNode->GetId());

        Ptr<SpectrumChannel> ch = sp->GetSpectrumChannel();
        NS_ABORT_MSG_IF(!ch, "Missing SpectrumChannel on mobile node " << ueNode->GetId());
    }

    // Manually attach each UE to its closest gNB.

    auto AttachUeToGnbSafe = [&](Ptr<NetDevice> ueDev, Ptr<NetDevice> gnbDev) {
        auto gnbNetDev = DynamicCast<NrGnbNetDevice>(gnbDev);
        auto ueNetDev = DynamicCast<NrUeNetDevice>(ueDev);
        NS_ABORT_MSG_IF(!gnbNetDev, "Attach failed: gNB device is not NrGnbNetDevice");
        NS_ABORT_MSG_IF(!ueNetDev, "Attach failed: UE device is not NrUeNetDevice");

        if (!gnbNetDev->IsCellConfigured())
        {
            gnbNetDev->ConfigureCell();
        }

        uint32_t ccSize = gnbNetDev->GetCcMapSize();
        NS_ABORT_MSG_IF(ccSize == 0, "Attach failed: gNB has no CCs");
        for (uint32_t i = 0; i < ccSize; ++i)
        {
            Ptr<NrGnbPhy> gnbPhy = gnbNetDev->GetPhy(i);
            Ptr<NrUePhy> uePhy = ueNetDev->GetPhy(i);
            Ptr<NrGnbMac> gnbMac = gnbNetDev->GetMac(i);
            Ptr<NrMacScheduler> gnbSched = gnbNetDev->GetScheduler(i);

            NS_ABORT_MSG_IF(!gnbPhy, "Attach failed: null gNB PHY at CC " << i);
            NS_ABORT_MSG_IF(!uePhy, "Attach failed: null UE PHY at CC " << i);
            NS_ABORT_MSG_IF(!gnbMac, "Attach failed: null gNB MAC at CC " << i);
            NS_ABORT_MSG_IF(!gnbSched, "Attach failed: null gNB Scheduler at CC " << i);

            auto schedNs3 = DynamicCast<NrMacSchedulerNs3>(gnbSched);
            NS_ABORT_MSG_IF(!schedNs3,
                            "Attach failed: scheduler is not NrMacSchedulerNs3 at CC " << i);
            gnbPhy->RegisterUe(ueNetDev->GetImsi(), ueNetDev);
            uePhy->RegisterToGnb(gnbNetDev->GetBwpId(i));
            uePhy->SetDlAmc(schedNs3->GetDlAmc());

            uePhy->SetDlCtrlSyms(gnbMac->GetDlCtrlSyms());
            uePhy->SetUlCtrlSyms(gnbMac->GetUlCtrlSyms());
            uePhy->SetNumRbPerRbg(gnbMac->GetNumRbPerRbg());
            uePhy->SetRbOverhead(gnbPhy->GetRbOverhead());
            uePhy->SetSymbolsPerSlot(gnbPhy->GetSymbolsPerSlot());
            uePhy->SetNumerology(gnbPhy->GetNumerology());
            uePhy->SetPattern(gnbPhy->GetPattern());

            Ptr<NrEpcUeNas> ueNas = ueNetDev->GetNas();
            NS_ABORT_MSG_IF(!ueNas, "Attach failed: null UE NAS");
            ueNas->Connect(gnbNetDev->GetBwpId(i), gnbNetDev->GetEarfcn(i));
        }

        if (epcHelper)
        {
            epcHelper->ActivateEpsBearer(ueDev,
                                         ueNetDev->GetImsi(),
                                         NrEpcTft::Default(),
                                         NrEpsBearer(NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT));
        }

        ueNetDev->SetTargetGnb(gnbNetDev);
    };

    // Attach all UEs to the closest gNB using safe attach
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<NetDevice> ueDev = ueDevs.Get(i);
        NS_ABORT_MSG_IF(!ueDev, "Null UE device at index " << i);
        Ptr<Node> ueNode = ueDev->GetNode();
        NS_ABORT_MSG_IF(!ueNode, "UE device has null Node at index " << i);
        Ptr<MobilityModel> ueMm = ueNode->GetObject<MobilityModel>();
        NS_ABORT_MSG_IF(!ueMm,
                        "Missing UE mobility during attach for node " << ueDev->GetNode()->GetId());

        double minDistance = std::numeric_limits<double>::infinity();
        Ptr<NetDevice> closestGnb;
        for (uint32_t j = 0; j < gnbDevs.GetN(); ++j)
        {
            Ptr<NetDevice> gnbDev = gnbDevs.Get(j);
            NS_ABORT_MSG_IF(!gnbDev, "Null gNB device at index " << j);
            Ptr<Node> gnbNode = gnbDev->GetNode();
            NS_ABORT_MSG_IF(!gnbNode, "gNB device has null Node at index " << j);
            Ptr<MobilityModel> gnbMm = gnbNode->GetObject<MobilityModel>();
            NS_ABORT_MSG_IF(!gnbMm,
                            "Missing gNB mobility during attach for node "
                                << gnbDev->GetNode()->GetId());

            double distance = CalculateDistance(ueMm->GetPosition(), gnbMm->GetPosition());
            if (distance < minDistance)
            {
                minDistance = distance;
                closestGnb = gnbDev;
            }
        }
        NS_ABORT_MSG_IF(!closestGnb, "No gNB found for UE node " << ueDev->GetNode()->GetId());
        AttachUeToGnbSafe(ueDev, closestGnb);
    }

    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnbNetDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(i));
        NS_ABORT_MSG_IF(!gnbNetDev, "Missing NrGnbNetDevice at index " << i);
        g_cellIdToGnbNodeId[gnbNetDev->GetCellId()] = gnbNetDev->GetNode()->GetId();
        g_gnbNodeToCellId[gnbNetDev->GetNode()->GetId()] = gnbNetDev->GetCellId();
        g_gnbDeviceIndexByNodeId[gnbNetDev->GetNode()->GetId()] = gnbNetDev->GetIfIndex();
    }

    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ueNetDev = DynamicCast<NrUeNetDevice>(ueDevs.Get(i));
        NS_ABORT_MSG_IF(!ueNetDev, "Missing NrUeNetDevice at index " << i);
        Ptr<NrUeRrc> ueRrc = ueNetDev->GetRrc();
        NS_ABORT_MSG_IF(!ueRrc, "Missing NrUeRrc at index " << i);
        g_imsiToNodeId[ueNetDev->GetImsi()] = ueNetDev->GetNode()->GetId();
        g_vehicleDeviceIndexByNodeId[ueNetDev->GetNode()->GetId()] = ueNetDev->GetIfIndex();
        g_lastCellByImsi[ueNetDev->GetImsi()] = ueNetDev->GetCellId();
        UpdateVehicleBearerMapping(ueNetDev->GetNode()->GetId(),
                                   ueNetDev->GetCellId(),
                                   ueRrc->GetRnti());
        Simulator::Schedule(MilliSeconds(1),
                            &ConnectDownlinkTaskTracesForVehicle,
                            ueNetDev->GetNode()->GetId(),
                            ueNetDev->GetCellId(),
                            ueRrc->GetRnti(),
                            0);
    }

    // Address lists for vehicles and fogs.
    std::vector<Ipv4Address> vehicleAddresses;
    std::vector<Ipv4Address> fogAddresses;
    std::vector<Ptr<Node>> fogNodePtrs;
    std::unordered_map<Ipv4Address, NodeType, Ipv4Hash> addrToType;

    for (uint32_t i = 0; i < mobileNodes.GetN(); ++i)
    {
        Ipv4Address addr = ueIpIface.GetAddress(i);
        if (i < numVehicles)
        {
            vehicleAddresses.push_back(addr);
        }
        else
        {
            fogAddresses.push_back(addr);
            fogNodePtrs.push_back(mobileNodes.Get(i));
            addrToType[addr] = GetNodeType(mobileNodes.Get(i)->GetId());
        }
    }

    std::filesystem::create_directories(outputDir);
    std::string handoverCsv = outputDir + "/results_handover.csv";
    std::string handoverLatencyCsv = outputDir + "/handover_latency.csv";
    std::string methodologyCsv = outputDir + "/results_methodology_a.csv";
    std::string responsePathCsv = outputDir + "/results_response_path.csv";
    std::string downlinkRadioCsv = outputDir + "/results_downlink_radio.csv";
    std::string assocDiagnosticsCsv = outputDir + "/assoc_diagnostics.csv";
    g_handoverLog.open(handoverCsv.c_str(), std::ios::out);
    g_handoverLog << std::fixed << std::setprecision(6);
    g_handoverLog << "time_s,vehicle_id,old_cell_id,new_cell_id,interruption_s\n";
    g_methodologyLog.open(methodologyCsv.c_str(), std::ios::out);
    g_methodologyLog << std::fixed << std::setprecision(6);
    g_methodologyLog << "Time_Seconds,Vehicle_ID,Connected_Node_Type,Relative_Velocity_kmh,"
                        "Packet_State,Handover_Event,Fog_Reassociation_Event\n";
    g_responsePathLog.open(responsePathCsv.c_str(), std::ios::out);
    g_responsePathLog << std::fixed << std::setprecision(6);
    g_responsePathLog << "time_s,task_id,fog_id,vehicle_id,observed_src_ip,resolved_dst_ip,"
                         "resolved_dst_port,bytes_sent\n";
    g_downlinkRadioLog.open(downlinkRadioCsv.c_str(), std::ios::out);
    g_downlinkRadioLog << std::fixed << std::setprecision(6);
    g_downlinkRadioLog << "Time_Seconds,Stage,Task_Id,Gnb_NodeId,Vehicle_NodeId,CellId,Rnti,"
                          "Lcid,Bytes,Delay_Seconds\n";
    g_assocDiagnosticsLog.open(assocDiagnosticsCsv.c_str(), std::ios::out);
    g_assocDiagnosticsLog << std::fixed << std::setprecision(6);
    g_assocDiagnosticsLog << "row_type,time_s,vehicle_id,ever_associated,"
                             "first_association_time_s,fog_id,fog_type,distance_m,snr_db,"
                             "queue_size,first_association,is_migration\n";

    Ptr<TaskLogger> logger = CreateObject<TaskLogger>();
    std::string tasksCsv = outputDir + "/results_tasks.csv";
    std::string summaryCsv = outputDir + "/" + simTag + "-summary.csv";
    logger->Open(tasksCsv);

    // Fog applications: upload sink and task processing.
    ApplicationContainer fogApps;
    for (uint32_t i = 0; i < fogNodes.GetN(); ++i)
    {
        Ptr<FogServerApp> fogApp = CreateObject<FogServerApp>();
        fogApp->Setup(fogPort, serviceTime, logger);
        fogNodes.Get(i)->AddApplication(fogApp);
        fogApp->SetStartTime(Seconds(0.5));
        fogApp->SetStopTime(simTime);
        fogApps.Add(fogApp);
    }

    // Vehicle task senders.
    ApplicationContainer vehicleApps;
    for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i)
    {
        Ptr<VehicleApp> sender = CreateObject<VehicleApp>();
        sender->Setup(fogNodePtrs,
                      fogAddresses,
                      fogPort,
                      eventRate,
                      taskSizeBytes,
                      taskDeadline,
                      assocInterval,
                      testMode,
                      queueLimit,
                      sinrThresholdDb,
                      busTxPowerDbm,
                      rsuTxPowerDbm);
        vehicleNodes.Get(i)->AddApplication(sender);
        sender->SetStartTime(Seconds(1.0));
        sender->SetStopTime(simTime);
        vehicleApps.Add(sender);
    }

    // Downlink broadcast receivers.
    PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                            InetSocketAddress(Ipv4Address::GetAny(), downlinkPort));
    ApplicationContainer dlSinks = dlSink.Install(vehicleNodes);
    dlSinks.Start(Seconds(0.2));
    dlSinks.Stop(simTime);

    // Downlink broadcast senders on fog nodes.
    ApplicationContainer dlSenders;
    for (uint32_t i = 0; i < fogNodes.GetN(); ++i)
    {
        Ptr<BroadcastSenderApp> bcast = CreateObject<BroadcastSenderApp>();
        bcast->Setup(vehicleAddresses, downlinkPort, downlinkInterval, downlinkPacketSize);
        fogNodes.Get(i)->AddApplication(bcast);
        bcast->SetStartTime(Seconds(0.5));
        bcast->SetStopTime(simTime);
        dlSenders.Add(bcast);
    }

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> flowmon = flowmonHelper.InstallAll();

    std::unique_ptr<AnimationInterface> anim;
    if (!animFile.empty())
    {
        anim = std::make_unique<AnimationInterface>(animFile);
        for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
        {
            anim->UpdateNodeDescription(gnbNodes.Get(i), "gNB-" + std::to_string(i));
            anim->UpdateNodeColor(gnbNodes.Get(i), 255, 0, 0); // red
        }
        for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i)
        {
            anim->UpdateNodeDescription(vehicleNodes.Get(i), "UE-" + std::to_string(i));
            anim->UpdateNodeColor(vehicleNodes.Get(i), 0, 0, 255); // blue
        }
        for (uint32_t i = 0; i < fogNodes.GetN(); ++i)
        {
            anim->UpdateNodeDescription(fogNodes.Get(i), "Fog-" + std::to_string(i));
            anim->UpdateNodeColor(fogNodes.Get(i), 0, 150, 0); // green
        }
    }

    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionReconfiguration",
                    MakeCallback(&NotifyUeConnectionReconfiguration));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                    MakeCallback(&NotifyUeHandoverStart));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                    MakeCallback(&NotifyUeHandoverEndOk));
    Simulator::Schedule(Seconds(2.0), &ConnectAllDownlinkTaskTraces);
    Simulator::Schedule(Seconds(5.0), &ConnectAllDownlinkTaskTraces);

    Simulator::Stop(simTime + Seconds(0.1));
    Simulator::Run();

    flowmon->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());

    std::string throughputCsv = outputDir + "/" + simTag + "-throughput.csv";
    std::ofstream throughput(throughputCsv.c_str(), std::ios::out);
    throughput << "flow_id,src,src_port,dst,dst_port,tx_bytes,rx_bytes,first_tx_s,last_rx_s,"
                  "throughput_mbps,mean_delay_s,loss_ratio\n";
    throughput << std::fixed << std::setprecision(6);

    auto stats = flowmon->GetFlowStats();

    struct RelAgg
    {
        uint64_t txPackets{0};
        uint64_t rxPackets{0};
        uint64_t txBytes{0};
        uint64_t rxBytes{0};
        double delaySum{0.0};
    };

    std::unordered_map<NodeType, RelAgg> reliability;

    for (const auto& [flowId, stat] : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        double duration = (stat.timeLastRxPacket - stat.timeFirstTxPacket).GetSeconds();
        double throughputMbps = 0.0;
        if (duration > 0.0)
        {
            throughputMbps = (stat.rxBytes * 8.0) / duration / 1e6;
        }
        double meanDelay = stat.rxPackets > 0 ? stat.delaySum.GetSeconds() / stat.rxPackets : 0.0;
        double lossRatio =
            stat.txPackets > 0
                ? static_cast<double>(stat.txPackets - stat.rxPackets) / stat.txPackets
                : 0.0;

        throughput << flowId << ',' << t.sourceAddress << ',' << t.sourcePort << ','
                   << t.destinationAddress << ',' << t.destinationPort << ',' << stat.txBytes << ','
                   << stat.rxBytes << ',' << stat.timeFirstTxPacket.GetSeconds() << ','
                   << stat.timeLastRxPacket.GetSeconds() << ',' << throughputMbps << ','
                   << meanDelay << ',' << lossRatio << '\n';

        auto typeIt = addrToType.find(t.destinationAddress);
        if (typeIt != addrToType.end())
        {
            RelAgg& agg = reliability[typeIt->second];
            agg.txPackets += stat.txPackets;
            agg.rxPackets += stat.rxPackets;
            agg.txBytes += stat.txBytes;
            agg.rxBytes += stat.rxBytes;
            agg.delaySum += stat.delaySum.GetSeconds();
        }
    }

    throughput.close();

    std::string reliabilityCsv = outputDir + "/results_reliability.csv";
    std::ofstream relOut(reliabilityCsv.c_str(), std::ios::out);
    relOut << "NodeType,TotalTx,TotalRx,LossRatio,AvgDelay\n";
    relOut << std::fixed << std::setprecision(6);
    auto writeRel = [&](NodeType type, const std::string& label) {
        auto it = reliability.find(type);
        if (it == reliability.end())
        {
            relOut << label << ",0,0,0,0\n";
            return;
        }
        const RelAgg& agg = it->second;
        double loss = agg.txPackets > 0
                          ? static_cast<double>(agg.txPackets - agg.rxPackets) / agg.txPackets
                          : 0.0;
        double avgDelay = agg.rxPackets > 0 ? agg.delaySum / agg.rxPackets : 0.0;
        relOut << label << ',' << agg.txPackets << ',' << agg.rxPackets << ',' << loss << ','
               << avgDelay << '\n';
    };
    writeRel(NODE_TYPE_RSU, "CFN");
    writeRel(NODE_TYPE_BUS, "VFN");
    relOut.close();

    logger->WriteSummary(summaryCsv);
    logger->Close();

    // Per-run task accounting written as a small summary CSV so the
    // pipeline denominator is unambiguous without per-packet logging.
    {
        uint64_t flowTxPackets = 0;
        uint64_t uplinkRxPackets = 0;
        for (const auto& [type, agg] : reliability)
        {
            flowTxPackets += agg.txPackets;
            uplinkRxPackets += agg.rxPackets;
        }

        const uint64_t successTasks = logger->GetSuccessTasks();
        const uint64_t deadlineMissTasks = logger->GetDeadlineMissTasks();
        const uint64_t completedTasks = successTasks + deadlineMissTasks;
        auto subClamp = [](uint64_t lhs, uint64_t rhs) {
            return lhs >= rhs ? lhs - rhs : 0;
        };
        const uint64_t uplinkLossTasks = subClamp(g_counters.sent_tasks, uplinkRxPackets);
        const uint64_t returnPathLossTasks = subClamp(uplinkRxPackets, completedTasks);
        const uint64_t classifiedTasks = g_counters.no_association_drops + uplinkLossTasks +
                                         returnPathLossTasks + deadlineMissTasks + successTasks;
        const uint64_t unclassifiedTasks = subClamp(g_counters.offered_tasks, classifiedTasks);
        const double offeredReliability =
            g_counters.offered_tasks > 0
                ? static_cast<double>(successTasks) / g_counters.offered_tasks
                : 0.0;
        const double sentReliability =
            g_counters.sent_tasks > 0 ? static_cast<double>(successTasks) / g_counters.sent_tasks
                                      : 0.0;
        const double uplinkDeliveryRate =
            g_counters.sent_tasks > 0
                ? static_cast<double>(uplinkRxPackets) / g_counters.sent_tasks
                : 0.0;

        std::ofstream tcOut(outputDir + "/task_counters.csv");
        tcOut << "metric,value\n";
        tcOut << "offered_tasks," << g_counters.offered_tasks << '\n';
        tcOut << "sent_tasks," << g_counters.sent_tasks << '\n';
        tcOut << "no_association_drops," << g_counters.no_association_drops << '\n';
        tcOut << "flow_monitor_tx_packets," << flowTxPackets << '\n';
        tcOut << "uplink_rx_packets," << uplinkRxPackets << '\n';
        tcOut << "uplink_loss_tasks," << uplinkLossTasks << '\n';
        tcOut << "success_tasks," << successTasks << '\n';
        tcOut << "deadline_miss_tasks," << deadlineMissTasks << '\n';
        tcOut << "return_path_loss_tasks," << returnPathLossTasks << '\n';
        tcOut << "unclassified_loss_tasks," << unclassifiedTasks << '\n';
        tcOut << std::fixed << std::setprecision(6);
        tcOut << "offered_load_reliability," << offeredReliability << '\n';
        tcOut << "sent_load_reliability," << sentReliability << '\n';
        tcOut << "uplink_delivery_rate," << uplinkDeliveryRate << '\n';
        tcOut.close();
    }

    if (g_assocDiagnosticsLog.is_open())
    {
        for (uint32_t i = 0; i < vehicleNodes.GetN(); ++i)
        {
            const uint32_t vehicleId = vehicleNodes.Get(i)->GetId();
            auto assocIt = g_vehicleFirstAssocTime.find(vehicleId);
            const bool everAssociated = assocIt != g_vehicleFirstAssocTime.end();
            const double firstAssocTime = everAssociated ? assocIt->second.GetSeconds() : -1.0;
            g_assocDiagnosticsLog << "vehicle_summary,," << vehicleId << ','
                                  << (everAssociated ? 1 : 0) << ',' << firstAssocTime
                                  << ",,,,,,,\n";
        }
    }

    if (g_handoverLog.is_open())
    {
        g_handoverLog.close();
    }
    if (g_methodologyLog.is_open())
    {
        g_methodologyLog.close();
    }
    if (g_responsePathLog.is_open())
    {
        g_responsePathLog.close();
    }
    if (g_downlinkRadioLog.is_open())
    {
        g_downlinkRadioLog.close();
    }
    if (g_assocDiagnosticsLog.is_open())
    {
        g_assocDiagnosticsLog.close();
    }
    // Mirror handover log to legacy filename for older readers.
    if (std::filesystem::exists(handoverCsv))
    {
        std::error_code ec;
        std::filesystem::copy_file(handoverCsv,
                                   handoverLatencyCsv,
                                   std::filesystem::copy_options::overwrite_existing,
                                   ec);
    }

    Simulator::Destroy();

    return 0;
}
