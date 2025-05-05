#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ucmp-routing-helper.h"
#include "ns3/ucmp-routing.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/tcp-socket-base.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UcmpSliceSimulation");

uint32_t numToRs = 3;
uint32_t hostsPerToR = 2;
uint32_t numSlices = 3;
uint32_t numFlows = 6;
Time sliceInterval = MilliSeconds(3);

std::vector<std::vector<std::pair<uint32_t, uint32_t>>> slices = {
    {{0, 1}}, {{1, 2}}, {{2, 0}}};

std::vector<NetDeviceContainer> torLinks;
std::vector<Ptr<PointToPointChannel>> torChannels;

std::map<uint32_t, Time> flowStartTimes;
std::map<uint32_t, Time> flowEndTimes;
std::map<Address, uint32_t> addressToFlowId;

void FlowCompletionTimeLogger(uint32_t flowId)
{
    Time start = flowStartTimes[flowId];
    Time end = flowEndTimes[flowId];
    NS_LOG_INFO("Flow " << flowId
                        << " completed in " << (end - start).GetMilliSeconds()
                        << " ms at " << Simulator::Now().GetSeconds() << "s");
}

void OnPacketSend(Ptr<const Packet> p, const Address &from, const Address &to)
{
    uint32_t flowId = addressToFlowId[to];
    NS_LOG_INFO("[App Tx]   t=" << Simulator::Now().GetSeconds()
                                << "   from=" << from << "   to=" << to
                                << "   size=" << p->GetSize());
    if (flowStartTimes.find(flowId) == flowStartTimes.end())
    {
        flowStartTimes[flowId] = Simulator::Now();
        NS_LOG_INFO("Flow " << flowId
                            << " started at " << Simulator::Now().GetSeconds() << "s");
    }
}

void OnPacketReceive(Ptr<const Packet> p, const Address &from, const Address &to)
{
    uint32_t flowId = addressToFlowId[to];
    flowEndTimes[flowId] = Simulator::Now();
    NS_LOG_INFO("[App Rx]   t=" << Simulator::Now().GetSeconds()
                                << "   from=" << from << "   to=" << to
                                << "   size=" << p->GetSize());
    Simulator::ScheduleNow(&FlowCompletionTimeLogger, flowId);
}

void SliceActivator(uint32_t sliceIndex, NodeContainer tors)
{
    // Initially set all delays to a large value
    for (auto &ch : torChannels)
    {
        ch->SetAttribute("Delay", TimeValue(Seconds(1000)));
    }
    // For all present slices, set delay to a very low value
    for (auto [u, v] : slices[sliceIndex])
    {
        for (size_t i = 0; i < torLinks.size(); ++i)
        {
            Ptr<Node> n0 = torLinks[i].Get(0)->GetNode();
            Ptr<Node> n1 = torLinks[i].Get(1)->GetNode();
            if ((n0->GetId() == u && n1->GetId() == v) ||
                (n0->GetId() == v && n1->GetId() == u))
            {
                torChannels[i]->SetAttribute("Delay", TimeValue(NanoSeconds(500)));
            }
        }
    }
    for (uint32_t i = 0; i < tors.GetN(); ++i)
    {
        Ptr<UcmpRouting> proto = tors.Get(i)->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<UcmpRouting>();
        proto->InitializeRoutes();
    }
    // Call slice activator again after scheduled slice interval
    Simulator::Schedule(sliceInterval, &SliceActivator, (sliceIndex + 1) % numSlices, tors);
}

int main(int argc, char *argv[])
{
    LogComponentEnable("UcmpRouting", LOG_LEVEL_INFO);
    LogComponentEnable("UcmpSliceSimulation", LOG_LEVEL_INFO);
    // Enable ECN
    Config::SetDefault("ns3::TcpSocketBase::UseEcn", EnumValue(TcpSocketState::On));
    NS_LOG_INFO("===== Simulation begins =====");

    // Setup topology
    NodeContainer tors, hosts;
    tors.Create(numToRs);
    hosts.Create(numToRs * hostsPerToR);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Gbps"));
    p2p.SetChannelAttribute("Delay", StringValue("500ns"));

    // Add UCMP routing helper to the stack
    InternetStackHelper stack;
    UcmpRoutingHelper ucmp;
    stack.SetRoutingHelper(ucmp);
    stack.Install(tors);
    stack.Install(hosts);

    Ipv4AddressHelper ip;
    uint32_t netCount = 1;

    // ToR–host links
    for (uint32_t t = 0; t < numToRs; ++t)
    {
        for (uint32_t h = 0; h < hostsPerToR; ++h)
        {
            NetDeviceContainer link = p2p.Install(tors.Get(t), hosts.Get(t * hostsPerToR + h));
            std::ostringstream subnet;
            subnet << "10." << netCount++ << ".0.0";
            ip.SetBase(subnet.str().c_str(), "255.255.255.0");
            ip.Assign(link);
            // NS_LOG_INFO("Created ToR " << t
            //                              << " <-> Host " << (t * hostsPerToR + h));
        }
    }

    // // Log host IPs
    // for (uint32_t h = 0; h < hosts.GetN(); ++h)
    // {
    // Ipv4Address addr = hosts.Get(h)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    // NS_LOG_INFO("Host " << h << " has IP " << addr);
    // }

    // // Log ToR IPs
    // for (uint32_t t = 0; t < tors.GetN(); ++t)
    // {
    // Ipv4Address addr = tors.Get(t)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    // NS_LOG_INFO("ToR " << t << " has IP " << addr);
    // }

    TrafficControlHelper tch;
    tch.SetRootQueueDisc("ns3::RedQueueDisc",
                         "MinTh", DoubleValue(20),
                         "MaxTh", DoubleValue(50),
                         "MaxSize", QueueSizeValue(QueueSize("1000p")),
                         "LinkBandwidth", DataRateValue(DataRate("100Gbps")),
                         "LinkDelay", TimeValue(NanoSeconds(500)),
                         "UseEcn", BooleanValue(true));

    // Define ToR–ToR slice links
    for (auto &slice : slices)
    {
        for (auto [i, j] : slice)
        {
            if (i >= tors.GetN() || j >= tors.GetN())
            {
                NS_LOG_INFO("Invalid slice link " << i << "," << j
                                                  << " : there are only " << tors.GetN() << " ToRs");
                continue;
            }
            // Set up new link from tor i <-> tor j
            NetDeviceContainer link = p2p.Install(tors.Get(i), tors.Get(j));
            torLinks.push_back(link);
            torChannels.push_back(DynamicCast<PointToPointChannel>(link.Get(0)->GetChannel()));
            tch.Install(link);
            // Define subnet for this link
            std::ostringstream subnet;
            subnet << "10." << netCount++ << ".0.0";
            ip.SetBase(subnet.str().c_str(), "255.255.255.0");
            ip.Assign(link);
            torChannels.back()->SetAttribute("Delay", TimeValue(Seconds(1000)));
            // NS_LOG_INFO("Created Slice Link: ToR " << i << " <-> ToR " << j);
        }
    }

    // Log interface IPs for ToRs to verify connections
    for (uint32_t n = 0; n < tors.GetN(); ++n)
    {
        Ptr<Node> node = tors.Get(n);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        for (uint32_t iface = 1; iface < ipv4->GetNInterfaces(); ++iface)
        {
            std::ostringstream oss;
            oss << "ToR " << node->GetId()
                << " interface " << iface << " addrs:";
            for (uint32_t a = 0; a < ipv4->GetNAddresses(iface); ++a)
            {
                oss << " " << ipv4->GetAddress(iface, a).GetLocal();
            }
            NS_LOG_INFO(oss.str());
        }
    }

    // Log interface IPs for hosts to verify connections
    for (uint32_t h = 0; h < hosts.GetN(); ++h)
    {
        Ptr<Node> node = hosts.Get(h);
        Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
        for (uint32_t iface = 1; iface < ipv4->GetNInterfaces(); ++iface)
        {
            std::ostringstream oss;
            oss << "Host " << node->GetId()
                << " IF " << iface << " addrs:";
            for (uint32_t a = 0; a < ipv4->GetNAddresses(iface); ++a)
            {
                oss << " " << ipv4->GetAddress(iface, a).GetLocal();
            }
            NS_LOG_INFO(oss.str());
        }
    }

    // Initialize UCMP on every node
    for (uint32_t i = 0; i < tors.GetN(); ++i)
    {
        uint32_t id = tors.Get(i)->GetId();
        Ptr<UcmpRouting> proto = tors.Get(i)->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<UcmpRouting>();
        if (proto)
        {
            proto->InitializeRoutes();
            // NS_LOG_INFO("Initialized routes for ToR " << n->GetId());
        }
        else
        {
            NS_LOG_INFO("Init failed for ToR " << id);
        }
    }

    for (uint32_t i = 0; i < hosts.GetN(); ++i)
    {
        uint32_t id = hosts.Get(i)->GetId();
        Ptr<Ipv4> ipv4 = hosts.Get(i)->GetObject<Ipv4>();
        Ptr<UcmpRouting> proto = hosts.Get(i)->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<UcmpRouting>();
        if (proto)
        {
            proto->InitializeRoutes();
            // NS_LOG_INFO("Initialized routes for Host " << n->GetId());
        }
        else
        {
            NS_LOG_INFO("Init failed for Host " << id);
        }
    }

    // Traffic setup

    ApplicationContainer allApps;

    for (uint32_t f = 0; f < numFlows; ++f)
    {
        uint32_t srcHost = f;
        uint32_t dstHost = hosts.GetN() - f - 1;
        uint16_t port = 50000 + f;

        // Setup sink
        Ipv4Address dstAddr = hosts.Get(dstHost)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
        Address sinkAddr(InetSocketAddress(dstAddr, port));
        addressToFlowId[sinkAddr] = f;

        // Setup packet sink helper
        PacketSinkHelper sink("ns3::TcpSocketFactory", sinkAddr);
        auto sinkApp = sink.Install(hosts.Get(dstHost));
        sinkApp.Start(Seconds(0.0));
        sinkApp.Stop(Seconds(20.0));
        Ptr<PacketSink> pktSink = DynamicCast<PacketSink>(sinkApp.Get(0));
        pktSink->TraceConnectWithoutContext("RxWithAddresses",
                                            MakeCallback(&OnPacketReceive));
        NS_LOG_INFO("Flow " << f << " Sink is Host " << dstHost);

        // Setup client
        OnOffHelper client("ns3::TcpSocketFactory", sinkAddr);
        client.SetAttribute("DataRate", StringValue("10Gbps"));
        client.SetAttribute("PacketSize", UintegerValue(1000));
        client.SetAttribute("MaxBytes", UintegerValue(1000));
        client.SetAttribute("StartTime", TimeValue(Seconds(1.0 + 0.2 * f)));

        auto cliApp = client.Install(hosts.Get(srcHost));
        cliApp.Get(0)->TraceConnectWithoutContext("TxWithAddresses",
                                                  MakeCallback(&OnPacketSend));
        NS_LOG_INFO("Flow " << f << " Client is Host " << srcHost);

        allApps.Add(sinkApp);
        allApps.Add(cliApp);
    }

    // Record stats using flow monitor helper

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    // Begin simulation
    NS_LOG_INFO("Scheduling slices and starting simulation...");
    Simulator::Schedule(Seconds(0.1), &SliceActivator, 0, tors);
    Simulator::Stop(Seconds(45.0));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    for (auto &kv : stats)
    {
        FlowId fid = kv.first;
        auto &st = kv.second;
        Ipv4FlowClassifier::FiveTuple ft = classifier->FindFlow(fid);
        std::cout << "Flow " << fid
                  << ft.sourceAddress << " to " << ft.destinationAddress << "\n"
                  << "   TxPackets = " << st.txPackets
                  << ",  RxPackets = " << st.rxPackets
                  << ",  Lost = " << st.lostPackets
                  << std::endl;
    }
    NS_LOG_INFO("Simulation complete.");
    Simulator::Destroy();
    return 0;
}
