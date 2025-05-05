/*
 * scratch/ucmp-performance.cc
 * Simulation script to evaluate UCMP performance based on
 * "Uniform-Cost Multi-Path Routing for Reconfigurable Data Center Networks"
 * SIGCOMM '24 paper by Li et al.
 *
 * Build with:
 *   ./waf configure --enable-examples --build-profile=debug
 *   ./waf build
 *
 * Usage:
 *   ./waf --run "scratch/ucmp-performance \
 *     --KFat=8 --Alpha=0.5 --KPaths=4 \
 *     --workload=websearch --simTime=1.0"
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ucmp-routing-helper.h"

using namespace ns3;

void BuildFatTree(uint32_t k,
                  NodeContainer &hosts,
                  NodeContainer &torSwitches,
                  NetDeviceContainer &hostDevices,
                  NetDeviceContainer &torDevices,
                  Ipv4InterfaceContainer &hostInterfaces)
{
    uint32_t podCount             = k;
    uint32_t edgeCount            = k / 2;
    uint32_t aggCount             = k / 2;
    uint32_t coreCount            = (k / 2) * (k / 2);
    uint32_t hostsPerEdge         = k / 2;

    // Create nodes
    NodeContainer edgeSwitches; edgeSwitches.Create(podCount * edgeCount);
    NodeContainer aggSwitches;  aggSwitches.Create(podCount * aggCount);
    NodeContainer coreSwitches; coreSwitches.Create(coreCount);
    NodeContainer hostNodes;    hostNodes.Create(podCount * edgeCount * hostsPerEdge);

    // Populate references
    hosts.Add(hostNodes);
    torSwitches.Add(edgeSwitches);
    torSwitches.Add(aggSwitches);
    torSwitches.Add(coreSwitches);

    // Point-to-Point helper
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay", StringValue("500ns"));

    // Address helper
    Ipv4AddressHelper ipv4;
    uint32_t netCount = 0;

    // 1) Connect hosts to edge switches
    for (uint32_t pod = 0; pod < podCount; ++pod)
    {
        for (uint32_t es = 0; es < edgeCount; ++es)
        {
            Ptr<Node> eswitch = edgeSwitches.Get(pod * edgeCount + es);
            for (uint32_t h = 0; h < hostsPerEdge; ++h)
            {
                uint32_t hostIndex = pod * edgeCount * hostsPerEdge + es * hostsPerEdge + h;
                Ptr<Node> host = hostNodes.Get(hostIndex);
                NetDeviceContainer link = p2p.Install(eswitch, host);
                // Assign unique /24 network per link
                ++netCount;
                std::ostringstream base;
                base << "10." << (netCount/256) << "." << (netCount%256) << ".0";
                ipv4.SetBase(base.str().c_str(), "255.255.255.0");
                Ipv4InterfaceContainer ifc = ipv4.Assign(link);
                hostDevices.Add(link.Get(1));
                torDevices.Add(link.Get(0));
                hostInterfaces.Add(ifc.Get(1));
            }
        }
    }

    // 2) Connect edge to aggregation switches within each pod
    for (uint32_t pod = 0; pod < podCount; ++pod)
    {
        for (uint32_t es = 0; es < edgeCount; ++es)
        {
            Ptr<Node> eswitch = edgeSwitches.Get(pod * edgeCount + es);
            for (uint32_t ag = 0; ag < aggCount; ++ag)
            {
                Ptr<Node> aswitch = aggSwitches.Get(pod * aggCount + ag);
                NetDeviceContainer link = p2p.Install(eswitch, aswitch);
                ++netCount;
                std::ostringstream base;
                base << "10." << (netCount/256) << "." << (netCount%256) << ".0";
                ipv4.SetBase(base.str().c_str(), "255.255.255.0");
                Ipv4InterfaceContainer ifc = ipv4.Assign(link);
                torDevices.Add(link.Get(0));
                torDevices.Add(link.Get(1));
            }
        }
    }

    // 3) Connect aggregation to core switches
    // Core is organized in (k/2) groups of (k/2)
    for (uint32_t ag = 0; ag < aggCount; ++ag)
    {
        for (uint32_t pod = 0; pod < podCount; ++pod)
        {
            Ptr<Node> aswitch = aggSwitches.Get(pod * aggCount + ag);
            for (uint32_t c = 0; c < (coreCount / (k/2)); ++c)
            {
                // map to core index
                uint32_t coreIndex = c * aggCount + ag;
                Ptr<Node> cswitch = coreSwitches.Get(coreIndex);
                NetDeviceContainer link = p2p.Install(aswitch, cswitch);
                ++netCount;
                std::ostringstream base;
                base << "10." << (netCount/256) << "." << (netCount%256) << ".0";
                ipv4.SetBase(base.str().c_str(), "255.255.255.0");
                Ipv4InterfaceContainer ifc = ipv4.Assign(link);
                torDevices.Add(link.Get(0));
                torDevices.Add(link.Get(1));
            }
        }
    }
}

int
main(int argc, char *argv[])
{
    uint32_t kFat       = 8;
    double   alpha      = 0.5;
    uint32_t kPaths     = 4;
    std::string workload = "websearch";
    double   simTime    = 1.0;

    CommandLine cmd;
    cmd.AddValue("KFat",    "Fat-tree parameter k (even)",           kFat);
    cmd.AddValue("Alpha",   "UCMP Alpha weighting factor",           alpha);
    cmd.AddValue("KPaths",  "Number of UCMP paths (K)",             kPaths);
    cmd.AddValue("workload","Workload: websearch or datamining",     workload);
    cmd.AddValue("simTime", "Simulation time in seconds",           simTime);
    cmd.Parse(argc, argv);

    // Build topology
    NodeContainer hosts;
    NodeContainer torSwitches;
    NetDeviceContainer hostDevices;
    NetDeviceContainer torDevices;
    Ipv4InterfaceContainer hostInterfaces;
    BuildFatTree(kFat, hosts, torSwitches, hostDevices, torDevices, hostInterfaces);

    // Install routing on ToR switches (UCMP + static fallback)
    InternetStackHelper stack;
    Ipv4ListRoutingHelper listRH;
    UcmpRoutingHelper ucmp;
    ucmp.Set("Alpha",  DoubleValue(alpha));
    ucmp.Set("KPaths", UintegerValue(kPaths));
    listRH.Add(ucmp, 0);
    Ipv4StaticRoutingHelper staticRH;
    listRH.Add(staticRH, 10);
    stack.SetRoutingHelper(listRH);
    stack.Install(torSwitches);
    // Hosts just need IP stack
    InternetStackHelper hostStack;
    hostStack.Install(hosts);

    // Create traffic: each host -> host halfway around
    uint16_t port = 50000;
    ApplicationContainer apps;
    for (uint32_t i = 0; i < hosts.GetN(); ++i)
    {
        Ptr<Node> src = hosts.Get(i);
        Ptr<Node> dst = hosts.Get((i + hosts.GetN()/2) % hosts.GetN());

        // Sink on dst
        PacketSinkHelper sinkH("ns3::UdpSocketFactory",
                               InetSocketAddress(hostInterfaces.GetAddress(i), port));
        apps.Add(sinkH.Install(dst));

        // OnOff client on src
        OnOffHelper onoff("ns3::UdpSocketFactory",
                          Address(InetSocketAddress(hostInterfaces.GetAddress(i), port)));
        onoff.SetAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
        onoff.SetAttribute("PacketSize", UintegerValue(1024));
        if (workload == "websearch") {
            onoff.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=0.01]"));
            onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        } else {
            onoff.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        }
        apps.Add(onoff.Install(src));
    }
    apps.Start(Seconds(0.1));
    apps.Stop(Seconds(simTime));

    // Flow monitor
    FlowMonitorHelper fmHelper;
    Ptr<FlowMonitor> monitor = fmHelper.InstallAll();

    Simulator::Stop(Seconds(simTime + 0.1));
    Simulator::Run();

    // Compute average FCT
    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();
    double totalFct = 0.0;
    uint64_t count  = 0;
    for (auto &kv : stats)
    {
        const auto &fs = kv.second;
        if (fs.rxPackets > 0)
        {
            double fct = fs.timeLastRxPacket.GetSeconds() - fs.timeFirstTxPacket.GetSeconds();
            totalFct += fct;
            count++;
        }
    }
    std::cout << "Average Flow Completion Time = "
              << (totalFct / count) << " s" << std::endl;

    Simulator::Destroy();
    return 0;
}
