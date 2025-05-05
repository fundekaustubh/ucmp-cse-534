#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ucmp-routing-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UcmpTestSimulation");

// static void OnPacketReceive(Ptr<const Packet> packet, const Address &from)
// {
// 	NS_LOG_INFO("Sink received a packet from " << from);
// }

// static void OnPacketSend(Ptr<const Packet> packet, const Address &to)
// {
// 	NS_LOG_INFO("Source sent a packet to " << to);
// }

int main(int argc, char *argv[])
{
	LogComponentEnable("UcmpRouting", LOG_LEVEL_INFO);
	LogComponentEnable("UcmpTestSimulation", LOG_LEVEL_INFO);

	uint32_t numToRs = 8;
	uint32_t hostsPerToR = 2;
	NodeContainer tors, hosts;
	tors.Create(numToRs);
	hosts.Create(numToRs * hostsPerToR);

	// Point-to-point helpers
	PointToPointHelper p2p;
	p2p.SetDeviceAttribute("DataRate", StringValue("100Gbps"));
	p2p.SetChannelAttribute("Delay", StringValue("500ns"));

	InternetStackHelper stack;
	UcmpRoutingHelper ucmp;
	stack.SetRoutingHelper(ucmp);
	stack.Install(tors);
	stack.Install(hosts);

	Ipv4AddressHelper ip;
	uint32_t netCount = 1;

	// Connect each host to its ToR
	for (uint32_t t = 0; t < numToRs; ++t)
	{
		for (uint32_t h = 0; h < hostsPerToR; ++h)
		{
			NetDeviceContainer link = p2p.Install(tors.Get(t), hosts.Get(t * hostsPerToR + h));
			std::ostringstream subnet;
			subnet << "10." << netCount++ << ".0.0";
			ip.SetBase(subnet.str().c_str(), "255.255.255.0");
			ip.Assign(link);
		}
	}

	// Connect ToRs in a mesh (for path diversity)
	for (uint32_t i = 0; i < numToRs; ++i)
	{
		for (uint32_t j = i + 1; j < numToRs; ++j)
		{
			NetDeviceContainer link = p2p.Install(tors.Get(i), tors.Get(j));
			std::ostringstream subnet;
			subnet << "10." << netCount++ << ".0.0";
			ip.SetBase(subnet.str().c_str(), "255.255.255.0");
			ip.Assign(link);
		}
	}

	// Install TCP flow: From Host0 to Host15
	uint16_t port = 50000;
	Address sinkAddr(InetSocketAddress(hosts.Get(15)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal(), port));
	PacketSinkHelper sink("ns3::TcpSocketFactory", sinkAddr);
	ApplicationContainer sinkApp = sink.Install(hosts.Get(15));
	sinkApp.Start(Seconds(0.0));
	sinkApp.Stop(Seconds(10.0));

	// sinkApp.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&OnPacketReceive));

	OnOffHelper client("ns3::TcpSocketFactory", sinkAddr);
	client.SetAttribute("DataRate", StringValue("10Gbps"));
	client.SetAttribute("PacketSize", UintegerValue(1000));
	client.SetAttribute("StartTime", TimeValue(Seconds(1.0)));
	client.SetAttribute("StopTime", TimeValue(Seconds(9.0)));
	ApplicationContainer clientApp = client.Install(hosts.Get(0));

	// clientApp.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&OnPacketSend));

	Ipv4GlobalRoutingHelper::PopulateRoutingTables();
	NS_LOG_INFO("Simulation setup complete. Starting simulation...");
	Simulator::Run();
	NS_LOG_INFO("Simulation complete.");
	Simulator::Destroy();
	return 0;
}
