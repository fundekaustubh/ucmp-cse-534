#ifndef NS3_UCMP_ROUTING_H
#define NS3_UCMP_ROUTING_H

#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-address.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/ipv4-header.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/attribute-helper.h"
#include <unordered_map>
#include <map>
#include <vector>
#include <cstdint>
#include <set>

namespace ns3
{
	using FlowId = uint32_t;
	struct Link
	{
		uint32_t node;
		double cost;
	};

	struct Ipv4AddrHash
	{
		size_t operator()(Ipv4Address const &a) const noexcept
		{
			return std::hash<uint32_t>()(a.Get());
		}
	};
	struct Ipv4AddrEq
	{
		bool operator()(Ipv4Address const &a, Ipv4Address const &b) const noexcept
		{
			return a == b;
		}
	};

	class UcmpRouting : public Ipv4RoutingProtocol
	{
	public:
		static TypeId GetTypeId(void);
		UcmpRouting();
		virtual ~UcmpRouting();

		void SetIpv4(Ptr<Ipv4> ipv4);
		void InitializeRoutes();

		virtual void NotifyInterfaceUp(uint32_t) override {}
		virtual void NotifyInterfaceDown(uint32_t) override {}
		virtual void NotifyAddAddress(uint32_t, Ipv4InterfaceAddress) override {}
		virtual void NotifyRemoveAddress(uint32_t, Ipv4InterfaceAddress) override {}

	private:
		virtual Ptr<Ipv4Route> RouteOutput(Ptr<Packet> p,
										   const Ipv4Header &header,
										   Ptr<NetDevice> inputDevice,
										   Socket::SocketErrno &sockerr) override;
		virtual bool RouteInput(Ptr<const Packet> p,
								const Ipv4Header &header,
								Ptr<const NetDevice> idev,
								UnicastForwardCallback ucb,
								MulticastForwardCallback mcb,
								LocalDeliverCallback lcb,
								ErrorCallback ecb) override;
		virtual void DoDispose(void) override;

		void BuildTopologyGraph();
		void ComputeKShortestPaths(); // Yen
		std::vector<uint32_t> Dijkstra(uint32_t src, uint32_t dst,
									   const std::set<std::pair<uint32_t, uint32_t>> &removed) const;

		// Pick a pre-computed path
		uint32_t GetPathIndex(FlowId flowId, Ipv4Address src, Ipv4Address dst) const;
		// 5-tuple hash to flow ID
		FlowId MakeFlowId(Ptr<Packet> p,
						  const Ipv4Header &header) const;
		void ScheduleGammaUpdate();
		void UpdateGamma();

		Ptr<Ipv4> m_ipv4; // IPv4 instance
		double m_alpha;	  // [0,1] weighting between delay vs. 1/bw
		uint32_t m_k;	  // number of candidate paths per src/dst
		double m_gamma;	  // new: weight for ECN penalty term
		

		// Adjacency list for node ID → outgoing links
		std::unordered_map<uint32_t, std::vector<Link>> m_graph;
		// Node to Primary IP address
		std::unordered_map<uint32_t, Ipv4Address> m_nodeIp;
		std::map<std::pair<uint32_t, uint32_t>, double> m_ecnMetric;
		std::map<std::pair<Ipv4Address, Ipv4Address>, std::vector<std::vector<uint32_t>>> m_paths;
		std::unordered_map<Ipv4Address, uint32_t, Ipv4AddrHash, Ipv4AddrEq> m_ipToNode;
	};

}

#endif
