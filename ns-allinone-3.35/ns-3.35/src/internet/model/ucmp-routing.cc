#include "ns3/address.h"
#include "ns3/log.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/ipv4-route.h"
#include "ns3/ipv4-header.h"
#include "ns3/ipv4.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/double.h"				   // for DoubleValue
#include "ns3/uinteger.h"			   // for UintegerValue
#include "ns3/attribute-helper.h"	   // for MakeDoubleAccessor, MakeUintegerAccessor, etc.
#include "ns3/output-stream-wrapper.h" // for PrintRoutingTable
#include "ns3/simulator.h"
#include "ns3/node.h"
#include "ns3/ucmp-routing.h"
#include <queue>
#include <limits>
#include <algorithm>

namespace ns3
{

	NS_LOG_COMPONENT_DEFINE("UcmpRouting");
	NS_OBJECT_ENSURE_REGISTERED(UcmpRouting);

	TypeId
	UcmpRouting::GetTypeId(void)
	{
		static TypeId tid = TypeId("ns3::UcmpRouting")
								.SetParent<Ipv4RoutingProtocol>()
								.SetGroupName("Internet")
								.AddConstructor<UcmpRouting>()
								.AddAttribute("Alpha",
											  "Weighting factor between delay and bandwidth",
											  DoubleValue(0.5),
											  MakeDoubleAccessor(&UcmpRouting::m_alpha),
											  MakeDoubleChecker<double>(0.0, 1.0))
								.AddAttribute("KPaths",
											  "Number of candidate paths per source-destination",
											  UintegerValue(4),
											  MakeUintegerAccessor(&UcmpRouting::m_k),
											  MakeUintegerChecker<uint32_t>())
								.AddAttribute("Gamma", "ECN‐penalty weight in the cost metric",
											  DoubleValue(0.0),
											  MakeDoubleAccessor(&UcmpRouting::m_gamma),
											  MakeDoubleChecker<double>(0.0));
		return tid;
	}

	UcmpRouting::UcmpRouting() : m_alpha(0.5), m_k(4), m_gamma(0.0) { ScheduleGammaUpdate(); }

	UcmpRouting::~UcmpRouting() {}

	void
	UcmpRouting::DoDispose(void)
	{
		m_graph.clear();
		m_nodeIp.clear();
		m_paths.clear();
		Ipv4RoutingProtocol::DoDispose();
	}

	void
	UcmpRouting::SetIpv4(Ptr<Ipv4> ipv4)
	{
		m_ipv4 = ipv4;
	}

	void
	UcmpRouting::ScheduleGammaUpdate()
	{
		Simulator::Schedule(Seconds(1.0), &UcmpRouting::UpdateGamma, this);
	}

	void
	UcmpRouting::UpdateGamma()
	{
		double sumMarks = 0.0;
		for (auto &kv : m_ecnMetric)
			sumMarks += kv.second;
		double avgMarks = sumMarks / m_ecnMetric.size();
		double target = 10.0;
		double Kp = 0.01;
		m_gamma = std::max(0.0, Kp * (avgMarks - target));
		Simulator::Schedule(Seconds(1.0), &UcmpRouting::UpdateGamma, this);
	}

	void
	UcmpRouting::InitializeRoutes()
	{
		NS_LOG_FUNCTION(this);
		m_graph.clear();
		m_paths.clear();
		m_nodeIp.clear();
		m_ipToNode.clear();

		for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
		{
			uint32_t nid = (*it)->GetId();
			Ptr<Ipv4> ipv4 = (*it)->GetObject<Ipv4>();
			if (!ipv4) continue;

			for (uint32_t iface = 0; iface < ipv4->GetNInterfaces(); ++iface)
			{
				for (uint32_t idx = 0; idx < ipv4->GetNAddresses(iface); ++idx)
				{
					Ipv4Address addr = ipv4->GetAddress(iface, idx).GetLocal();
					if (addr == Ipv4Address::GetLoopback()) continue;
					m_ipToNode[addr] = nid;
					if (!m_nodeIp.count(nid))
					{
						m_nodeIp[nid] = addr;
					}
				}
			}
		}
		BuildTopologyGraph();
		ComputeKShortestPaths();
	}

	void
	UcmpRouting::BuildTopologyGraph()
	{
		for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
		{
			Ptr<Node> node = *it;
			uint32_t u = (*it)->GetId();
			Ptr<Ipv4> ipv4 = (*it)->GetObject<Ipv4>();
			if (!ipv4) continue;

			for (uint32_t i = 1; i < ipv4->GetNInterfaces(); ++i) {
				for (uint32_t j = 0; j < ipv4->GetNAddresses(i); ++j) {
					Ipv4Address addr = ipv4->GetAddress(i, j).GetLocal();
					if (addr == Ipv4Address::GetLoopback()) continue;
					m_ipToNode[addr] = u;
					if (!m_nodeIp.count(u)) m_nodeIp[u] = addr;
				}
			}
			// build adjacency
			for (uint32_t i = 1; i < ipv4->GetNInterfaces(); ++i) {
				Ptr<NetDevice> dev = ipv4->GetNetDevice(i);
				Ptr<PointToPointNetDevice> ppd = ipv4->GetNetDevice(i)->GetObject<PointToPointNetDevice>();
				if (!ppd)
					continue;
				Ptr<PointToPointChannel> ch = ppd->GetChannel()->GetObject<PointToPointChannel>();
				if (!ch)
					continue;
				// get neighbor
				Ptr<NetDevice> nd0 = ch->GetDevice(0);
				Ptr<NetDevice> nd1 = ch->GetDevice(1);
				uint32_t v = (nd0 == ipv4->GetNetDevice(i) ? nd1 : nd0)->GetNode()->GetId();
				// Compute uniform cost
				TimeValue tv;
				ch->GetAttribute("Delay", tv);
				double d = tv.Get().GetSeconds();
				DataRateValue drv;
				ppd->GetAttribute("DataRate", drv);
				double bw = drv.Get().GetBitRate();
				double cost = m_alpha * d + (1.0 - m_alpha) * (1.0 / bw);
				m_graph[u].push_back({v, cost});
			}
		}
	}

	std::vector<uint32_t>
	UcmpRouting::Dijkstra(uint32_t src, uint32_t dst, const std::set<std::pair<uint32_t, uint32_t>> &removed) const {
		std::unordered_map<uint32_t, double> dist;
		std::unordered_map<uint32_t, uint32_t> prev;
		struct NodeCost
		{
			uint32_t node;
			double cost;
		};
		struct Cmp
		{
			bool operator()(NodeCost a, NodeCost b) const { return a.cost > b.cost; }
		};

		std::priority_queue<NodeCost, std::vector<NodeCost>, Cmp> pq;

		for (auto &kv : m_graph)
		{
			dist[kv.first] = std::numeric_limits<double>::infinity();
		}
		dist[src] = 0;
		pq.push(NodeCost{src, 0});
		while (!pq.empty())
		{
			auto nc = pq.top();
			pq.pop();
			if (nc.cost > dist[nc.node])
				continue;
			if (nc.node == dst)
				break;
			for (auto &link : m_graph.at(nc.node))
			{
				auto edge = std::make_pair(nc.node, link.node);
				if (removed.count(edge))
					continue;

				double ecnPen = 0.0;
				if (m_gamma >= 0.0)
				{
					auto it = m_ecnMetric.find(edge);
					if (it != m_ecnMetric.end())
					{
						ecnPen = m_gamma * it->second;
					}
				}

				double alt = nc.cost + link.cost + ecnPen;

				if (alt < dist[link.node])
				{
					dist[link.node] = alt;
					prev[link.node] = nc.node;
					pq.push(NodeCost{link.node, alt});
				}
			}
		}

		std::vector<uint32_t> path;
		if (!dist.count(dst) || dist[dst] == std::numeric_limits<double>::infinity())
			return path;
		for (uint32_t u = dst;; u = prev[u])
		{
			path.push_back(u);
			if (u == src)
				break;
		}
		std::reverse(path.begin(), path.end());
		return path;
	}

	void
	UcmpRouting::ComputeKShortestPaths()
	{
		for (auto &srcKv : m_nodeIp)
		{
			for (auto &dstKv : m_nodeIp)
			{
				uint32_t src = srcKv.first;
				uint32_t dst = dstKv.first;
				if (src == dst)
					continue;

				std::vector<std::vector<uint32_t>> paths;
				std::vector<uint32_t> first = Dijkstra(src, dst, {});
				if (first.empty())
					continue;
				paths.push_back(first);
				std::set<std::pair<std::vector<uint32_t>, std::set<std::pair<uint32_t, uint32_t>>>> candidates;
				// Yen's algorithm
				for (uint32_t k = 1; k < m_k; ++k)
				{
					for (size_t i = 0; i + 1 < paths[k - 1].size(); ++i)
					{
						uint32_t spurNode = paths[k - 1][i];
						std::vector<uint32_t> root(paths[k - 1].begin(), paths[k - 1].begin() + i + 1);
						std::set<std::pair<uint32_t, uint32_t>> removed;
						for (auto &p : paths)
						{
							if (p.size() > i && std::equal(root.begin(), root.end(), p.begin()))
							{
								removed.insert({p[i], p[i + 1]});
							}
						}
						auto spurPath = Dijkstra(spurNode, dst, removed);
						if (spurPath.empty())
							continue;
						std::vector<uint32_t> total = root;
						total.insert(total.end(), spurPath.begin() + 1, spurPath.end());
						double cost = 0;
						for (size_t e = 0; e + 1 < total.size(); ++e)
						{
							uint32_t u = total[e], v = total[e + 1];
							for (auto &lnk : m_graph.at(u))
							{
								if (lnk.node == v)
								{
									cost += lnk.cost;
									break;
								}
							}
						}
						candidates.insert({total, removed});
					}
					if (candidates.empty())
						break;
					auto best = *candidates.begin();
					paths.push_back(best.first);
					candidates.erase(candidates.begin());
				}
				std::vector<std::vector<uint32_t>> dummy;
				auto &out = m_paths[{m_nodeIp[src], m_nodeIp[dst]}];
				for (auto &p : paths)
				{
					out.push_back(p);
				}
				// NS_LOG_INFO("Computed " << paths.size() << " UCMP paths from "
				// 						<< m_nodeIp[src] << " to " << m_nodeIp[dst]);
				// for (size_t i = 0; i < paths.size(); ++i)
				// {
				// 	std::ostringstream oss;
				// 	for (uint32_t n : paths[i])
				// 		oss << n << " ";
				// 	NS_LOG_INFO("  Path " << i << ": " << oss.str());
				// }
			}
		}
	}

	FlowId
	UcmpRouting::MakeFlowId(Ptr<Packet> p,
							const Ipv4Header &header) const
	{
		uint32_t h = 0;
		h ^= header.GetSource().Get();
		h ^= header.GetDestination().Get() << 1;
		h ^= header.GetProtocol() << 2;
		h ^= header.GetIdentification() << 3;
		return h;
	}

	uint32_t
	UcmpRouting::GetPathIndex(FlowId flowId,
							  Ipv4Address src,
							  Ipv4Address dst) const
	{
		auto it = m_paths.find({src, dst});
		if (it == m_paths.end() || it->second.empty())
			return 0;
		return flowId % it->second.size();
	}

	Ptr<Ipv4Route>
	UcmpRouting::RouteOutput(Ptr<Packet> packet,
							 const Ipv4Header &header,
							 Ptr<NetDevice> inputDevice,
							 Socket::SocketErrno &sockerr)
	{
		uint32_t nodeId = m_ipv4->GetObject<Node>()->GetId();
		NS_LOG_INFO("[RouteOutput]   node=" << nodeId
											<< "   rawSrc=" << header.GetSource()
											<< "   dst=" << header.GetDestination()
											<< "   inDev=" << inputDevice);
		Ptr<NetDevice> actualInDev = inputDevice;
		if (actualInDev == nullptr)
		{
			actualInDev = m_ipv4->GetNetDevice(1);
			// NS_LOG_INFO("  Locally generated → using host dev@1 = " << actualInDev);
		}
		int32_t inIf = m_ipv4->GetInterfaceForDevice(actualInDev);
		if (inIf < 0)
		{
			sockerr = Socket::ERROR_NOROUTETOHOST;
			NS_LOG_ERROR("Cannot map device to interface");
			return nullptr;
		}
		Ipv4Address src = m_ipv4->GetAddress(inIf, 0).GetLocal();
		Ipv4Address dst = header.GetDestination(); // destination is always correct

		// NS_LOG_INFO("  Mapped src to " << src << " on if@" << inIf);
		auto sit = m_ipToNode.find(src);
		auto dit = m_ipToNode.find(dst);
		if (sit == m_ipToNode.end() || dit == m_ipToNode.end())
		{
			sockerr = Socket::ERROR_NOROUTETOHOST;
			// NS_LOG_INFO("Node mapping failed: src=" << src << " dst=" << dst);
			return nullptr;
		}
		uint32_t srcNode = sit->second;
		uint32_t dstNode = dit->second;

		FlowId fid = MakeFlowId(packet, header);
		// NS_LOG_INFO("FlowID=" << fid << " srcNode=" << srcNode
		// 					  << " dstNode=" << dstNode);
		Ipv4Address srcKey = m_nodeIp[srcNode];
		Ipv4Address dstKey = m_nodeIp[dstNode];

		NS_LOG_INFO(" Lookup key = {" << srcKey << "," << dstKey << "}");

		auto pit = m_paths.find({srcKey, dstKey});
		if (pit == m_paths.end() || pit->second.empty())
		{
			NS_LOG_WARN("  No UCMP path for {" << srcKey << "," << dstKey << "}");
			return nullptr;
		}

		NS_LOG_INFO("   m_paths " << srcKey << " to " << dstKey << " count = "
								   << pit->second.size());

		auto &nodePath = pit->second[GetPathIndex(fid, srcKey, dstKey)];

		auto pos = std::find(nodePath.begin(), nodePath.end(), srcNode);
		if (pos == nodePath.end() || pos + 1 == nodePath.end())
		{
			sockerr = Socket::ERROR_NOROUTETOHOST;
			// NS_LOG_INFO("Invalid position in path for node " << srcNode);
			return nullptr;
		}
		uint32_t nextNode = *(pos + 1);
		Ipv4Address nextHop = m_nodeIp[nextNode];

		int32_t outIf = -1;
		Ptr<NetDevice> outDev;
		for (uint32_t i = 1; i < m_ipv4->GetNInterfaces(); ++i)
		{
			auto dev = m_ipv4->GetNetDevice(i)
						   ->GetObject<PointToPointNetDevice>();
			if (!dev)
				continue;
			auto ch = dev->GetChannel()->GetObject<PointToPointChannel>();
			if (!ch)
				continue;
			Ptr<NetDevice> nb = (ch->GetDevice(0) == dev ? ch->GetDevice(1)
														 : ch->GetDevice(0));
			if (nb->GetNode()->GetId() == nextNode)
			{
				outIf = i;
				outDev = dev;
				break;
			}
		}
		if (outIf < 0)
		{
			sockerr = Socket::ERROR_NOROUTETOHOST;
			NS_LOG_INFO("No interface to next hop " << nextNode
													 << " (gateway=" << nextHop
													 << ") on node "
													 << m_ipv4->GetObject<Node>()->GetId());
			return nullptr;
		}

		NS_LOG_INFO("[  → UCMP]  " << "   nextHop=" << nextHop
								   << "   outIf=" << outIf
								   << "   dev=" << outDev);

		Ptr<Ipv4Route> route = Create<Ipv4Route>();
		route->SetDestination(dst);
		route->SetSource(src);
		route->SetGateway(nextHop);
		route->SetOutputDevice(outDev);

		sockerr = Socket::ERROR_NOTERROR;
		// NS_LOG_INFO("Forwarding flow " << fid
		// 							   << " via " << nextHop
		// 							   << " on if@" << outIf);
		return route;
	}

	bool
	UcmpRouting::RouteInput(Ptr<const Packet> packet,
							const Ipv4Header &header,
							Ptr<const NetDevice> idev,
							UnicastForwardCallback ucb,
							MulticastForwardCallback mcb,
							LocalDeliverCallback lcb,
							ErrorCallback ecb)
	{
		NS_LOG_INFO("[RouteInput]    node=" << m_ipv4->GetObject<Node>()->GetId()
											<< "   dst=" << header.GetDestination()
											<< "   idev=" << idev);
		Ptr<PointToPointNetDevice> ppd = idev->GetObject<PointToPointNetDevice>();
		if (ppd)
		{
			Ptr<PointToPointChannel> ch = ppd->GetChannel()->GetObject<PointToPointChannel>();
			if (ch)
			{
				Ptr<NetDevice> peer = (ch->GetDevice(0) == idev
										   ? ch->GetDevice(1)
										   : ch->GetDevice(0));
				uint32_t peerNode = peer->GetNode()->GetId();
				NS_LOG_INFO("    Packet came in from ToR node " << peerNode);
			}
		}
		if (header.GetDestination() ==
			m_ipv4->GetAddress(m_ipv4->GetInterfaceForDevice(idev), 0).GetLocal())
		{
			uint32_t iif = m_ipv4->GetInterfaceForDevice(idev);
			NS_LOG_INFO("[RouteInput]    LOCAL DELIVER on if at " << iif);
			lcb(packet, header, iif);
			return true;
		}

		Socket::SocketErrno sockerr;
		Ptr<Packet> pkt = packet->Copy();
		Ptr<NetDevice> inDev = const_cast<NetDevice *>(idev.operator->());

		NS_LOG_INFO(" [RouteInput]    calling RouteOutput from node="
					<< m_ipv4->GetObject<Node>()->GetId());
		Ptr<Ipv4Route> route = RouteOutput(pkt, header, inDev, sockerr);
		if (route)
		{
			if (header.GetEcn() != Ipv4Header::NotEct && m_gamma >= 0.0)
			{
				uint32_t thisNode = m_ipv4->GetObject<Node>()->GetId();
				Ptr<NetDevice> outDev = route->GetOutputDevice();
				Ptr<PointToPointNetDevice> ppdOut = outDev->GetObject<PointToPointNetDevice>();
				if (ppdOut)
				{
					Ptr<PointToPointChannel> outCh =
						ppdOut->GetChannel()->GetObject<PointToPointChannel>();
					if (outCh)
					{
						// get neighbor on the other end
						Ptr<NetDevice> peerDev =
							(outCh->GetDevice(0) == outDev
								 ? outCh->GetDevice(1)
								 : outCh->GetDevice(0));
						uint32_t nextNode = peerDev->GetNode()->GetId();

						auto edge = std::make_pair(thisNode, nextNode);
						m_ecnMetric[edge] += 1.0;
					}
				}
			}
			// --------------------------------------------------------------------

			NS_LOG_INFO(" [RouteInput]    got back a Route: "
						<< " nextHop=" << route->GetGateway()
						<< " outIf=" << m_ipv4->GetInterfaceForDevice(route->GetOutputDevice()));
			ucb(route, pkt, header);
			return true;
		}

		return false;
	}

} // namespace ns3