#include "ns3/ucmp-routing-helper.h"
#include "ns3/ucmp-routing.h"
#include "ns3/names.h"
#include "ns3/ipv4-list-routing.h"

namespace ns3
{

	NS_LOG_COMPONENT_DEFINE("UcmpRoutingHelper");
	NS_OBJECT_ENSURE_REGISTERED(UcmpRoutingHelper);

	TypeId
	UcmpRoutingHelper::GetTypeId(void)
	{
		static TypeId tid = TypeId("ns3::UcmpRoutingHelper")
								.SetParent<Object>()
								.SetGroupName("Internet")
								.AddConstructor<UcmpRoutingHelper>()
								.AddAttribute("Alpha",
											  "Bandwidth-delay factor",
											  DoubleValue(0.5),
											  MakeDoubleAccessor(&UcmpRoutingHelper::m_alpha),
											  MakeDoubleChecker<double>(0.0, 1.0))
								.AddAttribute("KPaths",
											  "Number of paths from source-destination",
											  UintegerValue(4),
											  MakeUintegerAccessor(&UcmpRoutingHelper::m_k),
											  MakeUintegerChecker<uint32_t>());
		return tid;
	}

	UcmpRoutingHelper::~UcmpRoutingHelper() {}

	Ipv4RoutingHelper *
	UcmpRoutingHelper::Copy(void) const
	{
		return new UcmpRoutingHelper(*this);
	}

	UcmpRoutingHelper::UcmpRoutingHelper() : m_alpha(0.5), m_k(4)
	{
		m_factory.SetTypeId("ns3::UcmpRouting");
		m_factory.Set("Alpha", DoubleValue(m_alpha));
		m_factory.Set("KPaths", UintegerValue(m_k));
	}

	void
	UcmpRoutingHelper::Set(std::string name, const AttributeValue &value)
	{
		m_factory.Set(name, value);
	}

	Ptr<Ipv4RoutingProtocol>
	UcmpRoutingHelper::Create(Ptr<Node> node) const
	{
		Ptr<UcmpRouting> routing = m_factory.Create<UcmpRouting>();
		routing->SetIpv4(node->GetObject<Ipv4>());
		return routing;
	}

}