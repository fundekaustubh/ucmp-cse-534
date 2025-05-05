#ifndef NS3_UCMP_ROUTING_HELPER_H
#define NS3_UCMP_ROUTING_HELPER_H

#include "ns3/object.h"
#include "ns3/object-factory.h"
#include "ns3/node.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/attribute.h"
#include <string>
#include <cstdint>

namespace ns3
{

	class UcmpRoutingHelper : public Ipv4RoutingHelper, public Object
	{
	public:
		static TypeId GetTypeId(void);
		UcmpRoutingHelper();
		virtual ~UcmpRoutingHelper();

		void Set(std::string name, const AttributeValue &value);

		virtual Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override;
		virtual Ipv4RoutingHelper *Copy(void) const override;

	private:
		ObjectFactory m_factory;
		double m_alpha;
		uint32_t m_k;
	};

}

#endif