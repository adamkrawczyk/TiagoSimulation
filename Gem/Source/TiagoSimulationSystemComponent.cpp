

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include "TiagoSimulationSystemComponent.h"

namespace TiagoSimulation
{
    void TiagoSimulationSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<TiagoSimulationSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<TiagoSimulationSystemComponent>("TiagoSimulation", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void TiagoSimulationSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("TiagoSimulationService"));
    }

    void TiagoSimulationSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("TiagoSimulationService"));
    }

    void TiagoSimulationSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void TiagoSimulationSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    TiagoSimulationSystemComponent::TiagoSimulationSystemComponent()
    {
        if (TiagoSimulationInterface::Get() == nullptr)
        {
            TiagoSimulationInterface::Register(this);
        }
    }

    TiagoSimulationSystemComponent::~TiagoSimulationSystemComponent()
    {
        if (TiagoSimulationInterface::Get() == this)
        {
            TiagoSimulationInterface::Unregister(this);
        }
    }

    void TiagoSimulationSystemComponent::Init()
    {
    }

    void TiagoSimulationSystemComponent::Activate()
    {
        TiagoSimulationRequestBus::Handler::BusConnect();
    }

    void TiagoSimulationSystemComponent::Deactivate()
    {
        TiagoSimulationRequestBus::Handler::BusDisconnect();
    }
}
