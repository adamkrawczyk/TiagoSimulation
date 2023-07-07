
#include <AzCore/Serialization/SerializeContext.h>
#include "TiagoSimulationEditorSystemComponent.h"

namespace TiagoSimulation
{
    void TiagoSimulationEditorSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<TiagoSimulationEditorSystemComponent, TiagoSimulationSystemComponent>()
                ->Version(0);
        }
    }

    TiagoSimulationEditorSystemComponent::TiagoSimulationEditorSystemComponent() = default;

    TiagoSimulationEditorSystemComponent::~TiagoSimulationEditorSystemComponent() = default;

    void TiagoSimulationEditorSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        BaseSystemComponent::GetProvidedServices(provided);
        provided.push_back(AZ_CRC_CE("TiagoSimulationEditorService"));
    }

    void TiagoSimulationEditorSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        BaseSystemComponent::GetIncompatibleServices(incompatible);
        incompatible.push_back(AZ_CRC_CE("TiagoSimulationEditorService"));
    }

    void TiagoSimulationEditorSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        BaseSystemComponent::GetRequiredServices(required);
    }

    void TiagoSimulationEditorSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        BaseSystemComponent::GetDependentServices(dependent);
    }

    void TiagoSimulationEditorSystemComponent::Activate()
    {
        TiagoSimulationSystemComponent::Activate();
        AzToolsFramework::EditorEvents::Bus::Handler::BusConnect();
    }

    void TiagoSimulationEditorSystemComponent::Deactivate()
    {
        AzToolsFramework::EditorEvents::Bus::Handler::BusDisconnect();
        TiagoSimulationSystemComponent::Deactivate();
    }

} // namespace TiagoSimulation
