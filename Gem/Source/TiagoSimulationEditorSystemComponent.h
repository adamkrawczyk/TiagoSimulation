
#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <TiagoSimulationSystemComponent.h>

namespace TiagoSimulation
{
    /// System component for TiagoSimulation editor
    class TiagoSimulationEditorSystemComponent
        : public TiagoSimulationSystemComponent
        , protected AzToolsFramework::EditorEvents::Bus::Handler
    {
        using BaseSystemComponent = TiagoSimulationSystemComponent;
    public:
        AZ_COMPONENT(TiagoSimulationEditorSystemComponent, "{EE14538C-1492-4A93-8409-10AA1ACAE12E}", BaseSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        TiagoSimulationEditorSystemComponent();
        ~TiagoSimulationEditorSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Activate() override;
        void Deactivate() override;
    };
} // namespace TiagoSimulation
