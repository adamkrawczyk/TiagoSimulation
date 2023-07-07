
#include <TiagoSimulationModuleInterface.h>
#include "TiagoSimulationEditorSystemComponent.h"

#include "TiagoSimulationSampleComponent.h"
namespace TiagoSimulation
{
    class TiagoSimulationEditorModule
        : public TiagoSimulationModuleInterface
    {
    public:
        AZ_RTTI(TiagoSimulationEditorModule, "{14CB3522-2174-4175-AEFA-23B4A8E1A380}", TiagoSimulationModuleInterface);
        AZ_CLASS_ALLOCATOR(TiagoSimulationEditorModule, AZ::SystemAllocator, 0);

        TiagoSimulationEditorModule()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                TiagoSimulationEditorSystemComponent::CreateDescriptor(),
                TiagoSimulationSampleComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         * Non-SystemComponents should not be added here
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList {
                azrtti_typeid<TiagoSimulationEditorSystemComponent>(),
            };
        }
    };
}// namespace TiagoSimulation

AZ_DECLARE_MODULE_CLASS(Gem_TiagoSimulation, TiagoSimulation::TiagoSimulationEditorModule)
