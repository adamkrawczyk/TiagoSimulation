
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include "TiagoSimulationSystemComponent.h"
#include "TiagoSimulationSampleComponent.h"

namespace TiagoSimulation
{
    class TiagoSimulationModule
        : public AZ::Module
    {
    public:
        AZ_RTTI(TiagoSimulationModule, "{14CB3522-2174-4175-AEFA-23B4A8E1A380}", AZ::Module);
        AZ_CLASS_ALLOCATOR(TiagoSimulationModule, AZ::SystemAllocator);

        TiagoSimulationModule()
            : AZ::Module()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            m_descriptors.insert(m_descriptors.end(), {
                TiagoSimulationSystemComponent::CreateDescriptor(),
                TiagoSimulationSampleComponent::CreateDescriptor(),
            });
        }

        /**
         * Add required SystemComponents to the SystemEntity.
         */
        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<TiagoSimulationSystemComponent>(),
            };
        }
    };
}// namespace TiagoSimulation

AZ_DECLARE_MODULE_CLASS(Gem_TiagoSimulation, TiagoSimulation::TiagoSimulationModule)
