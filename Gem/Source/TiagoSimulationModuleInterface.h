
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>
#include <TiagoSimulationSystemComponent.h>

namespace TiagoSimulation
{
    class TiagoSimulationModuleInterface
        : public AZ::Module
    {
    public:
        AZ_RTTI(TiagoSimulationModuleInterface, "{0BC5938B-005A-4EE2-ABE0-CB4767718A50}", AZ::Module);
        AZ_CLASS_ALLOCATOR(TiagoSimulationModuleInterface, AZ::SystemAllocator, 0);

        TiagoSimulationModuleInterface()
        {
            // Push results of [MyComponent]::CreateDescriptor() into m_descriptors here.
            // Add ALL components descriptors associated with this gem to m_descriptors.
            // This will associate the AzTypeInfo information for the components with the the SerializeContext, BehaviorContext and EditContext.
            // This happens through the [MyComponent]::Reflect() function.
            m_descriptors.insert(m_descriptors.end(), {
                TiagoSimulationSystemComponent::CreateDescriptor(),
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
