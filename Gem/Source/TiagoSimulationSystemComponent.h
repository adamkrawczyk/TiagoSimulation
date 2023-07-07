/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>

#include <TiagoSimulation/TiagoSimulationBus.h>

namespace TiagoSimulation
{
    class TiagoSimulationSystemComponent
        : public AZ::Component
        , protected TiagoSimulationRequestBus::Handler
    {
    public:
        AZ_COMPONENT(TiagoSimulationSystemComponent, "{EBBD8D18-963C-452E-A18E-387FA75127B9}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        TiagoSimulationSystemComponent();
        ~TiagoSimulationSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // TiagoSimulationRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
