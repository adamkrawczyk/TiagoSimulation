/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace TiagoSimulation
{
    class TiagoSimulationRequests
    {
    public:
        AZ_RTTI(TiagoSimulationRequests, "{6DB46FD8-625F-4120-AFE1-930B48F77AA1}");
        virtual ~TiagoSimulationRequests() = default;
        // Put your public methods here
    };

    class TiagoSimulationBusTraits
        : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using TiagoSimulationRequestBus = AZ::EBus<TiagoSimulationRequests, TiagoSimulationBusTraits>;
    using TiagoSimulationInterface = AZ::Interface<TiagoSimulationRequests>;

} // namespace TiagoSimulation
