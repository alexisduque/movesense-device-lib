#pragma once

#include <whiteboard/LaunchableModule.h>
#include <whiteboard/ResourceClient.h>

#include "hrFilter.h"

#include "BleStandardHRS.hpp"

class BleServicesApp FINAL : 
    private wb::ResourceClient, 
    public wb::LaunchableModule
{

public:
    /** Name of this class. Used in StartupProvider list. */
    static const char* const LAUNCHABLE_NAME;
    static BleServicesApp* spInstance;

    BleServicesApp();
    ~BleServicesApp();

private:
    void setBleAdvPacket();
    void hrsNotificationChanged(bool enabled);
    void configAccel();

protected:
    whiteboard::TimerId mTimer;

    uint32_t mCounter;
    bool mHrsEnabled;
    bool mBLEConnected;
    bool mLeadsConnected;
    uint8_t mDataLoggerState;
    HrFilterState mHrfilterState;
    uint16_t mPreFilterHR;

    /** @see whiteboard::ILaunchableModule::initModule */
    virtual bool initModule() OVERRIDE;

    /** @see whiteboard::ILaunchableModule::deinitModule */
    virtual void deinitModule() OVERRIDE;

    /** @see whiteboard::ILaunchableModule::startModule */
    virtual bool startModule() OVERRIDE;

    /** @see whiteboard::ILaunchableModule::stopModule */
    virtual void stopModule() OVERRIDE;

    /**
    *	Callback for resource notifications.
    *   Note that this function will not be called for notifications that are
    *   of types WB_RESOURCE_NOTIFICATION_TYPE_INSERT or WB_RESOURCE_NOTIFICATION_TYPE_DELETE,
    *   just for notifications that are of type WB_RESOURCE_NOTIFICATION_TYPE_UPDATE.
    *
    *	@param resourceId Resource id associated with the update
    *	@param value Current value of the resource
    */
    virtual void
    onNotify(whiteboard::ResourceId resourceId, const whiteboard::Value& value, const whiteboard::ParameterList& parameters);

    virtual void onPutResult(whiteboard::RequestId requestId,
                             whiteboard::ResourceId resourceId,
                             whiteboard::Result resultCode,
                             const whiteboard::Value& result) OVERRIDE;

    virtual void onGetResult(whiteboard::RequestId requestId,
                             whiteboard::ResourceId resourceId,
                             whiteboard::Result resultCode,
                             const whiteboard::Value& result) OVERRIDE;

    /**
    *	Prepare to shutdown and set timer
    */
    void startShutdownTimer();
    void stopShutdownTimer();

    /**
    *	Timer callback.
    *
    *	@param timerId Id of timer that triggered
    */
    virtual void onTimer(whiteboard::TimerId timerId) OVERRIDE;

    /**
    *	DPS callback - events handler for HRS BLE events
    */
    void dpcHandler();
};
