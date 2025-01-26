#include "movesense.h"

#include <meas_acc/resources.h>

#include "BleServicesApp.hpp"
#include "common/core/debug.h"
#include "common/compiler/pack.h"

#include "component_led/resources.h"
#include "ui_ind/resources.h"
#include "comm_ble/resources.h"
#include "comm_ble_hrs/resources.h"
#include "mem_datalogger/resources.h"
#include "component_max3000x/resources.h"
#include "system_mode/resources.h"
#include "system_states/resources.h"

#include "modules-resources/resources.h"

#include "whiteboard/builtinTypes/UnknownStructure.h"
#include "wb-resources/resources.h"

#include <float.h>
#include <math.h>

const char* const BleServicesApp::LAUNCHABLE_NAME = "BleServicesApp";

// Time between wake-up and going to power-off mode
#define AVAILABILITY_TIME 180000

// Time between turn on AFE wake circuit to power off
// (must be LED_BLINKING_PERIOD multiple)
#define WAKE_PREPARATION_TIME 5000

// LED blinking period in adertsing mode
#define LED_BLINKING_PERIOD 5000

// Declaration missing from hrFilter.h
extern "C" {
    void ibiBuffer_initS(HrFilterState* pTLab);
}

BleServicesApp::BleServicesApp()
    : ResourceClient(WBDEBUG_NAME(__FUNCTION__), WB_EXEC_CTX_APPLICATION),
      LaunchableModule(LAUNCHABLE_NAME, WB_EXEC_CTX_APPLICATION),
      mTimer(whiteboard::ID_INVALID_TIMER),
      mHrsEnabled(false),
      mBLEConnected(false),
      mPreFilterHR(0),
      mLeadsConnected(false),
      mDataLoggerState(WB_RES::DataLoggerStateValues::DATALOGGER_INVALID),
      mCounter(0)
{
}

BleServicesApp::~BleServicesApp()
{
}

void BleServicesApp::hrsNotificationChanged(bool enabled)
{
    if (enabled == mHrsEnabled) return;

    if (enabled)
    {
        stopShutdownTimer();
        // Initialize HR filtering
        hrFilter_initS(&mHrfilterState);
        ibiBuffer_initS(&mHrfilterState);
        
        mPreFilterHR = 0;
        // Send initial dummy data (HR=72).
        // This helps Spartan & Suunto9 to use belt instead of OHR
        WB_RES::HRSData dummy;
        dummy.hr = 72;
        asyncPost(WB_RES::LOCAL::COMM_BLE_HRS(), AsyncRequestOptions::Empty, dummy);

        // Subscribe ECGRR
        asyncSubscribe(WB_RES::LOCAL::ALGO_ECGRR());
    }
    else
    {
        asyncUnsubscribe(WB_RES::LOCAL::ALGO_ECGRR());
        if (!mBLEConnected) startShutdownTimer();
    }

    mHrsEnabled = enabled;
}

bool BleServicesApp::initModule()
{
    mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void BleServicesApp::deinitModule()
{
    mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

void BleServicesApp::onGetResult(whiteboard::RequestId requestId,
                                 whiteboard::ResourceId resourceId,
                                 whiteboard::Result resultCode,
                                 const whiteboard::Value& result)
{
    debugOut(true, "BleServicesApp::onGetResult: %u, result: %u", resourceId.localResourceId, resultCode);
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::MEM_DATALOGGER_STATE::LID:
        {
            WB_RES::DataLoggerState dlState = result.convertTo<WB_RES::DataLoggerState>();
            mDataLoggerState = dlState;
            break;
        }
    }
}

void BleServicesApp::onPutResult(whiteboard::RequestId requestId,
                                 whiteboard::ResourceId resourceId,
                                 whiteboard::Result resultCode,
                                 const whiteboard::Value& result)
{
    debugOut(true, "BleServicesApp::onPutResult: %u, result: %u", resourceId.localResourceId, resultCode);
}

/** @see whiteboard::ILaunchableModule::startModule */
bool BleServicesApp::startModule()
{
    mModuleState = WB_RES::ModuleStateValues::STARTED;

    asyncSubscribe(WB_RES::LOCAL::SYSTEM_STATES_STATEID(), AsyncRequestOptions::Empty, WB_RES::StateIdValues::CONNECTOR);

    startShutdownTimer();

    // Subscribe to BLE peers list changes
    asyncSubscribe(WB_RES::LOCAL::COMM_BLE_PEERS());

    // Subscribe to HRS state notifications
    asyncSubscribe(WB_RES::LOCAL::COMM_BLE_HRS());

    configAccel();

    return true;
}

/** @see whiteboard::ILaunchableModule::startModule */
void BleServicesApp::stopModule()
{
    // un-Subscribe to HRS state notifications
    asyncUnsubscribe(WB_RES::LOCAL::COMM_BLE_HRS());

    // Unsubscribe BLE peers list changes
    asyncUnsubscribe(WB_RES::LOCAL::COMM_BLE_PEERS());

    // Unsubscribe lead state
    asyncUnsubscribe(WB_RES::LOCAL::SYSTEM_STATES_STATEID(), AsyncRequestOptions::Empty, WB_RES::StateIdValues::CONNECTOR);

    // Stop LED timer
    stopTimer(mTimer);
    mTimer = whiteboard::ID_INVALID_TIMER;
}

void BleServicesApp::onNotify(whiteboard::ResourceId resourceId,
                              const whiteboard::Value& value,
                              const whiteboard::ParameterList& parameters)
{
    // Heart rate notification
    switch (resourceId.localResourceId)
    {
    case WB_RES::LOCAL::SYSTEM_STATES_STATEID::LID:
    {
        WB_RES::StateChange stateChange = value.convertTo<WB_RES::StateChange>(); 
        if (stateChange.stateId == WB_RES::StateIdValues::CONNECTOR)
        {
            DEBUGLOG("Lead state updated. newState: %d", stateChange.newState);
            mLeadsConnected = stateChange.newState;
        } 
        break;
    }
    case WB_RES::LOCAL::ALGO_ECGRR::LID:
    {
        // Get average heart rate data
        const WB_RES::ECGRRData& rrdata = value.convertTo<const WB_RES::ECGRRData&>();
        uint16_t rrInMs = rrdata.rR;

        // Update HR filter
        hrFilter_updateWithNewRR(&mHrfilterState, rrInMs);
        DEBUGLOG("BleServicesApp.HrfilterState. rrInMs: %d ms, bpm: %d", rrInMs, mHrfilterState.hr_bpm);

        uint16_t hr = 0;
        if (mHrfilterState.hrSmooth != 0)
        {
            // Hr smooth is in 10x bpm
            hr = (mHrfilterState.hrSmooth + 5) / 10;
        }
        else
        {
            // If RR valid, give IIR averaged hr before actual HR filter kicks in
            if (rrInMs >= 250 && rrInMs < 3000)
            {
                hr = static_cast<uint16_t>(60000 / rrInMs);
                if (mPreFilterHR > 0)
                {
                    hr = (7 * mPreFilterHR + hr) / 8;
                }
                mPreFilterHR = static_cast<uint16_t>(hr);
                DEBUGLOG("mPreFilterHR: %u bpm", mPreFilterHR);
            }
        }

        DEBUGLOG("ECGRR update: rr: %d", rrInMs);

        // Forward hr + rr data to HRS module, if it was valid
        if (hr > 0)
        {
            WB_RES::HRSData hrsData;
            hrsData.hr = hr;
            hrsData.rr = wb::MakeArray<uint16_t>(&rrInMs, 1);

            asyncPost(WB_RES::LOCAL::COMM_BLE_HRS(), AsyncRequestOptions::Empty, hrsData);
        }

        break;
    }

    // BLE connection notification
    case WB_RES::LOCAL::COMM_BLE_PEERS::LID:
    {
        // Get whiteborad routing table notification
        uint8_t peerState = value.convertTo<const WB_RES::PeerChange&>().state;
        DEBUGLOG("COMM_BLE_PEERS: peerState: %d", peerState);

        // if there is BLE connection, stop timer
        if (peerState == WB_RES::PeerStateValues::CONNECTED)
        {
            stopShutdownTimer();
            mBLEConnected = true;
            return;
        }

        // if BLE connection lost, prepare to shutdown
        if (peerState == WB_RES::PeerStateValues::DISCONNECTED)
        {
            mBLEConnected = false;
            // If HRS was enabled, unsubscribe from ECGRR
            if (mHrsEnabled)
            {
                mHrsEnabled = false;
                asyncUnsubscribe(WB_RES::LOCAL::ALGO_ECGRR());
            }
            startShutdownTimer();
        }

        break;
    }

    // BLE connection notification
    case WB_RES::LOCAL::COMM_BLE_HRS::LID:
        {
            // Get whiteborad routing table notification
            const bool hrsNotifEnabled = value.convertTo<const WB_RES::HRSState&>().notificationEnabled;
            DEBUGLOG("COMM_BLE_HRS: hrsNotifEnabled: %d", hrsNotifEnabled);

            hrsNotificationChanged(hrsNotifEnabled);
        }
        break;
    }
}

void BleServicesApp::startShutdownTimer()
{
    DEBUGLOG("Start shutdown timer");

    if (mTimer != whiteboard::ID_INVALID_TIMER) stopTimer(mTimer);

    // Start timer
    mTimer = startTimer(LED_BLINKING_PERIOD, true);

    // Reset timeout counter
    mCounter = 0;
}

void BleServicesApp::stopShutdownTimer()
{
    DEBUGLOG("Stop shutdown timer");

    if (mTimer == whiteboard::ID_INVALID_TIMER) return;

    stopTimer(mTimer);
    mTimer = whiteboard::ID_INVALID_TIMER;
}

void BleServicesApp::onTimer(whiteboard::TimerId timerId)
{

    // Check leads connection and datalogger state. if either is on, reset counter
    // NOTE: Trust that this module and datalogger are in same thread so the call is synchronous
    STATIC_VERIFY(WB_EXEC_CTX_APPLICATION == WB_RES::LOCAL::MEM_DATALOGGER_STATE::EXECUTION_CONTEXT, DataLogger_must_be_application_thread);
    asyncGet(WB_RES::LOCAL::MEM_DATALOGGER_STATE());
    if (mLeadsConnected || mDataLoggerState == WB_RES::DataLoggerStateValues::DATALOGGER_LOGGING)
    {
        DEBUGLOG("leads connected [%d] or datalogger running [%d]. postponing shutdown", mLeadsConnected, mDataLoggerState);
        mCounter = 0;
        return;
    }

    // Ok, no reason to stay awake. keep incrementing and blinking
    mCounter += LED_BLINKING_PERIOD;

    if (mCounter < AVAILABILITY_TIME)
    {
        asyncPut(WB_RES::LOCAL::UI_IND_VISUAL(), AsyncRequestOptions::Empty,
                 WB_RES::VisualIndTypeValues::SHORT_VISUAL_INDICATION);
        return;
    }

    if (mCounter < AVAILABILITY_TIME + WAKE_PREPARATION_TIME)
    {
        // Prepare AFE to wake-up mode
        asyncPut(WB_RES::LOCAL::COMPONENT_MAX3000X_WAKEUP(), AsyncRequestOptions(NULL, 0, true), (uint8_t)1);
        return;
    }

    // Make PUT request to switch LED on
    asyncPut(WB_RES::LOCAL::COMPONENT_LED(), AsyncRequestOptions::Empty, true);

    // Make PUT request to enter power off mode
    asyncPut(WB_RES::LOCAL::SYSTEM_MODE(),
             AsyncRequestOptions(NULL, 0, true), // Force async
             WB_RES::SystemModeValues::FULLPOWEROFF);
}

void BleServicesApp::configAccel()
{
    // Set Accelerometer FSR
    WB_RES::AccConfig accConfig;
    accConfig.gRange = 16; // 2, 4, 8, 16

    asyncPut(WB_RES::LOCAL::MEAS_ACC_CONFIG(), AsyncRequestOptions::Empty, accConfig);
}