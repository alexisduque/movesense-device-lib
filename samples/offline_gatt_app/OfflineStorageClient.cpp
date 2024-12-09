#include "movesense.h"

#include "OfflineStorageClient.h"
#include "common/core/debug.h"

#include "component_led/resources.h"
#include "component_max3000x/resources.h"
#include "mem_datalogger/resources.h"
#include "ui_ind/resources.h"
#include "mem_datalogger/resources.h"
#include "mem_logbook/resources.h"
#include "comm_ble/resources.h"

constexpr wb::ExecutionContextId MY_EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;
const char* const OfflineStorageClient::LAUNCHABLE_NAME = "OfflineStg";

constexpr size_t RECONNECT_DATALOGGER_STOP_TIMEOUT = 1000; // 1s timeout for stopping logging

//static
bool OfflineStorageClient::s_bleConnected = false;
bool OfflineStorageClient::s_isLogging = false;

bool OfflineStorageClient::IsBLEConnected()
{
    return OfflineStorageClient::s_bleConnected;
}

bool OfflineStorageClient::IsLogging()
{
    return OfflineStorageClient::s_isLogging;
}

OfflineStorageClient::OfflineStorageClient():
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), MY_EXECUTION_CONTEXT),
    LaunchableModule(LAUNCHABLE_NAME, MY_EXECUTION_CONTEXT)
{
}

OfflineStorageClient::~OfflineStorageClient()
{
}

bool OfflineStorageClient::initModule()
{
    mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void OfflineStorageClient::deinitModule()
{
    mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool OfflineStorageClient::startModule()
{
    mModuleState = WB_RES::ModuleStateValues::STARTED;
    asyncSubscribe(WB_RES::LOCAL::COMM_BLE_PEERS());
    return true;
}

void OfflineStorageClient::stopModule()
{
    asyncUnsubscribe(WB_RES::LOCAL::COMM_BLE_PEERS());
    mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void OfflineStorageClient::onTimer(whiteboard::TimerId timerId)
{
    // stop datalogger after reconnect timeout
    DEBUGLOG("stop datalogger after reconnect timeout");
    mLoggingStartRequested = false;
    asyncPut(WB_RES::LOCAL::MEM_DATALOGGER_STATE(), AsyncRequestOptions::ForceAsync, WB_RES::DataLoggerStateValues::DATALOGGER_READY);
}

void OfflineStorageClient::onGetResult(whiteboard::RequestId requestId,
                              whiteboard::ResourceId resourceId,
                              whiteboard::Result resultCode,
                              const whiteboard::Value& result)
{
    if (resultCode != wb::HTTP_CODE_OK)
    {
        DEBUGLOG("NON-OK GET Result: %d for localResourceId: ", resultCode, resourceId.localResourceId);
    }

    if (s_bleConnected == false)
    {
        // Handle case where BLE has been disconnected
        switch(resourceId.localResourceId)
        {
            case WB_RES::LOCAL::MEM_DATALOGGER_STATE::LID:
            {
                DEBUGLOG("onGetResult: MEM_DATALOGGER_STATE");
                const auto dlState = result.convertTo<WB_RES::DataLoggerState>();
                if (dlState == WB_RES::DataLoggerStateValues::DATALOGGER_READY)
                {
                    DEBUGLOG("DATALOGGER ready, checking config.");
                    // continue prep for starting datalogger if it's not yet running
                    asyncGet(WB_RES::LOCAL::MEM_DATALOGGER_CONFIG(), AsyncRequestOptions::ForceAsync);
                }
                else
                {
                    DEBUGLOG("DATALOGGER already running, wont start");
                }

                break;
            }
            case WB_RES::LOCAL::MEM_DATALOGGER_CONFIG::LID:
            {
                DEBUGLOG("onGetResult: MEM_DATALOGGER_CONFIG");
                const auto dlConfig = result.convertTo<WB_RES::DataLoggerConfig>();

                // If datalogger has configured paths (clients responsibility), start logging a new log
                if (dlConfig.dataEntries.dataEntry.size())
                {
                    DEBUGLOG("%d entries in config, starting logging.", dlConfig.dataEntries.dataEntry.size());
                    asyncPost(WB_RES::LOCAL::MEM_LOGBOOK_ENTRIES());
                    mLoggingStartRequested = true;
                    asyncPut(WB_RES::LOCAL::MEM_DATALOGGER_STATE(), AsyncRequestOptions::ForceAsync, WB_RES::DataLoggerStateValues::DATALOGGER_LOGGING);
                }
                else
                {
                    DEBUGLOG("No entries in config, wont start logging.");
                }
                break;
            }
        }
    }
    else // (s_bleConnected == true)
    {
        // Handle case where BLE has been re-connected
        switch(resourceId.localResourceId)
        {
            case WB_RES::LOCAL::MEM_DATALOGGER_STATE::LID:
            {
                DEBUGLOG("onGetResult: MEM_DATALOGGER_STATE. connected.");
                const auto dlState = result.convertTo<WB_RES::DataLoggerState>();
                if (dlState == WB_RES::DataLoggerStateValues::DATALOGGER_LOGGING)
                {
                    DEBUGLOG("DATALOGGER was running when reconnecting, stopping logging after timeout");
                    startTimer(RECONNECT_DATALOGGER_STOP_TIMEOUT);
                }

                break;
            }
        }
    }
}

void OfflineStorageClient::onPutResult(whiteboard::RequestId requestId,
                              whiteboard::ResourceId resourceId,
                              whiteboard::Result resultCode,
                              const whiteboard::Value& result)
{
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::MEM_DATALOGGER_STATE::LID:
        {
            if (resultCode == 200)
            {
                s_isLogging = mLoggingStartRequested;
            }
            break;
        }
    }
}

void OfflineStorageClient::handleBleConnected()
{
    s_bleConnected = true;
    DEBUGLOG("handleBleConnected()");

    asyncGet(WB_RES::LOCAL::MEM_DATALOGGER_STATE(), AsyncRequestOptions::ForceAsync); // Continue in onGetResp...
}

void OfflineStorageClient::handleBleDisconnected()
{
    s_bleConnected = false;
    DEBUGLOG("handleBleDisconnected()");

    asyncGet(WB_RES::LOCAL::MEM_DATALOGGER_STATE(), AsyncRequestOptions::ForceAsync); // Continue in onGetResp...
}

void OfflineStorageClient::onNotify(wb::ResourceId resourceId,
                           const wb::Value& value,
                           const wb::ParameterList& parameters)
{
    switch (resourceId.localResourceId)
    {
    // WB routing table notification
    case WB_RES::LOCAL::COMM_BLE_PEERS::LID:
    {
        uint8_t peerState = value.convertTo<const WB_RES::PeerChange&>().state;
        DEBUGLOG("COMM_BLE_PEERS: peerState: %d", peerState);

        // if there is a new BLE connection
        if (peerState == WB_RES::PeerStateValues::CONNECTED)
        {
            handleBleConnected();
        }
        // if BLE connection lost, deal it here
        else if (peerState == WB_RES::PeerStateValues::DISCONNECTED)
        {
            handleBleDisconnected();
        }

        break;
    }
    }
}
