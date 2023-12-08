#include "movesense.h"

#include "whiteboard/builtinTypes/ByteStream.h"

#include "GATTSensorDataClient.h"
#include "OfflineStorageClient.h"

#include "common/core/debug.h"
#include "platform/bsp/rtc.hpp"
#include "sbem/Sbem.hpp"
#include "oswrapper/thread.h"

#include "comm_ble_gattsvc/resources.h"
#include "comm_ble/resources.h"
#include "meas_temp/resources.h"
#include "mem_datalogger/resources.h"
#include "mem_logbook/resources.h"

// Functions for serializing binary data
#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"
#include "meas_ecg/resources.h"
#include "meas_hr/resources.h"
#include "movesense_time/resources.h"
#include "sbem-code/sbem_definitions.h"

const char* const GATTSensorDataClient::LAUNCHABLE_NAME = "OfflineGatt";
constexpr wb::ExecutionContextId MY_EXECUTION_CONTEXT = WB_EXEC_CTX_APPLICATION;

// UUID: 34802252-7185-4d5d-b431-630e7050e8f0
constexpr uint8_t SENSOR_DATASERVICE_UUID[] = { 0xf0, 0xe8, 0x50, 0x70, 0x0e, 0x63, 0x31, 0xb4, 0x5d, 0x4d, 0x85, 0x71, 0x52, 0x22, 0x80, 0x34 };
constexpr uint8_t COMMAND_CHAR_UUID[] = { 0xf0, 0xe8, 0x50, 0x70, 0x0e, 0x63, 0x31, 0xb4, 0x5d, 0x4d, 0x85, 0x71, 0x01, 0x00, 0x80, 0x34 };
constexpr uint16_t commandCharUUID16 = 0x0001;
constexpr uint8_t DATA_CHAR_UUID[] = { 0xf0, 0xe8, 0x50, 0x70, 0x0e, 0x63, 0x31, 0xb4, 0x5d, 0x4d, 0x85, 0x71, 0x02, 0x00, 0x80, 0x34 };
constexpr uint16_t dataCharUUID16 = 0x0002;

constexpr uint32_t UNSUBSCRIBE_TIMEOUT = 200;

GATTSensorDataClient::GATTSensorDataClient():
    ResourceClient(WBDEBUG_NAME(__FUNCTION__), MY_EXECUTION_CONTEXT),
    LaunchableModule(LAUNCHABLE_NAME, MY_EXECUTION_CONTEXT),
    mCommandCharResource(wb::ID_INVALID_RESOURCE),
    mDataCharResource(wb::ID_INVALID_RESOURCE),
    mNotificationsEnabled(false),
    mSensorSvcHandle(0),
    mCommandCharHandle(0),
    mDataCharHandle(0),
    mLogToSend(0),
    mSendBufferLength(0),
    mLogSendReference(0),
    mSendBuffer{0}
{
}

GATTSensorDataClient::~GATTSensorDataClient()
{
}

bool GATTSensorDataClient::initModule()
{
    mModuleState = WB_RES::ModuleStateValues::INITIALIZED;
    return true;
}

void GATTSensorDataClient::deinitModule()
{
    mModuleState = WB_RES::ModuleStateValues::UNINITIALIZED;
}

bool GATTSensorDataClient::startModule()
{
    mModuleState = WB_RES::ModuleStateValues::STARTED;

    // Clear subscription table
    for (size_t i=0; i<MAX_DATASUB_COUNT; i++)
    {
        mDataSubs[i].clean();
    }

    // Follow BLE connection status
    asyncSubscribe(WB_RES::LOCAL::COMM_BLE_PEERS());

    // Configure custom gatt service
    configGattSvc();

    return true;
}

void GATTSensorDataClient::stopModule()
{
    // Unsubscribe sensor data
    unsubscribeAllStreams();

    // Clean up GATT stuff
    asyncUnsubscribe(mCommandCharResource);
    asyncUnsubscribe(mDataCharResource);
    
    releaseResource(mCommandCharResource);
    releaseResource(mDataCharResource);
    
    mCommandCharResource = wb::ID_INVALID_RESOURCE;
    mDataCharResource = wb::ID_INVALID_RESOURCE;

    mModuleState = WB_RES::ModuleStateValues::STOPPED;
}

void GATTSensorDataClient::configGattSvc()
{
    WB_RES::GattSvc customGattSvc;
    WB_RES::GattChar characteristics[2];
    WB_RES::GattChar &commandChar = characteristics[0];
    WB_RES::GattChar &dataChar = characteristics[1];

    // Define the CMD characteristics
    WB_RES::GattProperty dataCharProp = WB_RES::GattProperty::NOTIFY;
    WB_RES::GattProperty commandCharProp = WB_RES::GattProperty::WRITE;

    dataChar.props = wb::MakeArray<WB_RES::GattProperty>( &dataCharProp, 1);
    dataChar.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&DATA_CHAR_UUID), sizeof(DATA_CHAR_UUID));

    commandChar.props = wb::MakeArray<WB_RES::GattProperty>( &commandCharProp, 1);
    commandChar.uuid = wb::MakeArray<uint8_t>( reinterpret_cast<const uint8_t*>(&COMMAND_CHAR_UUID), sizeof(COMMAND_CHAR_UUID));

    // Combine chars to service
    customGattSvc.uuid = wb::MakeArray<uint8_t>( SENSOR_DATASERVICE_UUID, sizeof(SENSOR_DATASERVICE_UUID));
    customGattSvc.chars = wb::MakeArray<WB_RES::GattChar>(characteristics, 2);

    // Create custom service
    asyncPost(WB_RES::LOCAL::COMM_BLE_GATTSVC(), AsyncRequestOptions(NULL,0,true), customGattSvc);
}

// Simple command structure:
// - command [1 byte]
// - client reference [1 byte, not zero!]
// - Command specific data
//
// Result and data notifications are returned via dataCharacteristic in format
// - result type [1 byte]: (1= response to command, )2: data notification from subscription
// - client reference [1 byte]
// - data: (2 byte "HTTP result" for commands, sbem formatted binary for subscriptions)

// Command reference:
// SUBSCRIBE (=1)
//   data == WB Resource path as string
//   data is returned in DATA -reponses with the given reference, the format 
//   is the generated SBEM serialization format for the firmware. Max 4 subscriptions at the time!
//
// UNSUBSCRIBE (=2)
//   no data
//   reference must match one given in SUBSCRIBE command
//
// FETCH_OFFLINE_DATA (=3)
//   no data
//   reference must be different than the one given in SUBSCRIBE command
//   data is returned in DATA -reponses with the given reference, the format 
//   is the same as in notification (No SBEM wrapping!). All notifications are returend
//   with the same reference, so they must be distinguished by length

enum Commands 
{
    HELLO = 0,
    SUBSCRIBE = 1,
    UNSUBSCRIBE = 2,
    FETCH_OFFLINE_DATA = 3,
    INIT_OFFLINE = 4,
};
enum Responses 
{
    COMMAND_RESULT = 1,
    DATA = 2,
    DATA_PART2 = 3, // In case the subscription data is larger than fits in the single BLE packet, continue with Part2 & 3
    DATA_PART3 = 4,
};

GATTSensorDataClient::DataSub* GATTSensorDataClient::findDataSub(const wb::ResourceId resourceId)
{
    for (size_t i=0; i<MAX_DATASUB_COUNT; i++)
    {
        const DataSub &ds= mDataSubs[i];
        if (ds.resourceId == resourceId)
            return &(mDataSubs[i]);
    }
    return nullptr;
}
GATTSensorDataClient::DataSub*  GATTSensorDataClient::findDataSubByRef(const uint8_t clientReference)
{
    for (size_t i=0; i<MAX_DATASUB_COUNT; i++)
    {
        const DataSub &ds= mDataSubs[i];
        if (ds.clientReference == clientReference)
            return &(mDataSubs[i]);
    }
    return nullptr;
}

GATTSensorDataClient::DataSub* GATTSensorDataClient::getFreeDataSubSlot()
{
    for (size_t i=0; i<MAX_DATASUB_COUNT; i++)
    {
        const DataSub &ds= mDataSubs[i];
        if (ds.clientReference == 0 && ds.resourceId == wb::ID_INVALID_RESOURCE)
            return &(mDataSubs[i]);
    }
    return nullptr;
}

void GATTSensorDataClient::sendOfflineData(uint8_t reference)
{
    // Start sending offline data by subscribing to the Logbook Data resource. Send only last log of first 4.
    mLogToSend = 0;
    mLogSendReference = reference;
    asyncGet(WB_RES::LOCAL::MEM_LOGBOOK_ENTRIES(), AsyncRequestOptions(NULL, 0, false));
    // The flow will continue in onGetResult...
}

void GATTSensorDataClient::handleSendingOfflineData(const uint8_t* data, size_t length)
{
    DEBUGLOG("handleSendingOfflineData(), length: %d", length);
    // Read sbem item at the time and send individually (for now)    
    size_t readIdx = 0;
    // Skip header if this is very first data packet.
    if (!mFirstPacketSent) {
        mFirstPacketSent = true;
        readIdx = 8;
    }

    while (readIdx<length)
    {
        int bytesLeftInSrc = length - readIdx;

        // Make sure that buffer has at least the sbem header worth of data
        constexpr size_t MAX_SBEM_HEADER_LENGTH = 6;// (2 + 4 bytes)
        if (mSendBufferLength < MAX_SBEM_HEADER_LENGTH)
        {
            int copyCount = MAX_SBEM_HEADER_LENGTH-mSendBufferLength;
            // Don't copy more than we have in the src
            if (copyCount > bytesLeftInSrc)
                copyCount = bytesLeftInSrc;

            memcpy(&mSendBuffer[mSendBufferLength], &(data[readIdx]), copyCount);
            readIdx += copyCount;
            mSendBufferLength += copyCount;
            bytesLeftInSrc -= copyCount;
        }

        if (bytesLeftInSrc > 0)
        {
            // Read sbemHeader from buffer to get the item length
            uint32 chunkId=0, payloadLen=0;
            uint32 headerBytes = sbem::readChunkHeader(mSendBuffer, chunkId, payloadLen);
            DEBUGLOG("sbemChunk: id: %d, headerBytes: %d, payloadLen: %d", chunkId, headerBytes, payloadLen);

            // Read the rest of the payload or as much as there is to copy
            const size_t sbemChunkSize = headerBytes + payloadLen;
            DEBUGLOG("sbemChunkSize:  %d, bytesLeftInSrc: %d", sbemChunkSize, bytesLeftInSrc);
            const int bytesNeededToFillSbemChunkInBuffer = sbemChunkSize - mSendBufferLength;
            DEBUGLOG("bytesNeededToFillSbemChunkInBuffer: %d", bytesNeededToFillSbemChunkInBuffer);

            const size_t bytesToCopy = WB_MIN(bytesNeededToFillSbemChunkInBuffer, bytesLeftInSrc);
            DEBUGLOG("bytesToCopy: %d", bytesToCopy);

            memcpy(&mSendBuffer[mSendBufferLength], &(data[readIdx]), bytesToCopy);
            readIdx += bytesToCopy;
            mSendBufferLength += bytesToCopy;
            DEBUGLOG("sbemChunkSize: %d, mSendBufferLength: %d", sbemChunkSize, mSendBufferLength);

            if (sbemChunkSize <= mSendBufferLength)
            {
                // There is enough data in buffer, send the payload
                WB_RES::Characteristic dataCharValue;
                // Re-use the sbem header area to fill response code & client reference
                uint8_t *packetStartPtr = &(mSendBuffer[headerBytes-2]);
                packetStartPtr[0] = Responses::DATA;
                packetStartPtr[1] = mLogSendReference;
                dataCharValue.bytes = wb::MakeArray<uint8_t>(packetStartPtr, payloadLen+2);
                asyncPut(mDataCharResource, AsyncRequestOptions::Empty, dataCharValue);
                // copy the possible excess bytes in buffer to the beginning of the buffer
                auto remainingBytes = mSendBufferLength - payloadLen - headerBytes;
                for (size_t src = headerBytes + payloadLen, dst=0; src < mSendBufferLength; src++, dst++)
                {
                    mSendBuffer[dst] = mSendBuffer[src];
                }
                mSendBufferLength = remainingBytes;
            }
            else
            {
                // Not enough data in buffer, waiting for next datafill
            }
        }    
        DEBUGLOG("end of while. readIdx: %d", readIdx);
    }
}

void GATTSensorDataClient::handleIncomingCommand(const wb::Array<uint8> &commandData)
{

    uint8_t cmd = commandData[0];
    uint8_t reference = commandData[1];
    const uint8_t *pData = commandData.size()>2 ? &(commandData[2]) : nullptr;
    uint16_t dataLen = commandData.size() - 2;

    switch (cmd)
    {
        case Commands::HELLO:
        {
            // Hello response
            uint8_t helloMsg[] = {COMMAND_RESULT, reference, 'H','e','l','l','o'};

            WB_RES::Characteristic dataCharValue;
            dataCharValue.bytes = wb::MakeArray<uint8_t>(helloMsg, sizeof(helloMsg));
            asyncPut(mDataCharResource, AsyncRequestOptions::ForceAsync, dataCharValue);
            return;
        }
        case Commands::SUBSCRIBE:
        {
            DataSub *pDataSub = getFreeDataSubSlot();

            if (!pDataSub)
            {
                DEBUGLOG("No free datasub slot");
                // 507: HTTP_CODE_INSUFFICIENT_STORAGE
                uint8_t errorMsg[] = {COMMAND_RESULT, reference, 0x01,0xFB};
                
                WB_RES::Characteristic dataCharValue;
                dataCharValue.bytes = wb::MakeArray<uint8_t>(errorMsg, sizeof(errorMsg));
                asyncPut(mDataCharResource, AsyncRequestOptions::ForceAsync, dataCharValue);
                return;
            }

            // Store client reference to array and trigger subsribe
            DataSub &dataSub = *pDataSub;

            char pathBuffer[160]; // Big enough since MTU is 161
            memset(pathBuffer, 0, sizeof(pathBuffer));

            // Copy and null-terminate
            memcpy(pathBuffer, pData, dataLen);

            dataSub.subStarted = true;
            dataSub.subCompleted = false;
            dataSub.clientReference = reference;
            getResource(pathBuffer, dataSub.resourceId);

            // See if path short enough to record in case of datalogger use in disconnect
            const auto pathLen = strnlen(pathBuffer, sizeof(pathBuffer));
            if (pathLen < sizeof(dataSub.resourcePath)-1)
            {
                DEBUGLOG("Path stored in case of disconnect: %s", pathBuffer);
                memcpy(dataSub.resourcePath, pathBuffer, pathLen+1);

                if (!OfflineStorageClient::IsLogging())
                {
                    // Update Datalogger config if there is no ongoing logging
                    // this situation would happen when sensor has reconnected
                    updateDataLoggerConfig();
                }
                else
                {
                    DEBUGLOG("Ongoing logging, won't update DataLogger config.");
                }
            }

            // Use non-critical subscription so that buffer full doesn't crash the sensor
            asyncSubscribe(dataSub.resourceId, AsyncRequestOptions::NotCriticalSubscription);
        }
        break;
        case Commands::UNSUBSCRIBE:
        {
            DEBUGLOG("Commands::UNSUBSCRIBE. reference: %d", reference);

            // Store client reference to array and trigger subsribe
            DataSub *pDataSub = findDataSubByRef(reference);
            if (pDataSub != nullptr)
            {
                asyncUnsubscribe(pDataSub->resourceId);
                DEBUGLOG(" asyncUnsubscribe sent, cleaning");
                pDataSub->clean();
            }
            updateDataLoggerConfig();
            break;
        }
        case Commands::FETCH_OFFLINE_DATA:
        {
            DEBUGLOG("Commands::FETCH_OFFLINE_DATA. reference: %d", reference);

            sendOfflineData(reference);
            break;
        }
        case Commands::INIT_OFFLINE:
        {
            // Clean offline storage 
            asyncDelete(WB_RES::LOCAL::MEM_LOGBOOK_ENTRIES());

            uint8_t okResponse[] = {COMMAND_RESULT, reference, 200u};
            
            WB_RES::Characteristic dataCharValue;
            dataCharValue.bytes = wb::MakeArray<uint8_t>(okResponse, sizeof(okResponse));
            asyncPut(mDataCharResource, AsyncRequestOptions::ForceAsync, dataCharValue);
            break;
        }
    }
}

void GATTSensorDataClient::updateDataLoggerConfig()
{
    DEBUGLOG("updateDataLoggerConfig()");
    // Change datalogger config to match current subscriptions
    // TODO: skip if already recording?!?
    WB_RES::DataLoggerConfig ldConfig;
    WB_RES::DataEntry entries[MAX_DATASUB_COUNT];
    size_t count = 0;
    for (size_t i=0; i<MAX_DATASUB_COUNT; i++)
    {
        DEBUGLOG("ref: %d, resource: %d, path: %s", mDataSubs[i].clientReference, mDataSubs[i].resourceId, mDataSubs[i].resourcePath);
        if (!mDataSubs[i].isEmpty() &&
            strnlen(mDataSubs[i].resourcePath, sizeof(mDataSubs[i].resourcePath)) > 0)
        {
            DEBUGLOG("Add path to config: %s", mDataSubs[i].resourcePath);
            entries[count++].path = mDataSubs[i].resourcePath;
        }
    }

    ldConfig.dataEntries.dataEntry = wb::MakeArray<WB_RES::DataEntry>(entries, count);
    // Set new config
    asyncPut(WB_RES::LOCAL::MEM_DATALOGGER_CONFIG(), AsyncRequestOptions::ForceAsync, ldConfig);
}

void GATTSensorDataClient::onGetResult(wb::RequestId requestId,
                                      wb::ResourceId resourceId,
                                      wb::Result resultCode,
                                      const wb::Value& rResultData)
{
    DEBUGLOG("GATTSensorDataClient::onGetResult");
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE::LID:
        {
            // This code finalizes the service setup (triggered by code in onPostResult)
            const WB_RES::GattSvc &svc = rResultData.convertTo<const WB_RES::GattSvc &>();
            for (size_t i=0; i<svc.chars.size(); i++)
            {
                // Find out characteristic handles and store them for later use
                const WB_RES::GattChar &c = svc.chars[i];
                // Extract 16 bit sub-uuid from full 128bit uuid
                DEBUGLOG("c.uuid.size(): %u", c.uuid.size());
                uint16_t uuid16 = *reinterpret_cast<const uint16_t*>(&(c.uuid[12]));
                
                DEBUGLOG("char[%u] uuid16: 0x%04X", i, uuid16);

                if(uuid16 == dataCharUUID16)
                    mDataCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
                else if(uuid16 == commandCharUUID16)
                    mCommandCharHandle = c.handle.hasValue() ? c.handle.getValue() : 0;
            }

            if (!mCommandCharHandle || !mDataCharHandle)
            {
                DEBUGLOG("ERROR: Not all chars were configured!");
                return;
            }

            char pathBuffer[32]= {'\0'};
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mSensorSvcHandle, mCommandCharHandle);
            getResource(pathBuffer, mCommandCharResource);
            snprintf(pathBuffer, sizeof(pathBuffer), "/Comm/Ble/GattSvc/%d/%d", mSensorSvcHandle, mDataCharHandle);
            getResource(pathBuffer, mDataCharResource);

            // Forse subscriptions asynchronously to save stack (will have stack overflow if not) 
            // Subscribe to listen to intervalChar notifications (someone writes new value to intervalChar) 
            asyncSubscribe(mCommandCharResource, AsyncRequestOptions::ForceAsync);
            // Subscribe to listen to measChar notifications (someone enables/disables the INDICATE characteristic) 
            asyncSubscribe(mDataCharResource,  AsyncRequestOptions::ForceAsync);
            break;
        }
        case WB_RES::LOCAL::MEM_LOGBOOK_ENTRIES::LID:
        {
            if (resultCode != wb::HTTP_CODE_OK)
            {
                DEBUGLOG("Error fetching log entries: %d", resultCode);
            }
            // This code finalizes the service setup (triggered by code in onPostResult)
            const auto &logEntries = rResultData.convertTo<const WB_RES::LogEntries &>();

            DEBUGLOG("MEM_LOGBOOK_ENTRIES. result: %d", resultCode);
            
            // Send last logId of the first page of logs
            mLogToSend = 0;
            for (size_t i=0; i<logEntries.elements.size(); i++)
            {
                mLogToSend = logEntries.elements[i].id;
                DEBUGLOG("- id: %d", mLogToSend);
            }

            if (mLogToSend > 0)
            {
                // In case sensor does not support it (eeprom v2.1.x), onSubscribeResult will do GET instead
                DEBUGLOG("Subscribing to data of log %d", mLogToSend);
                mFirstPacketSent = false;
                asyncSubscribe(WB_RES::LOCAL::MEM_LOGBOOK_BYID_LOGID_DATA(), AsyncRequestOptions::ForceAsync, mLogToSend);
            }
            else
            {
                DEBUGLOG("No logs to send");
            }
            break;
        }
        case WB_RES::LOCAL::MEM_LOGBOOK_BYID_LOGID_DATA::LID:
        {
            // This code finalizes the service setup (triggered by code in onPostResult)
            const auto &stream = rResultData.convertTo<const wb::ByteStream &>();
            DEBUGLOG("Sendind from get. size: %d", stream.length());

            handleSendingOfflineData(stream.data, stream.length());
            if (resultCode == wb::HTTP_CODE_CONTINUE)
            {
                // Do another GET request to get the next bytes (needs to be async)
                asyncGet(WB_RES::LOCAL::MEM_LOGBOOK_BYID_LOGID_DATA(), AsyncRequestOptions::ForceAsync, mLogToSend);
            }
            break;
        }
    }
}

/** @see whiteboard::ResourceClient::onSubscribeResult */
void  GATTSensorDataClient::onSubscribeResult(wb::RequestId requestId,
                                              wb::ResourceId resourceId,
                                              wb::Result resultCode,
                                              const wb::Value& rResultData)
{
    DEBUGLOG("onSubscribeResult() code: %d, localResourceId: %u", resultCode, resourceId.localResourceId);

    switch (resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_PEERS::LID:
            {
                DEBUGLOG("OnSubscribeResult: WB_RES::LOCAL::COMM_BLE_PEERS: %d", resultCode);
                return;
            }
            break;
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
            {
                DEBUGLOG("OnSubscribeResult: COMM_BLE_GATTSVC*: %d", resultCode);
                return;
            }
            break;
        case WB_RES::LOCAL::MEM_LOGBOOK_BYID_LOGID_DATA::LID:
            {
                if (resultCode >= 500)
                {
                    // Do GET instead.
                    DEBUGLOG("Logbook Data subscription not available, GET instead");
                    asyncGet(WB_RES::LOCAL::MEM_LOGBOOK_BYID_LOGID_DATA(), AsyncRequestOptions::Empty, mLogToSend);
                }
            }
            break;
        default:
        {
            // All other notifications. These must be the client subscribed data streams
            GATTSensorDataClient::DataSub *ds = findDataSub(resourceId);
            if (ds == nullptr)
            {
                DEBUGLOG("DataSub not found for resource: %u", resourceId);
                return;
            }
            ASSERT(ds->subStarted);
            if (ds->subCompleted)
            {
                DEBUGLOG("subCompleted already: %u", resourceId);
                return;
            }

            if (resultCode >= 400)
            {
                ds->clientReference = 0;
                ds->resourceId = wb::ID_INVALID_RESOURCE;
                ds->subStarted=false;
                ds->subCompleted=false;
            }
            else
            {
                ds->subCompleted=true;
            }
        }
        break;
    }
}


void GATTSensorDataClient::unsubscribeAllStreams()
{

    DEBUGLOG("unsubscribeAllStreams()");
    for (size_t i=0;i<MAX_DATASUB_COUNT; i++)
    {
        if (!mDataSubs[i].isEmpty())
        {
            DEBUGLOG("asyncUnsubscribe(). resourceId: %d", mDataSubs[i].resourceId);
            asyncUnsubscribe(mDataSubs[i].resourceId);
            mDataSubs[i].clean();
        }
    }
}

void GATTSensorDataClient::onTimer(whiteboard::TimerId timerId)
{
    // 
    unsubscribeAllStreams();
}

void GATTSensorDataClient::onNotify(wb::ResourceId resourceId,
                                   const wb::Value& value,
                                   const wb::ParameterList& rParameters)
{
    switch(resourceId.localResourceId)
    {
        case WB_RES::LOCAL::COMM_BLE_PEERS::LID:
            {
                DEBUGLOG("GATTSensorDataClient::onNotify::COMM_BLE_PEERS");
                WB_RES::PeerChange peerChange = value.convertTo<WB_RES::PeerChange>();
                
                if (peerChange.state == peerChange.state.DISCONNECTED)
                {
                    // if connection is dropped, 
                    // unsubscribe all data streams so that sensor does not stay on for no reason,
                    // However, we need to wait a while for the offline storage to start first so that there is no
                    // disconnect in data stream
                    startTimer(UNSUBSCRIBE_TIMEOUT);
                }
                else if (peerChange.peer.handle.hasValue())
                {
                    const auto handle = peerChange.peer.handle.getValue();
                    // Set shorter timeout to BLE params
                    WB_RES::ConnParams connParams;
                    connParams.min_conn_interval = 12;
                    connParams.max_conn_interval = 36;
                    connParams.slave_latency = 2;
                    connParams.sup_timeout = 100;
                    asyncPut(WB_RES::LOCAL::COMM_BLE_PEERS_CONNHANDLE_PARAMS(), AsyncRequestOptions::ForceAsync, handle, connParams);

                }
            }

            break;
        case WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::LID:
        {
            WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE_CHARHANDLE::SUBSCRIBE::ParameterListRef parameterRef(rParameters);
            if (parameterRef.getCharHandle() == mCommandCharHandle) 
            {
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic &>();

                DEBUGLOG("onNotify: mCommandCharHandle: len: %d", charValue.bytes.size());

                handleIncomingCommand(charValue.bytes);
                return;
            }
            else if (parameterRef.getCharHandle() == mDataCharHandle) 
            {
                const WB_RES::Characteristic &charValue = value.convertTo<const WB_RES::Characteristic &>();
                // Update the notification state so we know if to forward data to datapipe
                mNotificationsEnabled = charValue.notifications.hasValue() ? charValue.notifications.getValue() : false;
                DEBUGLOG("onNotify: mDataCharHandle. mNotificationsEnabled: %d", mNotificationsEnabled);
            }
            break;
        }
        case WB_RES::LOCAL::MEM_LOGBOOK_BYID_LOGID_DATA::LID:
        {
            const auto &dataNotification = value.convertTo<const WB_RES::LogDataNotification &>();
            DEBUGLOG("Sendind from notification. offset: %d, size: %d", dataNotification.offset, dataNotification.bytes.size());

            if (dataNotification.bytes.size()>0) {
                handleSendingOfflineData(&(dataNotification.bytes[0]), dataNotification.bytes.size());
            }
            else {
                // length=0  ===> end of transfer
                asyncUnsubscribe(WB_RES::LOCAL::MEM_LOGBOOK_BYID_LOGID_DATA(), AsyncRequestOptions::ForceAsync, mLogToSend);
                mLogToSend = 0;
            }
            break;
        }

        default:
        {
            // All other notifications. These must be the client subscribed data streams
            GATTSensorDataClient::DataSub *ds = findDataSub(resourceId);
            if (ds == nullptr)
            {
                DEBUGLOG("DataSub not found for resource: %u", resourceId);
                return;
            }

            // Make sure we can serialize the data
            size_t length = getSbemLength(resourceId.localResourceId, value);
            if (length == 0)
            {
                DEBUGLOG("No length for localResourceId: %u", resourceId.localResourceId);
                return;
            }

            // Forward data to client
            memset(mDataMsgBuffer, 0, sizeof(mDataMsgBuffer));
            mDataMsgBuffer[0] = DATA;
            mDataMsgBuffer[1] = ds->clientReference;

            length = writeToSbemBuffer(&mDataMsgBuffer[2], sizeof(mDataMsgBuffer)-2, 0, resourceId.localResourceId, value);

            WB_RES::Characteristic dataCharValue;
            dataCharValue.bytes = wb::MakeArray<uint8_t>(mDataMsgBuffer, length+2);

            asyncPut(mDataCharResource, AsyncRequestOptions::Empty, dataCharValue);

            return;

            break;
        }

    }
}

void GATTSensorDataClient::onPostResult(wb::RequestId requestId, 
                                       wb::ResourceId resourceId, 
                                       wb::Result resultCode, 
                                       const wb::Value& rResultData)
{
    DEBUGLOG("GATTSensorDataClient::onPostResult: %d", resultCode);

    if (resultCode == wb::HTTP_CODE_CREATED)
    {
        // Custom Gatt service was created
        mSensorSvcHandle = (int32_t)rResultData.convertTo<uint16_t>();
        DEBUGLOG("Custom Gatt service was created. handle: %d", mSensorSvcHandle);
        
        // Request more info about created svc so we get the char handles
        asyncGet(WB_RES::LOCAL::COMM_BLE_GATTSVC_SVCHANDLE(), AsyncRequestOptions(NULL,0,true), mSensorSvcHandle);
        // Note: The rest of the init is performed in onGetResult()
    }
}