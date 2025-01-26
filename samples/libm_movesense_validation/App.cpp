#include "BleServicesApp.hpp"
#include "ECGBasedRRService.h"
#include "GATTSensorDataProtocol.h"
#include "movesense.h"

MOVESENSE_APPLICATION_STACKSIZE(1024)

MOVESENSE_PROVIDERS_BEGIN(3)

MOVESENSE_PROVIDER_DEF(BleServicesApp)
MOVESENSE_PROVIDER_DEF(ECGBasedRRService)
MOVESENSE_PROVIDER_DEF(GATTSensorDataProtocol)

MOVESENSE_PROVIDERS_END(3)

MOVESENSE_FEATURES_BEGIN()
// Explicitly enable or disable Movesense framework core modules.
// List of modules and their default state is found in documentation
OPTIONAL_CORE_MODULE(DataLogger, true)
OPTIONAL_CORE_MODULE(Logbook, true)
OPTIONAL_CORE_MODULE(LedService, true)
OPTIONAL_CORE_MODULE(IndicationService, true)
OPTIONAL_CORE_MODULE(BleService, true)
OPTIONAL_CORE_MODULE(EepromService, true)
OPTIONAL_CORE_MODULE(BypassService, false)
OPTIONAL_CORE_MODULE(SystemMemoryService, false)
OPTIONAL_CORE_MODULE(DebugService, false)
OPTIONAL_CORE_MODULE(BleStandardHRS, true)
OPTIONAL_CORE_MODULE(BleNordicUART, false)
OPTIONAL_CORE_MODULE(CustomGattService, true)

// NOTE: If building a simulator build, these macros are obligatory!
DEBUGSERVICE_BUFFER_SIZE(6, 120); // 6 lines, 120 characters total
DEBUG_EEPROM_MEMORY_AREA(false, 0, 0)
LOGBOOK_EEPROM_MEMORY_AREA(0, MEMORY_SIZE_FILL_REST);

APPINFO_NAME("LIBM Movesense Validation Firmware");
APPINFO_VERSION("1.0.0");
APPINFO_COMPANY("LIBM");

// NOTE: SERIAL_COMMUNICATION & BLE_COMMUNICATION macros have been DEPRECATED
MOVESENSE_FEATURES_END()
