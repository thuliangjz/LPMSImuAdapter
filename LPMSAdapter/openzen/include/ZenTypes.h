//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ZEN_API_ZENTYPES_H_
#define ZEN_API_ZENTYPES_H_

#include <stdint.h>

#ifdef _WIN32

#if defined(ZEN_API_STATIC) || !defined(__cplusplus)
#define ZEN_API
#elif defined(ZEN_API_EXPORT)
#define ZEN_API __declspec(dllexport)
#else
#define ZEN_API __declspec(dllimport)
#endif

#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)

#ifdef ZEN_API_STATIC
#define ZEN_API
#elif defined(ZEN_API_EXPORT)
#define ZEN_API __attribute__((visibility("default")))
#else
#define ZEN_API
#endif

#elif SWIG
#define ZEN_API
#else

#error "This platform is not supported, yet."

#endif

#ifdef _WIN32
#define ZEN_CALLTYPE __cdecl
#else
#define ZEN_CALLTYPE
#endif

typedef struct ZenClientHandle { uintptr_t handle; } ZenClientHandle_t;
typedef struct ZenSensorHandle { uintptr_t handle; } ZenSensorHandle_t;
typedef struct ZenComponentHandle { uintptr_t handle; } ZenComponentHandle_t;

typedef int ZenError_t;

typedef enum ZenError
{
    ZenError_None = 0,                       // Not error reported
    ZenError_Unknown = 1,                    // Unkown error occured

    ZenError_IsNull = 10,                    // Pointer is invalid (null)
    ZenError_NotNull = 11,                   // Expected a null pointer
    ZenError_WrongDataType = 12,             // Wrong data type
    ZenError_BufferTooSmall = 13,            // Provided buffer is too small for the return data
    ZenError_InvalidArgument = 14,           // An invalid argument was provided
    ZenError_NotSupported = 15,              // The requested functionality is not supported by
                                             // this build of OpenZen

    ZenError_AlreadyInitialized = 20,        // Already initialized
    ZenError_NotInitialized = 21,            // Not initialized

    ZenError_Device_IoTypeInvalid = 30,         // Invalid IO type
    ZenError_Sensor_VersionNotSupported = 31,   // Sensor version is not supported
    ZenError_Device_ListingFailed = 32,         // An error occured while listing devices
    ZenError_Device_Listing = 35,               // Busy listing devices

    ZenError_WrongSensorType = 40,           // Wrong sensor type
    ZenError_WrongIoType = 41,               // Wrong IO type
    ZenError_UnknownDeviceId = 42,           // Unknown device ID

    ZenError_Io_AlreadyInitialized = 800,    // IO interface was already initialized
    ZenError_Io_NotInitialized = 801,        // IO interface is not initialized
    ZenError_Io_InitFailed = 802,            // Failed to open IO interface
    ZenError_Io_DeinitFailed = 803,          // Failed to deinitialize IO interface
    ZenError_Io_ReadFailed = 804,            // Failed to read from IO interface
    ZenError_Io_SendFailed = 805,            // Failed to send to IO interface
    ZenError_Io_GetFailed = 806,             // Failed to get value from IO interface
    ZenError_Io_SetFailed = 807,             // Failed to set value on IO interface
    ZenError_Io_Busy = 811,                  // IO Interface is busy sending/receiving another message. Try again later
    ZenError_Io_Timeout = 812,               // IO Interface timed out. Try again.
    ZenError_Io_UnexpectedFunction = 813,    // IO Interface received an unexpected function
    ZenError_Io_UnsupportedFunction = 814,   // IO Interface received an unsupported function
    ZenError_Io_MsgCorrupt = 815,            // Received message is corrupt
    ZenError_Io_MsgTooBig = 816,             // Trying to send too much data over CAN interface
    ZenError_Io_ExpectedAck = 820,           // IO state manager did not receive an ACK or NACK message
    ZenError_Io_BaudratesUnknown = 821,      // IO Interface does not know supported baudrates values

    ZenError_UnknownProperty = 850,          // Sensor does not support the property
    ZenError_UnknownCommandMode = 851,       // Sensor does not support the command mode
    ZenError_UnsupportedEvent = 852,        // Host does not support the event type

    ZenError_FW_FunctionFailed = 900,        // Firmware failed to execute the requested function

    ZenError_Can_BusError = 1001,            // Can interface is in an error state
    ZenError_Can_OutOfAddresses = 1002,      // Can interface cannot support any more devices on the channel. Close another device first
    ZenError_Can_ResetFailed = 1006,         // Failed to reset queues of CAN interface
    ZenError_Can_AddressOutOfRange = 1009,   // Trying to send message to an address that is too big (max: 255)

    ZenError_InvalidClientHandle = 2000,    // Invalid client handle
    ZenError_InvalidSensorHandle = 2001,    // Invalid sensor handle
    ZenError_InvalidComponentHandle = 2002, // Invalid component handle

    ZenError_Max
} ZenError;

typedef enum ZenSensorInitError
{
    ZenSensorInitError_None = 0,                // No error reported

    ZenSensorInitError_InvalidHandle,           // Provided client handle is invalid
    ZenSensorInitError_IsNull,                  // Provided pointer is null
    ZenSensorInitError_UnknownIdentifier,       // Provided sensor identifier is unknown
    ZenSensorInitError_UnsupportedComponent,    // At least one of the sensor's component types is not supported by the host
    ZenSensorInitError_UnsupportedDataFormat,   // Provided Modbus Format is not supported
    ZenSensorInitError_UnsupportedIoType,       // Provided IO type is not supported
    ZenSensorInitError_UnsupportedProtocol,     // The sensor's protocol version is not supported by the host
    ZenSensorInitError_UnsupportedFunction,     // This function is not supported by the IO sytsem

    ZenSensorInitError_ConnectFailed,           // Failed to establish a connection with the sensor
    ZenSensorInitError_IoFailed,                // Low-level IO API returned an error
    ZenSensorInitError_RetrieveFailed,          // Failed to retrieve a property from the sensor
    ZenSensorInitError_SetBaudRateFailed,       // Failed to change the BaudRate
    ZenSensorInitError_SendFailed,              // Failed to send data
    ZenSensorInitError_Timeout,                 // Timeout when sending command to initialize sensor
    ZenSensorInitError_IncompatibleBaudRates,   // Unable to find a compatible BaudRate
    ZenSensorInitError_InvalidAddress,          // Provided remote address is invalid
    ZenSensorInitError_InvalidConfig,           // The configuration file is invalid
    ZenSensorInitError_NoConfiguration,         // No configuration is available for this sensor type

    ZenSensorInitError_Max
} ZenSensorInitError;

typedef enum ZenAsyncStatus
{
    ZenAsync_Finished,

    ZenAsync_ThreadBusy,
    ZenAsync_InvalidArgument,
    ZenAsync_Updating,
    ZenAsync_Failed,

    ZenAsync_Max
} ZenAsyncStatus;

typedef enum ZenLogLevel
{
    ZenLogLevel_Off,
    ZenLogLevel_Error,
    ZenLogLevel_Warning,
    ZenLogLevel_Info,
    ZenLogLevel_Debug,

    ZenLogLevel_Max
} ZenLogLevel;

typedef struct ZenImuData
{
    /// Index of the data frame.
    /// Unit: no unit
    int frameCount;

    /// Sampling time of the data in seconds
    /// Unit: s
    double timestamp;

    /// Calibrated accelerometer sensor data.
    /// Unit: m/s^2
    float a[3];

    /// Calibrated gyroscope sensor data.
    /// Unit: degree/s
    float g[3];

    /// Calibrated magnetometer sensor data.
    /// Unit: micro Tesla
    float b[3];

    /// Raw accelerometer sensor data.
    /// Unit: m/s^2
    float aRaw[3];

    /// Raw gyroscope sensor data.
    /// Unit: degree/s
    float gRaw[3];

    /// Raw magnetometer sensor data.
    /// Unit: micro Tesla
    float bRaw[3];

    /// Angular velocity data.
    /// This angular velocity takes into account if an orientation offset
    /// has been set while the g and gRaw values in this struct
    /// do not.
    float w[3];

    /// Euler angle data.
    /// Unit: degree/s
    float r[3];

    /// Quaternion orientation data.
    /// The component order is w, x, y, z
    /// Unit: no unit
    float q[4];

    /// Orientation data as rotation matrix without offset.
    /// Unit: no unit
    float rotationM[9];

    /// Orientation data as rotation matrix after zeroing.
    /// Unit: no unit
    float rotOffsetM[9];

    /// Barometric pressure.
    /// Unit: mPascal
    float pressure;

    /// Linear acceleration x, y and z.
    /// Unit: m/s^2
    float linAcc[3];

    /// Gyroscope temperature.
    /// Unit: Degree Celcius
    float gTemp;

    /// Altitude.
    /// Unit: m
    float altitude;

    /// Temperature.
    /// Unit: Degree Celcius
    float temperature;

    /// heave motion
    /// Unit: m
    float heaveMotion;
} ZenImuData;

typedef ZenImuData ZenEventData_Imu;

/**
Type of the position fix provided by the global navigation satellite system (Gnss)
*/
typedef enum ZenGnssFixType
{
    /// No fix available
    ZenGnssFixType_NoFix = 0,

    /// Position estimation was done by using dead reckoning method because
    /// not GNSS location fix is available
    ZenGnssFixType_DeadReckoningOnly = 1,

    /// Position fix on the surface of the WGS84 ellipsoid
    ZenGnssFixType_2dFix = 2,

    /// 3d fix in space with a valid height information
    ZenGnssFixType_3dFix = 3,

    /// Position estimation was done using a combination of an up-to-date
    /// GNSS position measurement and dead reckoning
    ZenGnssFixType_GnssAndDeadReckoning = 4,

    /// Only the time and date was obtained from the GNSS so far.
    ZenGnssFixType_TimeOnlyFix = 5,

    ZenGnssFixType_Max
} ZenGnssFixType;

/**
Types of RTK carrier phase correction methods used for the
navigation solution.
*/
typedef enum ZenGnssFixCarrierPhaseSolution
{
    /// No carrier phase method used
    ZenGnssFixCarrierPhaseSolution_None = 0,

    /// Carrier phase method using floating ambiguities for solution
    ZenGnssFixCarrierPhaseSolution_FloatAmbiguities = 1,

    /// Carrier phase method using fixed ambiguities for solution
    ZenGnssFixCarrierPhaseSolution_FixedAmbiguities = 2,
} ZenGnssFixCarrierPhaseSolution;

/**
Global position, velocity and heading information measured using
a global navigation satellite system (GNSS) and dead reckoning methods with
the IMU sensors.
*/
typedef struct ZenGnssData
{
    /// Index of the data frame.
    int frameCount;

    /// Sampling time of the data in seconds
    double timestamp;

    /// Latitude measurement provided by the GNSS
    /// or the IMU/GNSS sensor fusion
    double latitude;

    /// Accuracy of the horizontal measurement in m
    double horizontalAccuracy;

    /// Longitude measurement provided by the GNSS
    /// or the IMU/GNSS sensor fusion
    double longitude;

    /// Accuracy of the vertical position measurement in m
    double verticalAccuracy;

    /// height above WGS84 ellipsoid in m
    double height;

    /// Heading of sensor motion in degrees in clockwise counting
    /// and 0 degree being north, only usable for Dead-reckoning GPS
    double headingOfMotion;

    /// Heading of Vehicle in degrees in clockwise counting
    /// and 0 degree being north, only usable for Dead-reckoning GPS
    /// This heading is not changing if the vehicle drives backwards for
    /// example as it is aligned with the forward direction of the vehicle
    double headingOfVehicle;

    /// Heading Accuracy in degrees for both headingOfVehicle and
    /// headingOfMotion
    double headingAccuracy;

    /// velocity over ground in m/s
    double velocity;

    /// velocity accuracy over ground in m/s
    double velocityAccuracy;

    /// type of the GNSS fix and dead-reckoning mode
    ZenGnssFixType fixType;

    /// Additional RTK carrier phase correction applied
    /// to improve the position solution
    ZenGnssFixCarrierPhaseSolution carrierPhaseSolution;

    /// the number of satellites that have been used to
    /// compute the position
    uint8_t numberSatellitesUsed;

    /// Year in UTC
    uint16_t year;

    /// Month in UTC
    uint8_t month;

    /// Day in UTC
    uint8_t day;

    /// Hour in UTC
    uint8_t hour;

    /// Minute in UTC
    uint8_t minute;

    /// Second in UTC, exact time rounded to the nearest second. See below.
    uint8_t second;

    /// All the date and time values above are rounded
    /// so they can be represented as integeres. This
    /// is the time in nanoseconds that the above date & time values need
    /// to be shifted to arrive at the exact time measured by the GNSS receiver.
    int32_t nanoSecondCorrection;

} ZenGnssData;

typedef ZenGnssData ZenEventData_Gnss;

typedef struct ZenSensorDesc
{
    /**
    User-readable name of the sensor device
    */
    char name[256];

    /**
    Hardware serial number of the sensor. If the IO subsystem
    cannot read of the serial number, this can be another form
    of identification number.
    */
    char serialNumber[64];

    /**
    The IO Subsystem name this sensor is connected by.
    */
    char ioType[64];

    /**
    This identifier holds the actual hardware address of the sensor
    and can be used by OpenZen to connect.
    */
    char identifier[64];

    /**
     baud rate to use with the device. A baud rate of 0 indicates
     that OpenZen should use the default baudrate or negotiagte a
     suitable baud rate with the device.
     */
    uint32_t baudRate;
} ZenSensorDesc;

typedef struct ZenEventData_SensorDisconnected
{
    ZenError_t error;
} ZenEventData_SensorDisconnected;

typedef ZenSensorDesc ZenEventData_SensorFound;

typedef struct ZenEventData_SensorListingProgress
{
    float progress;
    /* This variable is != zero if the search for sensors in complete */
    char complete;
} ZenEventData_SensorListingProgress;

typedef union
{
    ZenEventData_Imu imuData;
    ZenEventData_Gnss gnssData;
    ZenEventData_SensorDisconnected sensorDisconnected;
    ZenEventData_SensorFound sensorFound;
    ZenEventData_SensorListingProgress sensorListingProgress;
} ZenEventData;

typedef enum ZenEventType
{
    ZenEventType_None = 0,

    ZenEventType_SensorFound = 1,
    ZenEventType_SensorListingProgress = 2,
    ZenEventType_SensorDisconnected = 3,

    ZenEventType_ImuData = 100,

    ZenEventType_GnssData = 200,

    // Sensors are free to expose private events in this reserved region
    ZenEventType_SensorSpecific_Start = 1000,
    ZenEventType_SensorSpecific_End = 1999,

    // IMU Components are free to expose private events in this reserved region
    ZenEventType_ImuComponentSpecific_Start = 2000,
    ZenEventType_ImuComponentSpecific_End = 2999,

    // Components are free to expose private events in this reserved region
    ZenEventType_GnssComponentSpecific_Start = 3000,
    ZenEventType_GnssComponentSpecific_End = 3999,

    ZenEventType_Max
} ZenEventType;

typedef struct ZenEvent
{
    ZenEventType eventType;
    ZenSensorHandle_t sensor;
    ZenComponentHandle_t component;
    ZenEventData data;
} ZenEvent;

typedef int ZenProperty_t;

typedef enum EZenSensorProperty
{
    ZenSensorProperty_Invalid = 0,

    ZenSensorProperty_DeviceName = 1000,         // byte[16]
    ZenSensorProperty_FirmwareInfo,              // byte[16]
    ZenSensorProperty_FirmwareVersion,           // int[3]
    ZenSensorProperty_SerialNumber,              // byte[24]
    ZenSensorProperty_RestoreFactorySettings,    // void
    ZenSensorProperty_StoreSettingsInFlash,      // void

    ZenSensorProperty_BatteryCharging,           // bool
    ZenSensorProperty_BatteryLevel,              // float
    ZenSensorProperty_BatteryVoltage,            // float

    ZenSensorProperty_BaudRate,                  // int
    ZenSensorProperty_SupportedBaudRates,        // int[]

    ZenSensorProperty_DataMode,                  // int (0: 32-bit float, 1: 16-bit fixed)
    ZenSensorProperty_TimeOffset,                // int

    ZenSensorProperty_SensorModel,               // byte[24]

    // Sensors are free to expose private properties in this reserved region
    ZenSensorProperty_SensorSpecific_Start = 10000,
    ZenSensorProperty_SensorSpecific_End = 19999,

    ZenSensorProperty_Max
} EZenSensorProperty;

typedef enum EZenImuProperty
{
    ZenImuProperty_Invalid = 0,

    ZenImuProperty_StreamData = 1000,           // bool
    ZenImuProperty_SamplingRate,                // int
    ZenImuProperty_SupportedSamplingRates,      // int[]

    ZenImuProperty_PollSensorData,               // void - Manually request sensor data (when not streaming)
    ZenImuProperty_CalibrateGyro,                // void - Start gyro calibration
    ZenImuProperty_ResetOrientationOffset,       // void - Resets the orientation's offset

    ZenImuProperty_CentricCompensationRate,      // float
    ZenImuProperty_LinearCompensationRate,       // float

    ZenImuProperty_FieldRadius,                  // float
    ZenImuProperty_FilterMode,                   // int
    ZenImuProperty_SupportedFilterModes,         // byte[]
    ZenImuProperty_FilterPreset,                 // int (future: float acc_covar, mag_covar)

    ZenImuProperty_OrientationOffsetMode,        // int

    ZenImuProperty_AccAlignment,                 // float[9]
    ZenImuProperty_AccBias,                      // float[3]
    ZenImuProperty_AccRange,                     // int
    ZenImuProperty_AccSupportedRanges,           // int[]

    ZenImuProperty_GyrAlignment,                 // float[9]
    ZenImuProperty_GyrBias,                      // float[3]
    ZenImuProperty_GyrRange,                     // int
    ZenImuProperty_GyrSupportedRanges,           // int[]
    ZenImuProperty_GyrUseAutoCalibration,        // bool
    ZenImuProperty_GyrUseThreshold,              // bool

    ZenImuProperty_MagAlignment,                 // float[9]
    ZenImuProperty_MagBias,                      // float[3]
    ZenImuProperty_MagRange,                     // int
    ZenImuProperty_MagSupportedRanges,           // int[]
    ZenImuProperty_MagReference,                 // float[3]
    ZenImuProperty_MagHardIronOffset,            // float[3]
    ZenImuProperty_MagSoftIronMatrix,            // float[9]

    ZenImuProperty_OutputLowPrecision,           // bool
    ZenImuProperty_OutputRawAcc,                 // bool
    ZenImuProperty_OutputRawGyr,                 // bool
    ZenImuProperty_OutputRawMag,                 // bool
    ZenImuProperty_OutputEuler,                  // bool
    ZenImuProperty_OutputQuat,                   // bool
    ZenImuProperty_OutputAngularVel,             // bool
    ZenImuProperty_OutputLinearAcc,              // bool
    ZenImuProperty_OutputHeaveMotion,            // bool
    ZenImuProperty_OutputAltitude,               // bool
    ZenImuProperty_OutputPressure,               // bool
    ZenImuProperty_OutputTemperature,            // bool

    /* new Ig1 properties */
    ZenImuProperty_OutputAccCalibrated,          // bool
    ZenImuProperty_OutputRawGyr0,                // bool
    ZenImuProperty_OutputRawGyr1,                // bool
    ZenImuProperty_OutputGyr0BiasCalib,          // bool
    ZenImuProperty_OutputGyr1BiasCalib,          // bool
    ZenImuProperty_OutputGyr0AlignCalib,         // bool
    ZenImuProperty_OutputGyr1AlignCalib,         // bool
    ZenImuProperty_OutputMagCalib,               // bool

    /* switch between degrees or radian output (only for IG1 and newer) */
    /* If set to true, radians will be used as output unit */
    /* If set to false, degrees will be used as output unit */
    /* OpenZen will automatically convert to degrees in case the unit is set to */
    /* before outputting values to the user.*/
    ZenImuProperty_DegRadOutput,                 // bool

    /* CAN bus properties */
    ZenImuProperty_CanChannelMode,
    ZenImuProperty_CanPointMode,
    ZenImuProperty_CanStartId,
    ZenImuProperty_CanBaudrate,
    ZenImuProperty_CanMapping,
    ZenImuProperty_CanHeartbeat,

    /* UART output properties */
    ZenImuProperty_UartBaudRate,
    ZenImuProperty_UartFormat,

    /* Sensor sync commands */
    ZenImuProperty_StartSensorSync,              // void - send start sensor sync command
    ZenImuProperty_StopSensorSync,               // void - send stop sensor sync command

    ZenImuProperty_Max
} EZenImuProperty;

typedef enum EZenGnssProperty
{
    ZenGnssProperty_Invalid = 0,

    ZenGnssProperty_OutputNavPvtiTOW,
    ZenGnssProperty_OutputNavPvtYear,
    ZenGnssProperty_OutputNavPvtMonth,
    ZenGnssProperty_OutputNavPvtDay,
    ZenGnssProperty_OutputNavPvtHour,
    ZenGnssProperty_OutputNavPvtMinute,
    ZenGnssProperty_OutputNavPvtSecond,
    ZenGnssProperty_OutputNavPvtValid,
    ZenGnssProperty_OutputNavPvttAcc,
    ZenGnssProperty_OutputNavPvtNano,
    ZenGnssProperty_OutputNavPvtFixType,
    ZenGnssProperty_OutputNavPvtFlags,
    ZenGnssProperty_OutputNavPvtFlags2,
    ZenGnssProperty_OutputNavPvtNumSV,
    ZenGnssProperty_OutputNavPvtLongitude,
    ZenGnssProperty_OutputNavPvtLatitude,
    ZenGnssProperty_OutputNavPvtHeight,
    ZenGnssProperty_OutputNavPvthMSL,
    ZenGnssProperty_OutputNavPvthAcc,
    ZenGnssProperty_OutputNavPvtvAcc,
    ZenGnssProperty_OutputNavPvtVelN,
    ZenGnssProperty_OutputNavPvtVelE,
    ZenGnssProperty_OutputNavPvtVelD,
    ZenGnssProperty_OutputNavPvtgSpeed,
    ZenGnssProperty_OutputNavPvtHeadMot,
    ZenGnssProperty_OutputNavPvtsAcc,
    ZenGnssProperty_OutputNavPvtHeadAcc,
    ZenGnssProperty_OutputNavPvtpDOP,
    ZenGnssProperty_OutputNavPvtHeadVeh,

    ZenGnssProperty_OutputNavAttiTOW,
    ZenGnssProperty_OutputNavAttVersion,
    ZenGnssProperty_OutputNavAttRoll,
    ZenGnssProperty_OutputNavAttPitch,
    ZenGnssProperty_OutputNavAttHeading,
    ZenGnssProperty_OutputNavAttAccRoll,
    ZenGnssProperty_OutputNavAttAccPitch,
    ZenGnssProperty_OutputNavAttAccHeading,

    ZenGnssProperty_OutputEsfStatusiTOW,
    ZenGnssProperty_OutputEsfStatusVersion,
    ZenGnssProperty_OutputEsfStatusInitStatus1,
    ZenGnssProperty_OutputEsfStatusInitStatus2,
    ZenGnssProperty_OutputEsfStatusFusionMode,
    ZenGnssProperty_OutputEsfStatusNumSens,
    ZenGnssProperty_OutputEsfStatusSensStatus,

    ZenGnssProperty_Max
} EZenGnssProperty;

typedef enum ZenOrientationOffsetMode
{
    ZenOrientationOffsetMode_Object = 0,     // Object mode
    ZenOrientationOffsetMode_Heading = 1,    // Heading mode
    ZenOrientationOffsetMode_Alignment = 2,  // Alignment mode

    ZenOrientationOffsetMode_Max
} ZenOrientationOffsetMode;

typedef enum ZenPropertyType
{
    ZenPropertyType_Invalid = 0,

    ZenPropertyType_Byte = 1,
    ZenPropertyType_Bool = 2,
    ZenPropertyType_Float = 3,
    ZenPropertyType_Int32 = 4,
    ZenPropertyType_UInt64 = 5,

    ZenPropertyType_Max
} ZenPropertyType;

static const char g_zenSensorType_Imu[] = "imu";
static const char g_zenSensorType_Gnss[] = "gnss";

#endif
