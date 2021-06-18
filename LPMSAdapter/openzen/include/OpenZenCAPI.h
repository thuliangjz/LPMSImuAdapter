//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ZEN_API_OPENZENCAPI_H_
#define ZEN_API_OPENZENCAPI_H_

#include <stdbool.h>
#include <stddef.h>

#include "ZenTypes.h"

#ifdef __cplusplus
// disable C++ name mangling
extern "C" {
#endif

    /**
    This is the C API for the OpenZen library. If your project is compiled with C++, consider
    using the C++ interface in the file OpenZen.h
    */

    /**
    This function initializes the OpenZen system. It needs to be called before
    any other function call to the libray is performed
    @param outHandle pointer to a ZenClient handle. This handle will be set by the
                     ZenInit call and needs to be passed to all calls to OpenZen
                     library functions.
    */
    ZEN_API ZenError ZenInit(ZenClientHandle_t* outHandle);

    /**
    Releases the OpenZen library. This method needs to be called after the usage
    of the library is finished
    @param handle OpenZen handle obtained by ZenInit function.
    */
    ZEN_API ZenError ZenShutdown(ZenClientHandle_t handle);

    /**
    Set the level of log messages that OpenZen writes to the log sink. Currently,
    the only log sink is the console.
    */
    ZEN_API ZenError ZenSetLogLevel(ZenLogLevel logLevel);

    /** Opts in to an asynchronous process that lists available sensors.
     * ZenEventData_SensorListingProgress events will be queued to indicate progress.
     * ZenEventData_SensorFound events will be queued to signal sensor descriptions.
     */
    ZEN_API ZenError ZenListSensorsAsync(ZenClientHandle_t handle);

    /** Obtain a sensor from the client */
    ZEN_API ZenSensorInitError ZenObtainSensor(ZenClientHandle_t clientHandle,
        const ZenSensorDesc* const desc,
        ZenSensorHandle_t* outSensorHandle);

    /** Obtain a sensor from the client giving directly the IO sub-system and sensor identifier.
        This method can be called without listing the sensor first with a call to ZenListSensorsAsync
        if the ioType and identifier of a sensor is known.
    @param ioType name of the IoType the sensor is connected on. Possible options are
                  Bluetooth, SiUsb, WindowsDevice for windows COM port
                  and LinuxDevive for Linux COM port
    @param sensorIdentifier Unique identifier, you can obtain this identifier by listing the
                            sensors with OpenZen.
    @param baudRate For some IO interfaces, a specific baud rate needs to be provided (esp. COM port
                    interfaces). For other interfaces or if you want to use the default baud rate,
                    give 0 for this parameter.
    @param outSensorHandle Outputs the obtained sensor handle.
    */
    ZEN_API ZenSensorInitError ZenObtainSensorByName(ZenClientHandle_t clientHandle,
        const char * ioType,
        const char * sensorIdentifier,
        uint32_t baudRate,
        ZenSensorHandle_t* outSensorHandle);

    /** Release a sensor from the client */
    ZEN_API ZenError ZenReleaseSensor(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle);

    /** Returns true and fills the next event on the queue if there is one, otherwise returns false. */
    ZEN_API bool ZenPollNextEvent(ZenClientHandle_t handle, ZenEvent* const outEvent);

    /** Returns true and fills the next event on the queue when there is a new one, otherwise returns false upon a call to ZenShutdown() */
    ZEN_API bool ZenWaitForNextEvent(ZenClientHandle_t handle, ZenEvent* const outEvent);

    /** Publish all data events encountered by OpenZen over a network interface */
    ZEN_API ZenError ZenPublishEvents(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, const char* endpoint);

    /** If successful, directs the outComponents pointer to a list of sensor components and sets its length to outLength, otherwise, returns an error.
     * If the type variable points to a string, only components of that type are returned. If it is a nullptr, all components are returned, irrespective of type.
     */
    ZEN_API ZenError ZenSensorComponents(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, const char* const type, ZenComponentHandle_t** outComponentHandles, size_t* const outLength);

    /** Returns the sensor component by its type and number. If no sensor component with that type and number is available, outComponentHandle will be null.
     *  Numbering starts at 0. This function is especially used to keep the library wrapper SWIG interface easier.
     */
    ZEN_API ZenError ZenSensorComponentsByNumber(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, const char* const type, size_t number, ZenComponentHandle_t* outComponentHandle);

    /** Returns the sensor's IO type */
    ZEN_API const char* ZenSensorIoType(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle);

    /** Returns whether the sensor is equal to the sensor description */
    ZEN_API bool ZenSensorEquals(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, const ZenSensorDesc* const desc);

    /** On first call, tries to initialises a firmware update, and returns an error on failure.
     * Subsequent calls do not require a valid buffer and buffer size, and only report the current status:
     * Returns ZenAsync_Updating while busy updating firmware.
     * Returns ZenAsync_Finished once the entire firmware has been written to the sensor.
     * Returns ZenAsync_Failed if an error has occurred while updating.
     */
    ZEN_API ZenAsyncStatus ZenSensorUpdateFirmwareAsync(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, const unsigned char* const buffer, size_t bufferSize);

    /** On first call, tries to initialise an IAP update, and returns an error on failure.
     * Subsequent calls do not require a valid buffer and buffer size, and only report the current status:
     * Returns ZenAsync_Updating while busy updating IAP.
     * Returns ZenAsync_Finished once the entire IAP has been written to the sensor.
     * Returns ZenAsync_Failed if an error has occurred while updating.
     */
    ZEN_API ZenAsyncStatus ZenSensorUpdateIAPAsync(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, const unsigned char* const buffer, size_t bufferSize);

    /** If successful executes the property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorExecuteProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property);

    /** If successful fills the buffer with the array of properties and sets the buffer's size.
     * Otherwise, returns an error and potentially sets the desired buffer size - if it is too small.
     */
    ZEN_API ZenError ZenSensorGetArrayProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, ZenPropertyType type, void* const buffer, size_t* bufferSize);

    /** If successful fills the value with the property's boolean value, otherwise returns an error. */
    ZEN_API ZenError ZenSensorGetBoolProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, bool* const outValue);

    /** If successful fills the value with the property's floating-point value, otherwise returns an error. */
    ZEN_API ZenError ZenSensorGetFloatProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, float* const outValue);

    /** If successful fills the value with the property's signed integer value, otherwise returns an error. */
    ZEN_API ZenError ZenSensorGetInt32Property(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, int32_t* const outValue);

    /** If successful fills the value with the property's unsigned integer value, otherwise returns an error. */
    ZEN_API ZenError ZenSensorGetUInt64Property(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, uint64_t* const outValue);

    /** If successful sets the array properties, otherwise returns an error. */
    ZEN_API ZenError ZenSensorSetArrayProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, ZenPropertyType type, const void* buffer, size_t bufferSize);

    /** If successful sets the boolean property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorSetBoolProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, bool value);

    /** If successful sets the floating-point property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorSetFloatProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, float value);

    /** If successful sets the integer property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorSetInt32Property(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, int32_t value);

    /** If successful sets the unsigned integer property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorSetUInt64Property(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property, uint64_t value);

    /** Returns whether the property is an array type */
    ZEN_API bool ZenSensorIsArrayProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property);

    /** Returns whether the property can be executed as a command */
    ZEN_API bool ZenSensorIsConstantProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property);

    /** Returns whether the property is constant. If so, the property cannot be set */
    ZEN_API bool ZenSensorIsExecutableProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property);

    /** Returns the type of the property */
    ZEN_API ZenPropertyType ZenSensorPropertyType(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenProperty_t property);

    /** Returns the type of the sensor component */
    ZEN_API const char* ZenSensorComponentType(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle);

    /** If successful executes the property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentExecuteProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property);

    /** If successful fills the buffer with the array of properties and sets the buffer's size.
     * Otherwise, returns an error and potentially sets the desired buffer size - if it is too small.
     */
    ZEN_API ZenError ZenSensorComponentGetArrayProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, ZenPropertyType type, void* const buffer, size_t* bufferSize);

    /** If successful fills the value with the property's boolean value, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentGetBoolProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, bool* const outValue);

    /** If successful fills the value with the property's floating-point value, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentGetFloatProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, float* const outValue);

    /** If successful fills the value with the property's signed integer value, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentGetInt32Property(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, int32_t* const outValue);

    /** If successful fills the value with the property's unsigned integer value, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentGetUInt64Property(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, uint64_t* const outValue);

    /** If successful sets the array properties, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentSetArrayProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, ZenPropertyType type, const void* buffer, size_t bufferSize);

    /** If successful sets the boolean property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentSetBoolProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, bool value);

    /** If successful sets the floating-point property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentSetFloatProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, float value);

    /** If successful sets the integer property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentSetInt32Property(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, int32_t value);

    /** If successful sets the unsigned integer property, otherwise returns an error. */
    ZEN_API ZenError ZenSensorComponentSetUInt64Property(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property, uint64_t value);

    /** Returns whether the property is an array type */
    ZEN_API bool ZenSensorComponentIsArrayProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property);

    /** Returns whether the property can be executed as a command */
    ZEN_API bool ZenSensorComponentIsConstantProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property);

    /** Returns whether the property is constant. If so, the property cannot be set */
    ZEN_API bool ZenSensorComponentIsExecutableProperty(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property);

    /** Returns the type of the property */
    ZEN_API ZenPropertyType ZenSensorComponentPropertyType(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle, ZenProperty_t property);

    /**
     * Starts forwarding the RTK-GPS corrections to the sensor.
     * This method call is only supported on components of type GNSS.
     *
     * Use this type of method call to forward from a network source:
     *
     *      ZenSensorComponentGnnsForwardRtkCorrections("RTCM3Network", "192.168.1.117", 9000)
     *
     * And this call to read RTK corrections directly from a local COM port
     *
     *      ZenSensorComponentGnnsForwardRtkCorrections("RTCM3Serial", "COM11", 57600)
     */
    ZEN_API ZenError ZenSensorComponentGnnsForwardRtkCorrections(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle,
        const char* const rtkCorrectionSource,
        const char* const hostname,
        uint32_t port);
#ifdef __cplusplus
}
#endif
#endif
