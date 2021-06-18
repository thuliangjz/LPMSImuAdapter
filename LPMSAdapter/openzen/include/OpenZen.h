//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#ifndef ZEN_API_OPENZEN_H_
#define ZEN_API_OPENZEN_H_

/**
This is the C++ API to the OpenZen library. It is a header-only wrapper around
the C-API.

Depending on your chosen version of C++, this will be a C++14 or C++17 interface.
See below how to override the default choice.

Use the zen::make_client function to create a ZenClient object which you can then
use to list all available sensors with the ZenClient::listSensorsAsync() method. With all
available sensors, you can then use the ZenClient::obtainSensor() method to
connect to a sensor and receive its measurement data.

You can use the waitForNextEvent() and pollNextEvent() of the ZenClient class to get
ZenEvents about sensor discovery results and incoming measurement data.
*/

#include "OpenZenCAPI.h"

// Decide whether to use the C++17 or C++14 API.
// The user can force C++14 even if compiling with C++17 by defining OPENZEN_CXX14.
// Also, C++17 can be forced by defining OPENZEN_CXX17
//
// Visual C++ defines __cplusplus to 199807L unless /Zc:__cplusplus is given,
// so we need to treat it as a special case.
#if (__cplusplus >= 201703L || (defined(_MSVC_LANG) && _MSVC_LANG >= 201703L)) && !defined(OPENZEN_CXX14)
#ifndef OPENZEN_CXX17
#define OPENZEN_CXX17
#endif
#endif

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <array>

#ifdef OPENZEN_CXX17
#include <optional>
#include <string_view>
#endif

inline bool operator==(const ZenSensorHandle& lhs, const ZenSensorHandle& rhs) {
    return lhs.handle == rhs.handle;
}

inline bool operator==(const ZenClientHandle& lhs, const ZenClientHandle& rhs) {
    return lhs.handle == rhs.handle;
}

inline bool operator==(const ZenComponentHandle& lhs, const ZenComponentHandle& rhs) {
    return lhs.handle == rhs.handle;
}

namespace details
{
    template <typename T>
    struct PropertyType
    {};

#ifdef OPENZEN_CXX17
    template <> struct PropertyType<std::byte>
#else
    template <> struct PropertyType<unsigned char>
#endif
    {
        using type = std::integral_constant<ZenPropertyType, ZenPropertyType_Byte>;
    };

    template <> struct PropertyType<bool>
    {
        using type = std::integral_constant<ZenPropertyType, ZenPropertyType_Bool>;
    };

    template <> struct PropertyType<float>
    {
        using type = std::integral_constant<ZenPropertyType, ZenPropertyType_Float>;
    };

    template <> struct PropertyType<int32_t>
    {
        using type = std::integral_constant<ZenPropertyType, ZenPropertyType_Int32>;
    };

    template <> struct PropertyType<uint64_t>
    {
        using type = std::integral_constant<ZenPropertyType, ZenPropertyType_UInt64>;
    };
}

namespace zen
{
    class ZenClient;

    /**
    A sensor component represents one measurement data source on a sensor, for example an
    inertial measurement unit (IMU) or GPS receiver.
    */
    class ZenSensorComponent
    {
        friend class ZenSensor;

    private:
        ZenClientHandle_t m_clientHandle;
        ZenSensorHandle_t m_sensorHandle;
        ZenComponentHandle_t m_componentHandle;

        static constexpr size_t m_getArrayBufferSize = 9;

    protected:
        ZenSensorComponent(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle, ZenComponentHandle_t componentHandle) noexcept
            : m_clientHandle(clientHandle)
            , m_sensorHandle(sensorHandle)
            , m_componentHandle(componentHandle)
        {}

    public:
        ZenSensorComponent() {
            m_clientHandle.handle = 0;
            m_sensorHandle.handle = 0;
            m_componentHandle.handle = 0;
        }

        ZenSensorComponent(const ZenSensorComponent& other) noexcept
            : m_clientHandle(other.m_clientHandle)
            , m_sensorHandle(other.m_sensorHandle)
            , m_componentHandle(other.m_componentHandle)
        {}

        /**
         * Return the sensor handle this object in referencing to
         */
        ZenSensorHandle_t sensor() const noexcept
        {
            return m_sensorHandle;
        }

        /**
         * Return the component handle this object in referencing to
         */
        ZenComponentHandle_t component() const noexcept
        {
            return m_componentHandle;
        }

        /**
         * Returns the type name of this component. At this point, this method
         * will return either g_zenSensorType_Imu or g_zenSensorType_Gnss
         */
        std::string type() const noexcept
        {
            return ZenSensorComponentType(m_clientHandle, m_sensorHandle, m_componentHandle);
        }

        /**
         * Triggers the execution of a property that supports that feature, for example the
         * ZenImuProperty_CalibrateGyro property to start the Gyro calibration.
         */
        ZenError executeProperty(ZenProperty_t property) noexcept
        {
            return ZenSensorComponentExecuteProperty(m_clientHandle, m_sensorHandle, m_componentHandle, property);
        }

        /**
         * Loads an array property from this sensor component
         */
        template <class TDataType>
        std::pair<ZenError, std::vector<TDataType>> getArrayProperty(ZenProperty_t property) noexcept
        {
            std::vector<TDataType> outputArray(m_getArrayBufferSize);

            size_t outputSize = outputArray.size() * sizeof(TDataType);
            auto error = ZenSensorComponentGetArrayProperty(m_clientHandle, m_sensorHandle, m_componentHandle,
                property,
                details::PropertyType<TDataType>::type::value,
                outputArray.data(), &outputSize);
            outputSize /= sizeof(TDataType);
            // resize output size
            outputArray.resize(outputSize);
            return {error, outputArray};
        }

        /**
         * Loads a bool property from this sensor component
         */
        std::pair<ZenError, bool> getBoolProperty(ZenProperty_t property) noexcept
        {
            auto result = std::make_pair(ZenError_None, false);
            result.first = ZenSensorComponentGetBoolProperty(m_clientHandle, m_sensorHandle, m_componentHandle, property, &result.second);
            return result;
        }

        /**
         * Loads a float property from this sensor component
         */
        std::pair<ZenError, float> getFloatProperty(ZenProperty_t property) noexcept
        {
            auto result = std::make_pair(ZenError_None, 0.f);
            result.first = ZenSensorComponentGetFloatProperty(m_clientHandle, m_sensorHandle, m_componentHandle, property, &result.second);
            return result;
        }

        /**
         * Loads a int32 property from this sensor component
         */
        std::pair<ZenError, int32_t> getInt32Property(ZenProperty_t property) noexcept
        {
            auto result = std::make_pair(ZenError_None, 0);
            result.first = ZenSensorComponentGetInt32Property(m_clientHandle, m_sensorHandle, m_componentHandle, property, &result.second);
            return result;
        }

        /**
         * Loads an uint64 property from this sensor component
         */
        std::pair<ZenError, uint64_t> getUInt64Property(ZenProperty_t property) noexcept
        {
            auto result = std::make_pair(ZenError_None, uint64_t(0));
            result.first = ZenSensorComponentGetUInt64Property(m_clientHandle, m_sensorHandle, m_componentHandle, property, &result.second);
            return result;
        }

        /**
         * Sets an array property on this sensor component
         */
        template <typename TDataType>
        ZenError setArrayProperty(ZenProperty_t property, std::vector<TDataType> & inputArray) noexcept
        {
            return ZenSensorComponentSetArrayProperty(m_clientHandle, m_sensorHandle, m_componentHandle, property,
                details::PropertyType<TDataType>::type::value, inputArray.data(),
                    inputArray.size() * sizeof(TDataType));
        }

        /**
         * Sets a bool property on this sensor component
         */
        ZenError setBoolProperty(ZenProperty_t property, bool value) noexcept
        {
            return ZenSensorComponentSetBoolProperty(m_clientHandle, m_sensorHandle, m_componentHandle, property, value);
        }

        /**
         * Sets a float property on this sensor component
         */
        ZenError setFloatProperty(ZenProperty_t property, float value) noexcept
        {
            return ZenSensorComponentSetFloatProperty(m_clientHandle, m_sensorHandle, m_componentHandle, property, value);
        }

        /**
         * Sets an int32 property on this sensor component
         */
        ZenError setInt32Property(ZenProperty_t property, int32_t value) noexcept
        {
            return ZenSensorComponentSetInt32Property(m_clientHandle, m_sensorHandle, m_componentHandle, property, value);
        }

        /**
         * Sets an uint64 property on this sensor component
         */
        ZenError setUInt64Property(ZenProperty_t property, uint64_t value) noexcept
        {
            return ZenSensorComponentSetUInt64Property(m_clientHandle, m_sensorHandle, m_componentHandle, property, value);
        }

        /**
         * Starts forwarding the RTK-GPS corrections to the sensor.
         * This method call is only supported on components of type GNSS.
         *
         * Use this type of method call to forward from a network source:
         *
         *      component.forwardRtkCorrections("RTCM3Network", "192.168.1.117", 9000)
         *
         * And this call to read RTK corrections directly from a local COM port
         *
         *      component.forwardRtkCorrections("RTCM3Serial", "COM11", 57600)
         */
        ZenError forwardRtkCorrections(std::string const& rtkCorrectionSource,
            std::string const& hostname,
            uint32_t port) noexcept {
            return ZenSensorComponentGnnsForwardRtkCorrections(m_clientHandle, m_sensorHandle, m_componentHandle,
                rtkCorrectionSource.c_str(), hostname.c_str(), port);
        }
    };

    /**
    This class represents one sensor connected by OpenZen. One sensor can contain one or more
    components which deliver measurement data. Don't instantiate this class directly but use the
    ZenClient::obtainSensor() or ZenClient::obtainSensorByName() calls.
    */
    class ZenSensor
    {
        friend class ZenClient;

    private:
        ZenClientHandle_t m_clientHandle;
        ZenSensorHandle_t m_sensorHandle;

        static constexpr size_t m_getArrayBufferSize = 9;

    protected:
        ZenSensor(ZenClientHandle_t clientHandle, ZenSensorHandle_t sensorHandle)
            : m_clientHandle(clientHandle)
            , m_sensorHandle(sensorHandle)
        {}
    public:
        ZenSensor() : m_sensorHandle(ZenSensorHandle_t{0}) {
        }

        ZenSensor(ZenSensor&& other)
            : m_clientHandle(other.m_clientHandle)
            , m_sensorHandle(other.m_sensorHandle)
        {
            other.m_sensorHandle.handle = 0;
        }

        ~ZenSensor()
        {
            if (m_sensorHandle.handle != 0) {
                release();
            }
        }

        /**
         * Release the connected sensor. No further events from this sensor will
         * be transmitted.
         */
        ZenError release() noexcept
        {
            auto err = ZenReleaseSensor(m_clientHandle, m_sensorHandle);
            if (err == ZenError_None) {
                // invalidate sensor handle if closing was successful
                m_sensorHandle.handle = 0;
            }
            return err;
        }

        /** On first call, tries to initialises a firmware update, and returns an error on failure.
         * Subsequent calls do not require a valid buffer and buffer size, and only report the current status:
         * Returns ZenAsync_Updating while busy updating firmware.
         * Returns ZenAsync_Finished once the entire firmware has been written to the sensor.
         * Returns ZenAsync_Failed if an error has occurred while updating.
         */
        ZenAsyncStatus updateFirmwareAsync(const std::vector<unsigned char>& firmware) noexcept
        {
            return ZenSensorUpdateFirmwareAsync(m_clientHandle, m_sensorHandle, firmware.data(), firmware.size());
        }

        /** On first call, tries to initialise an IAP update, and returns an error on failure.
         * Subsequent calls do not require a valid buffer and buffer size, and only report the current status:
         * Returns ZenAsync_Updating while busy updating IAP.
         * Returns ZenAsync_Finished once the entire IAP has been written to the sensor.
         * Returns ZenAsync_Failed if an error has ocurred while updating.
         */
        ZenAsyncStatus updateIAPAsync(const std::vector<unsigned char>& iap) noexcept
        {
            return ZenSensorUpdateIAPAsync(m_clientHandle, m_sensorHandle, iap.data(), iap.size());
        }

        std::string ioType() const noexcept
        {
            return ZenSensorIoType(m_clientHandle, m_sensorHandle);
        }

        /**
         * Compare if a sensor description matches the sensor
         * this instance points to
         */
        bool equals(const ZenSensorDesc& desc) const noexcept
        {
            return ZenSensorEquals(m_clientHandle, m_sensorHandle, &desc);
        }

        /**
         * Return the sensor handle this object in referencing to
         */
        ZenSensorHandle_t sensor() const noexcept
        {
            return m_sensorHandle;
        }


        /**
         * Publish all data events from this sensor over a network interface
         */
        ZenError publishEvents(std::string const& endpoint) noexcept {
            return ZenPublishEvents(m_clientHandle, m_sensorHandle, endpoint.c_str());
        }

        /**
         * Execute a sensor property which supports to be executed
         */
        ZenError executeProperty(ZenProperty_t property) noexcept
        {
            return ZenSensorExecuteProperty(m_clientHandle, m_sensorHandle, property);
        }

        /**
         * Loads an array property from this sensor component
         */
        template <class TDataType>
        std::pair<ZenError, std::vector<TDataType>> getArrayProperty(ZenProperty_t property) noexcept
        {
            std::vector<TDataType> outputArray(m_getArrayBufferSize);

            size_t outputSize = outputArray.size() * sizeof(TDataType);
            auto error = ZenSensorGetArrayProperty(m_clientHandle, m_sensorHandle,
                property,
                details::PropertyType<TDataType>::type::value,
                outputArray.data(), &outputSize);
            outputSize /= sizeof(TDataType);
            // resize output size
            outputArray.resize(outputSize);
            return {error, outputArray};
        }
        /**
         * Loads a string property from this sensor component
         */
        std::pair<ZenError, std::string> getStringProperty(ZenProperty_t property) noexcept
        {
            std::array<unsigned char, 255> arrayString;
            size_t writtenBytes = 255;
            auto error = ZenSensorGetArrayProperty(m_clientHandle, m_sensorHandle, property, ZenPropertyType_Byte,
                arrayString.data(), &writtenBytes);
            if (error)
                return std::make_pair(error, "");

            std::string outputString;
            // make sure to honor the null termination
            size_t i = 0;
            for (auto ch : arrayString) {
                if (ch == 0)
                    break;
                if (i == writtenBytes)
                    break;
                outputString = outputString + (char)ch;
                i++;
            }

            return std::make_pair(error, outputString);
        }

        /**
         * Loads a bool property from this sensor component
         */
        std::pair<ZenError, bool> getBoolProperty(ZenProperty_t property) noexcept
        {
            auto result = std::make_pair(ZenError_None, false);
            result.first = ZenSensorGetBoolProperty(m_clientHandle, m_sensorHandle, property, &result.second);
            return result;
        }

        /**
         * Loads a float property from this sensor component
         */
        std::pair<ZenError, float> getFloatProperty(ZenProperty_t property) noexcept
        {
            auto result = std::make_pair(ZenError_None, 0.f);
            result.first = ZenSensorGetFloatProperty(m_clientHandle, m_sensorHandle, property, &result.second);
            return result;
        }

        /**
         * Loads an int32 property from this sensor component
         */
        std::pair<ZenError, int32_t> getInt32Property(ZenProperty_t property) noexcept
        {
            auto result = std::make_pair(ZenError_None, 0);
            result.first = ZenSensorGetInt32Property(m_clientHandle, m_sensorHandle, property, &result.second);
            return result;
        }

        /**
         * Loads an uint64 property from this sensor component
         */
        std::pair<ZenError, uint64_t> getUInt64Property(ZenProperty_t property) noexcept
        {
            auto result = std::make_pair(ZenError_None, uint64_t(0));
            result.first = ZenSensorGetUInt64Property(m_clientHandle, m_sensorHandle, property, &result.second);
            return result;
        }

        /**
         * Sets an array property for this sensor component
         */
        template <typename TDataType>
        ZenError setArrayProperty(ZenProperty_t property, std::vector<TDataType> & inputArray) noexcept
        {
            return ZenSensorSetArrayProperty(m_clientHandle, m_sensorHandle, property,
                details::PropertyType<TDataType>::type::value, inputArray.data(),
                    inputArray.size() * sizeof(TDataType));
        }

        /**
         * Sets a bool property for this sensor component
         */
        ZenError setBoolProperty(ZenProperty_t property, bool value) noexcept
        {
            return ZenSensorSetBoolProperty(m_clientHandle, m_sensorHandle, property, value);
        }

        /**
         * Sets a float property for this sensor component
         */
        ZenError setFloatProperty(ZenProperty_t property, float value) noexcept
        {
            return ZenSensorSetFloatProperty(m_clientHandle, m_sensorHandle, property, value);
        }

        /**
         * Sets an int32 property for this sensor component
         */
        ZenError setInt32Property(ZenProperty_t property, int32_t value) noexcept
        {
            return ZenSensorSetInt32Property(m_clientHandle, m_sensorHandle, property, value);
        }

        /**
         * Sets an uint64 property for this sensor component
         */
        ZenError setUInt64Property(ZenProperty_t property, uint64_t value) noexcept
        {
            return ZenSensorSetUInt64Property(m_clientHandle, m_sensorHandle, property, value);
        }

        /**
         * Returns an instance of a sensor component on this sensor. type can be either
         * g_zenSensorType_Imu or g_zenSensorType_Gnss. If a requested sensor component
         * is not available on a sensor, the bool entry of the std::pair is false.
         */
#ifdef OPENZEN_CXX17
        std::optional<ZenSensorComponent> getAnyComponentOfType(std::string const& type) noexcept
        {
            ZenComponentHandle_t* handles = nullptr;
            size_t nComponents;
            if (ZenSensorComponents(m_clientHandle, m_sensorHandle, type.c_str(), &handles, &nComponents) != ZenError_None)
                return std::nullopt;

            if (nComponents == 0)
                return std::nullopt;

            return ZenSensorComponent(m_clientHandle, m_sensorHandle, handles[0]);
        }
#else
        std::pair<bool, ZenSensorComponent> getAnyComponentOfType(std::string const& type) noexcept
        {
            ZenComponentHandle_t* handles = nullptr;
            size_t nComponents;
            if (auto error = ZenSensorComponents(m_clientHandle, m_sensorHandle, type.c_str(), &handles, &nComponents))
                return std::make_pair(false, ZenSensorComponent(m_clientHandle, m_sensorHandle, ZenComponentHandle_t{ 0 }));

            if (nComponents == 0)
                return std::make_pair(false, ZenSensorComponent(m_clientHandle, m_sensorHandle, ZenComponentHandle_t{ 0 }));

            return std::make_pair(true, ZenSensorComponent(m_clientHandle, m_sensorHandle, handles[0]));
        }
#endif
    };

    /**
    This class is the primary access point into the OpenZen library. Use the zen::make_client
    method to obtain an instance of this class.
    */
    class ZenClient
    {
    private:
        ZenClientHandle_t m_handle;

    public:
        ZenClient() noexcept : m_handle({0}) {
        }

        ZenClient(ZenClientHandle_t handle) noexcept
            : m_handle(handle)
        {}

        ZenClient(ZenClient&& other) noexcept
            : m_handle(other.m_handle)
        {
            other.m_handle.handle = 0;
        }

        ~ZenClient() noexcept
        {
            if (m_handle.handle != 0) {
                close();
            }
        }

        /**
         * Close connection to the OpenZen client. No additional sensor can be obtained via this ZenClient.
         */
        ZenError close() noexcept
        {
            auto err = ZenShutdown(m_handle);
            if (err == ZenError_None) {
                // invalidate client handle if closing was successful
                m_handle.handle = 0;
            }
            return err;
        }

        /** call the method ZenClient::listSensorsAsync to start the query for available sensors.
         * Depending on the IO systems, it can take a couple of seconds for the listing to be complete.
         * The ZenClient::listSensorsAsync method will return immediately and the information
         * on the found sensors will be send to ZenClient's event queue and can be retrieved
         * with calls to ZenClient::pollNextEvent or ZenClient::waitForNextEvent. You can either
         * do this on your applications main thread or use a background thread to retrieve the event
         * listing data.
         * The event types ZenSensorEvent_SensorListingProgress and ZenSensorEvent_SensorFound will contain
         * provide the progress of the listing and report if a sensor has been found.
         */
        ZenError listSensorsAsync() noexcept
        {
            return ZenListSensorsAsync(m_handle);
        }

        /**
         * Connect to a sensor with the ZenSensorDesc which was obtained via a call to listSensorsAsync
         */
        std::pair<ZenSensorInitError, ZenSensor> obtainSensor(const ZenSensorDesc& desc) noexcept
        {
            ZenSensorHandle_t sensorHandle;
            const auto error = ZenObtainSensor(m_handle, &desc, &sensorHandle);
            return std::make_pair(error, ZenSensor(m_handle, sensorHandle));
        }

        /**
         * Sensors can also connected directly if the IO system they are connected too and their name
         * is known already. Here, the method ZenClient::obtainSensorByName can be called with the
         * name of the IO system and the name of the sensor:
         *      // connect the sensor with the name lpmscu2000573 via the SiLabs USB IO System
         *      auto sensorPair = client.obtainSensorByName("SiUsb", "lpmscu2000573");
         */
        std::pair<ZenSensorInitError, ZenSensor> obtainSensorByName(const std::string& ioType,
            const std::string& identifier, uint32_t baudrate = 0) noexcept
        {
            ZenSensorHandle_t sensorHandle;
            const auto error = ZenObtainSensorByName(m_handle, ioType.c_str(), identifier.c_str(),
                baudrate, &sensorHandle);
            return std::make_pair(error, ZenSensor(m_handle, sensorHandle));
        }

        /**
         * Should be done via ZenSensor::release and this method will be removed in the future.
         */
        [[deprecated]]
        ZenError releaseSensor(ZenSensor& sensor) noexcept
        {
            if (auto error = ZenReleaseSensor(m_handle, sensor.m_sensorHandle))
                return error;

            sensor.m_sensorHandle.handle = 0;
            return ZenError_None;
        }

#ifdef OPENZEN_CXX17
        /**
         * Poll the next event from the queue of this ZenClient. This method will
         * return immediately. If no event is available on the queue, the std::optional
         * will be empty.
         */
        std::optional<ZenEvent> pollNextEvent() noexcept
#else
        /**
         * Poll the next event from the queue of this ZenClient. This method will
         * return immediately. If no event is available on the queue, bool entry
         * of the std::pair will be false.
         */
        std::pair<bool, ZenEvent> pollNextEvent() noexcept
#endif
        {
            ZenEvent event;
            if (ZenPollNextEvent(m_handle, &event))
            {
#ifdef OPENZEN_CXX17
                return event;
#else
                return std::make_pair(true, std::move(event));
#endif
            }

#ifdef OPENZEN_CXX17
            return std::nullopt;
#else
            return std::make_pair(false, std::move(event));
#endif
        }

#ifdef OPENZEN_CXX17
        /**
         * Wait for the next event from the queue of this ZenClient. This method will
         * return immediately if an entry is available. Otherwise the method call will
         * block until an event is available. If the sensor connection is released on
         * another thread, the method call will return.
         * If no event is available on the queue, the std::optional
         * will be empty.
         */
        std::optional<ZenEvent> waitForNextEvent() noexcept
#else
        /**
         * Wait for the next event from the queue of this ZenClient. This method will
         * return immediately if an entry is available. Otherwise the method call will
         * block until an event is available. If the sensor connection is released on
         * another thread, the method call will return.
         * If no event is available on the queue, bool entry
         * of the std::pair will be false.
         */
        std::pair<bool, ZenEvent> waitForNextEvent() noexcept
#endif
        {
            ZenEvent event;
            if (ZenWaitForNextEvent(m_handle, &event))
            {
#ifdef OPENZEN_CXX17
                return event;
#else
                return std::make_pair(true, std::move(event));
#endif
            }

#ifdef OPENZEN_CXX17
            return std::nullopt;
#else
            return std::make_pair(false, std::move(event));
#endif
        }
    };

    /**
    Use this function to create a ZenClient object. This instance of
    ZenClient can then be used to list all available sensors and connect
    to them.
    */
    inline std::pair<ZenError, ZenClient> make_client() noexcept
    {
        ZenClientHandle_t handle;
        const auto error = ZenInit(&handle);
        return std::make_pair(error, ZenClient(handle));
    }
}

#endif
