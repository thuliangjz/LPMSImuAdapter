//===========================================================================//
//
// Copyright (C) 2020 LP-Research Inc.
//
// This file is part of OpenZen, under the MIT License.
// See https://bitbucket.org/lpresearch/openzen/src/master/LICENSE for details
// SPDX-License-Identifier: MIT
//
//===========================================================================//

#include <OpenZen.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <limits>
#include <string>
#include <thread>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "LPMSAdapter.h"
#include <set>
#include <stddef.h>
#include <Windows.h>
std::vector<ZenSensorDesc> g_discoveredSensors;
std::condition_variable g_discoverCv;
std::mutex g_discoverMutex;
std::atomic_bool g_terminate(false);

std::atomic<uintptr_t> g_imuHandle;
std::atomic<uintptr_t> g_gnssHandle;

using namespace zen;
namespace
{
    void addDiscoveredSensor(const ZenEventData_SensorFound& desc)
    {
        std::lock_guard<std::mutex> lock(g_discoverMutex);
        g_discoveredSensors.push_back(desc);
    }
}

void pollLoop(std::reference_wrapper<ZenClient> client)
{
    int lastFrameCnt = -1;
    int frameTick = 0;
    while (!g_terminate)
    {
        unsigned int i = 0;
        int eventCnt = 0;
        while (true)
        {
            const auto pair = client.get().pollNextEvent();
            eventCnt++;
            const bool success = pair.first;
            auto& event = pair.second;
            if (!success)
                break;

            if (!event.component.handle)
            {
                switch (event.eventType)
                {
                case ZenEventType_SensorFound:
                    addDiscoveredSensor(event.data.sensorFound);
                    break;

                case ZenEventType_SensorListingProgress:
                    if (event.data.sensorListingProgress.progress == 1.0f)
                        g_discoverCv.notify_one();
                    break;
                }
            }
            else if ((g_imuHandle > 0) && (event.component.handle == g_imuHandle))
            {
                switch (event.eventType)
                {
                case ZenEventType_ImuData:
                    if (lastFrameCnt == -1) {
                        lastFrameCnt = event.data.imuData.frameCount;
                    }
                    else {
                        frameTick += event.data.imuData.frameCount - lastFrameCnt;
                        lastFrameCnt = event.data.imuData.frameCount;
                    }
                    if (i++ % 100 == 0) {
                        const auto& imuData = event.data.imuData;
                        std::cout << "Event type: " << event.eventType << std::endl;
                        std::cout << "> Event component: " << uint32_t(event.component.handle) << std::endl;
                        std::cout << "> Acceleration: \t x = " << event.data.imuData.a[0]
                            << "\t y = " << event.data.imuData.a[1]
                            << "\t z = " << event.data.imuData.a[2] << std::endl;
                        std::cout << "> Gyro: \t\t x = " << event.data.imuData.g[0]
                            << "\t y = " << event.data.imuData.g[1]
                            << "\t z = " << event.data.imuData.g[2] << std::endl;
                    }
                    break;
                }
            }
            else if ((g_gnssHandle > 0) && (event.component.handle == g_gnssHandle))
            {
                switch (event.eventType)
                {
                case ZenEventType_GnssData:
                    std::cout << "Event type: " << event.eventType << std::endl;
                    std::cout << "> Event component: " << uint32_t(event.component.handle) << std::endl;
                    std::cout << "> GPS Fix: \t = " << event.data.gnssData.fixType << std::endl;
                    std::cout << "> Longitude: \t = " << event.data.gnssData.longitude
                        << "   Latitude: \t = " << event.data.gnssData.latitude << std::endl;
                    std::cout << " > GPS Time " << int(event.data.gnssData.year) << "/"
                        << int(event.data.gnssData.month) << "/"
                        << int(event.data.gnssData.day) << " "
                        << int(event.data.gnssData.hour) << ":"
                        << int(event.data.gnssData.minute) << ":"
                        << int(event.data.gnssData.second) << " UTC" << std::endl;

                    if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_None) {
                        std::cout << " > RTK not used" << std::endl;
                    }
                    else if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_FloatAmbiguities) {
                        std::cout << " > RTK used with float ambiguities" << std::endl;
                    }
                    else if (event.data.gnssData.carrierPhaseSolution == ZenGnssFixCarrierPhaseSolution_FixedAmbiguities) {
                        std::cout << " > RTK used with fixed ambiguities" << std::endl;
                    }

                    break;
                }
            }
        }
    }

    std::cout << "--- Exit polling thread ---" << std::endl;
}

int client_example_main()
{
    ZenSetLogLevel(ZenLogLevel_Debug);

    g_imuHandle = 0;
    g_gnssHandle = 0;

    auto clientPair = make_client();
    auto& clientError = clientPair.first;
    auto& client = clientPair.second;

    if (clientError)
        return clientError;

    std::thread pollingThread(&pollLoop, std::ref(client));

    std::cout << "Listing sensors:" << std::endl;

    if (auto error = client.listSensorsAsync())
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return error;
    }

    std::unique_lock<std::mutex> lock(g_discoverMutex);
    g_discoverCv.wait(lock);

    if (g_discoveredSensors.empty())
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return ZenError_Unknown;
    }

    for (unsigned idx = 0; idx < g_discoveredSensors.size(); ++idx)
        std::cout << idx << ": " << g_discoveredSensors[idx].name << " (" << g_discoveredSensors[idx].ioType << ")" << std::endl;

    unsigned int idx;
    do
    {
        std::cout << "Provide an index within the range 0-" << g_discoveredSensors.size() - 1 << ":" << std::endl;
        std::cin >> idx;
    } while (idx >= g_discoveredSensors.size());
    //std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    auto sensorPair = client.obtainSensor(g_discoveredSensors[idx]);
    auto& obtainError = sensorPair.first;
    auto& sensor = sensorPair.second;
    if (obtainError)
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return obtainError;
    }

    auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
    auto& hasImu = imuPair.first;
    auto imu = imuPair.second;

    if (!hasImu)
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return ZenError_WrongSensorType;
    }

    // store the handle to the IMU to identify data coming from the imu
    // in our data processing thread
    g_imuHandle = imu.component().handle;

    // Get a string property
    auto sensorModelPair = sensor.getStringProperty(ZenSensorProperty_SensorModel);
    auto& sensorModelError = sensorModelPair.first;
    auto& sensorModelName = sensorModelPair.second;
    if (sensorModelError) {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return sensorModelError;
    }
    std::cout << "Sensor Model: " << sensorModelName << std::endl;

    // enable this to stream the sensor data to a network address
    // the ZEN_NETWORK build option needs to be enabled for this feature
    // to work.
    //sensor.publishEvents("tcp://*:8877");

    // check if a Gnss component is present on this sensor
    auto gnssPair = sensor.getAnyComponentOfType(g_zenSensorType_Gnss);
    auto& hasGnss = gnssPair.first;
    auto gnss = gnssPair.second;

    if (hasGnss)
    {
        // store the handle to the Gnss to identify data coming from the Gnss
        // in our data processing thread
        g_gnssHandle = gnss.component().handle;
        std::cout << "Gnss Component present on sensor" << std::endl;

        // enable this code for RTK forwarding from network
        /*
        if (gnss.forwardRtkCorrections("RTCM3Network", "192.168.1.117", 9000) != ZenError_None) {
            std::cout << "Cannot set RTK correction forwarding" << std::endl;
        }
        else {
            std::cout << "RTK correction forwarding started" << std::endl;
        }
        */

        // enable this code for RTK forwarding from serial port
        /*
        if (gnss.forwardRtkCorrections("RTCM3Serial", "COM11", 57600) != ZenError_None) {
            std::cout << "Cannot set RTK correction forwarding" << std::endl;
        }
        else {
            std::cout << "RTK correction forwarding started" << std::endl;
        }
        */
    }

    // Enable sensor streaming
    if (auto error = imu.setBoolProperty(ZenImuProperty_StreamData, true))
    {
        g_terminate = true;
        client.close();
        pollingThread.join();
        return error;
    }

    auto imuRatePair = imu.getInt32Property(ZenImuProperty_SamplingRate);
    if (!imuRatePair.first) {
        std::cout << "imu rate: " << imuRatePair.second << std::endl;
    }

    auto imuRateSupportedPair = imu.getArrayProperty<int>(ZenImuProperty_SupportedSamplingRates);
    if (!imuRateSupportedPair.first) {
        auto& supportedRateList = imuRateSupportedPair.second;
        std::cout << "supported rates: ";
        for (auto& rate : supportedRateList) {
            std::cout << "  " << rate;
        }
        std::cout << std::endl;
    }

    std::string line;
    while (!g_terminate)
    {
        std::cout << "Type: " << std::endl;
        std::cout << " - 'q' to quit;" << std::endl;
        std::cout << " - 'r' to manually release the sensor;" << std::endl;

        if (std::getline(std::cin, line))
        {
            if (line == "q")
                g_terminate = true;
            else if (line == "r")
                sensor.release();
        }
    }

    client.close();
    pollingThread.join();
    return 0;
}

struct VectorPack {
    double a[3];
    double w[3];
    double q[4];
};


int test_main() {
    //for (int i = 0; i < 100; ++i) {
    //    const auto p1 = std::chrono::system_clock::now();
    //    uint64_t microseconds_since_epoch = std::chrono::duration_cast<std::chrono::microseconds>(p1.time_since_epoch()).count();
    //    std::cout << "microsecond " << microseconds_since_epoch << std::endl;
    //}
    //std::set<std::string> s;
    //std::string strs[] = { "123", "456", "789" };
    //for (const auto& str : strs) {
    //    s.insert(str);
    //}
    //std::cout << s.size() << std::endl;
    
    //VectorPack vpk;
    //std::map<std::string, size_t> mpField2Pos;
    //mpField2Pos["a"] = offsetof(VectorPack, a);
    //mpField2Pos["w"] = offsetof(VectorPack, w);
    //mpField2Pos["q"] = offsetof(VectorPack, q);
    //std::map<std::string, size_t> mpField2Size;
    //mpField2Size["a"] = sizeof(vpk.a) / sizeof(*vpk.a);
    //mpField2Size["w"] = sizeof(vpk.w) / sizeof(*vpk.w);
    //mpField2Size["q"] = sizeof(vpk.q) / sizeof(*vpk.q);
    //char* p = reinterpret_cast<char*>(&vpk);
    //double d = 0;
    //for (auto& fp : mpField2Pos) {
    //    double* attrPos = reinterpret_cast<double*>(p + fp.second);
    //    size_t attrLen = mpField2Size[fp.first];
    //    for (size_t i = 0; i < attrLen; ++i) {
    //        attrPos[i] = d;
    //        d += 1;
    //    }
    //    std::cout << fp.first << ": " << Eigen::Map<Eigen::VectorXd>(attrPos, attrLen).transpose() << std::endl;
    //}
    //std::string field;
    //std::getline(std::cin, field);
    //if (mpField2Pos.count(field)) {
    //    double* attrPos = reinterpret_cast<double*>(p + mpField2Pos[field]);
    //    std::cout << Eigen::Map<Eigen::VectorXd>(attrPos, mpField2Size[field]).transpose() << std::endl;
    //}
    
    //size_t row = 3, col = 5;
    //char* p = new char[row * col * sizeof(double)];
    //Eigen::Map<Eigen::Matrix<double, -1, -1, Eigen::RowMajor>> m(reinterpret_cast<double*>(p), row, col);
    //for (size_t i = 0; i < row; i++) {
    //    for (size_t j = 0; j < col; j++) {
    //        m(i, j) = i + j;
    //    }
    //}
    //std::cout << m << std::endl;
    double a = 0.1;
    uint64_t b = static_cast<uint64_t>(a * 1e6);
    std::cout << b << std::endl;
    return 0;
}


int adapter_main() {
    int imuCnt = 1;
    LPMSAdapter adapter(imuCnt);
    try {
		adapter.start();
    }
    catch (std::string msg) {
        std::cout << "error when starting adapter: " << msg << std::endl;
        return -1;
    }
	std::string field = "a";
    size_t imuIdx = 0;
	const size_t col = 3;
    for (int i = 0; i < 100; ++i) {
        Sleep(50);
        try {
			LPMSImuDataList dataList = adapter.getImuBeforeTime(getMachineTimestamp());
			for (int j = 0; j < imuCnt; ++j) {
				size_t row = dataList.getRecordCnt(j);
                std::cout << "row cnt: " << row << std::endl;
				const float* const acc = dataList.retrieveFloatData(j, field.c_str(), row, col);
				if (acc == nullptr)
					continue;
				const Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> m(acc, row, col);
				std::cout << "imu " << j << ": " << m.block<1, col>(row-2, 0)  << std::endl;
			}
        }
        catch (std::string msg) {
            std::cout << "error retriving: " << msg << std::endl;
        }
        
    }
    adapter.stop();
    return 0;
}

int main() {
    return adapter_main();
}