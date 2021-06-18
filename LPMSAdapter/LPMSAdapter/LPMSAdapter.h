#pragma once

#include "cppshrhelp.h"
#include <thread>
#include <OpenZen.h>
#include <vector>
#include <map>
#include <mutex>
#include <queue>
#include <set>

struct AdapterImuData {
	ZenImuData zenImuData;
	uint64_t machineTimestamp;
};

class  LPMSImuDataList {
	std::vector<std::vector<AdapterImuData>> vData;
	std::map<std::string, std::size_t> mpField2Size;
	std::map<std::string, std::size_t> mpField2Offset;
	std::map<std::string, std::string> mpField2Type;
	std::vector<std::map<std::string, char*>> vmpDerivedDataPointers;
	std::string lastError;
public:
	LPMSImuDataList(std::vector<std::vector<AdapterImuData>>&& data);	//not to be exposed in matlab
	DLL_EXPORT const float* const retrieveFloatData(size_t imu, char const* field, size_t row, size_t col);
	DLL_EXPORT const double* const retrieveDoubleData(size_t imu, char const* field, size_t row, size_t col);
	DLL_EXPORT const uint64_t* const retrieveUint64Data(size_t imu, char const* field, size_t row, size_t col);
	DLL_EXPORT size_t getRecordCnt(size_t imu);
	DLL_EXPORT size_t getFieldLength(char const* field);
	DLL_EXPORT char const* getFieldType(char const* field);
	DLL_EXPORT ~LPMSImuDataList();
	DLL_EXPORT char const* getLastError();
private:
	void throwNoFieldError();
	std::pair<bool, std::string> checkRetrieveRequest(size_t imu, char const* field, size_t row, size_t col, char const* type);
	template<typename T>
	const T* const retrieveDataPointer(size_t imu, char const* field, size_t row, size_t col, char const* type);
};


class LPMSAdapter {
	const int nImuCnt;
	std::thread readerThread;
	std::string lastError;
	std::unique_ptr<zen::ZenClient> pClient;
	zen::ZenClient client;
	std::map<uintptr_t, int> mpHandle2Idx;
	std::atomic_bool readerThreadQuitFlag;
	std::vector<std::queue<AdapterImuData>> vvImuData;
	std::mutex mtxImuData;
	std::vector<zen::ZenSensor> vSensors;
	std::vector<zen::ZenSensorComponent> vSensorComponents;
	bool readerThreadStarted;
public:
	DLL_EXPORT LPMSAdapter(int imuCnt);
	DLL_EXPORT void start();
	DLL_EXPORT char const* getLastError();
	DLL_EXPORT LPMSImuDataList getImuBeforeTime(uint64_t stamp);
	DLL_EXPORT void stop();
	DLL_EXPORT AdapterImuData getSingleImuData(int idx);	//not to be exposed

	DLL_EXPORT ~LPMSAdapter();
private:
	void readerThreadFunction();
};

DLL_EXPORT uint64_t getMachineTimestamp();
