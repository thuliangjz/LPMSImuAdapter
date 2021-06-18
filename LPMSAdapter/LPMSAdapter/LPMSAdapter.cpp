#include "pch.h"
#include "LPMSAdapter.h"
#include <chrono>
#include <OpenZenCAPI.h>
#include <iostream>
#include <stddef.h>
#include <Eigen/Dense>
using std::chrono::duration_cast;
using std::chrono::microseconds;
using std::chrono::system_clock;
DLL_EXPORT uint64_t getMachineTimestamp() {
	return duration_cast<microseconds>(system_clock::now().time_since_epoch()).count();
}

uint64_t secondToMicroSecond(double timestamp) {
	return static_cast<uint64_t>(timestamp * 1e6);
}

LPMSImuDataList::LPMSImuDataList(std::vector<std::vector<AdapterImuData>>&& data):vData(std::move(data)){
	vmpDerivedDataPointers.resize(vData.size());
	//available attributes:  a, w, q, timestamp, machine_timestamp
	AdapterImuData entry;
	mpField2Type["a"] = "float";
	mpField2Type["w"] = "float";
	mpField2Type["q"] = "float";
	mpField2Type["timestamp"] = "double";
	mpField2Type["machine_timestamp"] = "uint64";

	mpField2Size["a"] = sizeof(entry.zenImuData.a) / sizeof(*entry.zenImuData.a);
	mpField2Size["w"] = sizeof(entry.zenImuData.w) / sizeof(*entry.zenImuData.w);
	mpField2Size["q"] = sizeof(entry.zenImuData.q) / sizeof(*entry.zenImuData.q);
	mpField2Size["timestamp"] = 1;
	mpField2Size["machine_timestamp"] = 1;
	
	mpField2Offset["a"] = offsetof(AdapterImuData, zenImuData.a);
	mpField2Offset["w"] = offsetof(AdapterImuData, zenImuData.w);
	mpField2Offset["q"] = offsetof(AdapterImuData, zenImuData.q);
	mpField2Offset["timestamp"] = offsetof(AdapterImuData, zenImuData.timestamp);
	mpField2Offset["machine_timestamp"] = offsetof(AdapterImuData, machineTimestamp);
}

LPMSImuDataList::~LPMSImuDataList() {
	for (auto& mp : vmpDerivedDataPointers) {
		for (auto& p : mp) {
			delete[]p.second;
		}
	}
}

size_t LPMSImuDataList::getRecordCnt(size_t imu) {
	if (imu > vData.size()) {
		lastError = "NoImuIndex";
		throw lastError;
	}
	return vData[imu].size();
}

size_t LPMSImuDataList::getFieldLength(char const* field) {
	if (mpField2Size.count(field)) {
		return mpField2Size[field];
	}
	throwNoFieldError();
}

char const* LPMSImuDataList::getFieldType(char const* field) {
	if (mpField2Type.count(field)) {
		return mpField2Type[field].c_str();
	}
	throwNoFieldError();
}

void LPMSImuDataList::throwNoFieldError() {
	lastError = "NoField";
	throw lastError;
}

std::pair<bool, std::string> LPMSImuDataList::checkRetrieveRequest(size_t imu, char const* field, size_t row, size_t col, char const* type) {
	if (!mpField2Type.count(field)) {
		return std::make_pair(false, "NoField");
	}
	if (mpField2Type[field] != type) {
		return std::make_pair(false, "TypeNotMatch");
	}
	if (imu >= vData.size()) {
		return std::make_pair(false, "NoImuIndex");
	}
	if (col != mpField2Size[field]) {
		return std::make_pair(false, "ColMismatch");
	}
	const std::vector<AdapterImuData>& vImuData = vData[imu];
	if (row != vImuData.size()) {
		return std::make_pair(false, "RowMismatch");
	}
	return std::make_pair(true, "");
}

const float* const LPMSImuDataList::retrieveFloatData(size_t imu, char const* field, size_t row, size_t col) {
	return retrieveDataPointer<float>(imu, field, row, col, "float");
}

const double* const LPMSImuDataList::retrieveDoubleData(size_t imu, char const* field, size_t row, size_t col){
	return retrieveDataPointer<double>(imu, field, row, col, "double");
}

const uint64_t* const LPMSImuDataList::retrieveUint64Data(size_t imu, char const* field, size_t row, size_t col) {
	return retrieveDataPointer<uint64_t>(imu, field, row, col, "uint64");
}

template<typename T>
const T* const LPMSImuDataList::retrieveDataPointer(size_t imu, char const* field, size_t row, size_t col, char const* type) {
	auto checkResult = checkRetrieveRequest(imu, field, row, col, type);
	if (!checkResult.first) {
		lastError = checkResult.second;
		throw lastError;
	}
	if (vmpDerivedDataPointers[imu].count(field))
		return reinterpret_cast<T*>(vmpDerivedDataPointers[imu][field]);
	const std::vector<AdapterImuData>& vImuData = vData[imu];
	if (vImuData.empty()) {
		vmpDerivedDataPointers[imu][field] = nullptr;
		return nullptr;
	}
	T* result = reinterpret_cast<T*>(new char[row * col * sizeof(T)]);
	Eigen::Map<Eigen::Matrix<T, -1, -1, Eigen::RowMajor>> mat(result, row, col);
	const size_t offset = mpField2Offset[field];
	for (size_t i = 0; i < row; ++i) {
		const char* pData = reinterpret_cast<const char*>(&vImuData[i]);
		const T* pAttr = reinterpret_cast<const T*>(pData + offset);
		for (size_t j = 0; j < col; ++j) {
			mat(i, j) = pAttr[j];
		}
	}
	vmpDerivedDataPointers[imu][field] = reinterpret_cast<char*>(result);
	return result;
}

char const* LPMSImuDataList::getLastError() {
	return lastError.c_str();
}

LPMSAdapter::LPMSAdapter(int imuCnt):nImuCnt(imuCnt), vvImuData(imuCnt), readerThreadStarted(false) {
	
}

void LPMSAdapter::start() {
	ZenSetLogLevel(ZenLogLevel_Debug);
	//see impelementation of make_pair
	auto pair = zen::make_client();
	if (pair.first) {
		lastError = "ClientCreationFailed";
		return;
	}
	pClient = std::make_unique<zen::ZenClient>(std::move(pair.second));
	if (auto error = pClient->listSensorsAsync()) {
		lastError = "ListSensorAsyncFailed";
		throw lastError;
	}
	std::vector<ZenSensorDesc> vSensorDesc;
	bool scanFinished = false;
	std::cout << "started scaning available sensors" << std::endl;
	while (!scanFinished) {
		const auto pair = pClient->waitForNextEvent();
		const bool success = pair.first;
		if (!success) {
			lastError = "ScanWaitEventFailed";
			throw lastError;
		}
		auto& event = pair.second;
		switch (pair.second.eventType) {
		case ZenEventType_SensorFound:
			vSensorDesc.push_back(event.data.sensorFound);
			break;
		case ZenEventType_SensorListingProgress:
			std::cout << "scaning process: " << event.data.sensorListingProgress.progress << std::endl;
			if (event.data.sensorListingProgress.progress >= 1.0f)
				scanFinished = true;
			break;
		default:
			lastError = "UnknownScanEvent";
			throw lastError;
		}
	}
	int nImuFound = 0;
	std::vector<ZenSensorDesc> vImuDesc;
	for (const auto& desc : vSensorDesc) {
		std::string ioType(desc.ioType);
		std::string name(desc.name);
		std::string serialNumber(desc.serialNumber);
		if (ioType == "Bluetooth" && name.rfind("LPMS", 0) == 0 && serialNumber != "") {
			//this is an IMU sensor
			vImuDesc.push_back(desc);
		}
	}
	std::cout << "available imu sensor found: " << vImuDesc.size() << std::endl;
	if (vImuDesc.size() < nImuCnt) {
		lastError = "InsufficientIMU";
		throw lastError;
	}
	int nextImuIdx = 0;
	for (auto& desc : vImuDesc) {
		auto sensorPair = pClient->obtainSensor(desc);
		if (sensorPair.first) {
			continue;
		}
		auto& sensor = sensorPair.second;
		auto imuPair = sensor.getAnyComponentOfType(g_zenSensorType_Imu);
		auto& hasImu = imuPair.first;
		auto imu = imuPair.second;
		if (!hasImu) {
			lastError = "ImuNotAvailableInSensor";
			throw lastError;
		}
		auto isStreamingPair = imu.getBoolProperty(ZenImuProperty_StreamData);
		if (isStreamingPair.first) {
			lastError = "NoStreamingProperty";
			throw lastError;
		}
		if (auto error = imu.setBoolProperty(ZenImuProperty_StreamData, true)) {
			lastError = "StreamSetFailed";
			throw lastError;
		}
		mpHandle2Idx[imu.sensor().handle] = nextImuIdx++;
		vSensors.emplace_back(std::move(sensor));
		vSensorComponents.emplace_back(std::move(imu));
		if (vSensors.size() == nImuCnt)
			break;
	}

	if (vSensors.size() < nImuCnt) {
		lastError = "InsufficientAvailableImu";
		throw lastError;
	}

	readerThreadQuitFlag = false;
	readerThread = std::thread(&LPMSAdapter::readerThreadFunction, this);
	readerThreadStarted = true;
}

void LPMSAdapter::stop() {
	if (pClient) {
		if (readerThreadStarted) {
			readerThreadQuitFlag = true;
			readerThread.join();
		}
		pClient->close();
		pClient.reset(nullptr);	//this would only call it once
	}
}

AdapterImuData LPMSAdapter::getSingleImuData(int idx) {
	if (idx >= nImuCnt) {
		lastError = "InvalidImuIndex";
		throw lastError;
	}
	AdapterImuData res;
	bool resGet = false;
	while (true) {
		{
			std::lock_guard<std::mutex> lock(mtxImuData);
			if (!vvImuData[idx].empty()) {
				res = vvImuData[idx].front();
				vvImuData[idx].pop();
				break;
			}
		}
		Sleep(10);
	}
	return res;
}

LPMSImuDataList LPMSAdapter::getImuBeforeTime(uint64_t stamp) {
	std::vector<std::vector<AdapterImuData>> res(nImuCnt);
	int nImuToCheck = nImuCnt;
	std::vector<bool> vbImuChecked(nImuCnt, false);
	uint64_t startStamp = getMachineTimestamp();
	while (nImuToCheck > 0) {
		bool someImuChecked = false;
		{
			std::lock_guard<std::mutex> lock(mtxImuData);
			for (size_t i = 0; i < nImuCnt; ++i) {
				if (vbImuChecked[i])
					continue;
				auto& q = vvImuData[i];
				if (!q.empty()) {
					someImuChecked = true;
					while (!q.empty()) {
						if (q.front().machineTimestamp < stamp) {
							res[i].push_back(q.front());
							q.pop();
						}
						else {
							vbImuChecked[i] = true;
							nImuToCheck--;
							break;
						}
					}
				}
			}
		}
		if (!someImuChecked) {
			Sleep(10);
		}
		uint64_t currentStamp = getMachineTimestamp();
		if (currentStamp - startStamp > 1000000) {
			lastError = "QueueTimeout";
			throw lastError;
		}
	}
	return LPMSImuDataList(std::move(res));
}

void LPMSAdapter::readerThreadFunction() {
	std::vector<uint64_t> vLag(nImuCnt, UINT64_MAX);
	std::vector<int> vCheckCnt(nImuCnt, 0);
	const int checkCntThreshold = 100;
	while (!readerThreadQuitFlag) {
		auto eventPair = pClient->waitForNextEvent();
		if (!eventPair.first) {
			continue;
		}
		uint64_t machineTimestamp = getMachineTimestamp();
		auto& event = eventPair.second;
		if (event.eventType == ZenEventType_ImuData) {
			const auto& imuData = event.data.imuData;
			int imuIdx = mpHandle2Idx[event.sensor.handle];
			uint64_t imuTimestamp = secondToMicroSecond(imuData.timestamp);
			uint64_t lag = machineTimestamp - imuTimestamp;
			vLag[imuIdx] = min(vLag[imuIdx], lag);
			vCheckCnt[imuIdx]++;
			bool allChecked = true;
			for (auto& cnt : vCheckCnt) {
				if (cnt < checkCntThreshold) {
					allChecked = false;
					break;
				}
			}
			if (allChecked)
				break;
		}
	}

	while (!readerThreadQuitFlag) {
		auto eventPair = pClient->pollNextEvent();
		if (!eventPair.first) {
			continue;
		}
		uint64_t machineTimestamp = getMachineTimestamp();
		auto& event = eventPair.second;
		if (event.eventType == ZenEventType_ImuData) {
			const auto& imuData = event.data.imuData;
			int imuIdx = mpHandle2Idx[event.sensor.handle];
			{
				std::lock_guard<std::mutex> lock(mtxImuData);
				AdapterImuData data;
				data.zenImuData = imuData;
				data.machineTimestamp = secondToMicroSecond(imuData.timestamp) + vLag[imuIdx];
				vvImuData[imuIdx].push(data);
			}
		}
	}
}

char const* LPMSAdapter::getLastError() {
	return lastError.c_str();
}

LPMSAdapter::~LPMSAdapter() {
	stop();
}
