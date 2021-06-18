%% About defineLPMSAdapterLib.mlx
% This file defines the MATLAB interface to the library |LPMSAdapterLib|.
%
% Commented sections represent C++ functionality that MATLAB cannot automatically define. To include
% functionality, uncomment a section and provide values for &lt;SHAPE&gt;, &lt;DIRECTION&gt;, etc. For more
% information, see <matlab:helpview(fullfile(docroot,'matlab','helptargets.map'),'cpp_define_interface') Define MATLAB Interface for C++ Library>.



%% Setup. Do not edit this section.
function libDef = defineLPMSAdapterLib()
libDef = clibgen.LibraryDefinition("LPMSAdapterLibData.xml");
%% OutputFolder and Libraries 
libDef.OutputFolder = convertCharsToStrings(pwd());
libDef.Libraries = [ convertCharsToStrings([pwd() '\LPMSAdapter\x64\Release\LPMSAdapter.lib']) convertCharsToStrings([pwd() '\LPMSAdapter\openzen\lib\x64\OpenZen.lib'])];

%% C++ class |ZenImuData| with MATLAB name |clib.LPMSAdapterLib.ZenEventData_Imu| 
ZenEventData_ImuDefinition = addClass(libDef, "ZenImuData", "MATLABName", "clib.LPMSAdapterLib.ZenEventData_Imu", ...
    "Description", "clib.LPMSAdapterLib.ZenEventData_Imu    Representation of C++ class ZenImuData."); % Modify help description values as needed.

%% C++ class constructor for C++ class |ZenImuData| 
% C++ Signature: ZenImuData::ZenImuData()
ZenEventData_ImuConstructor1Definition = addConstructor(ZenEventData_ImuDefinition, ...
    "ZenImuData::ZenImuData()", ...
    "Description", "clib.LPMSAdapterLib.ZenEventData_Imu.ZenImuData    Constructor of C++ class ZenImuData."); % Modify help description values as needed.
validate(ZenEventData_ImuConstructor1Definition);

%% C++ class constructor for C++ class |ZenImuData| 
% C++ Signature: ZenImuData::ZenImuData(ZenImuData const & input1)
ZenEventData_ImuConstructor2Definition = addConstructor(ZenEventData_ImuDefinition, ...
    "ZenImuData::ZenImuData(ZenImuData const & input1)", ...
    "Description", "clib.LPMSAdapterLib.ZenEventData_Imu.ZenImuData    Constructor of C++ class ZenImuData."); % Modify help description values as needed.
defineArgument(ZenEventData_ImuConstructor2Definition, "input1", "clib.LPMSAdapterLib.ZenEventData_Imu", "input");
validate(ZenEventData_ImuConstructor2Definition);

%% C++ class public data member |frameCount| for C++ class |ZenImuData| 
% C++ Signature: int ZenImuData::frameCount
addProperty(ZenEventData_ImuDefinition, "frameCount", "int32", ...
    "Description", "int32    Data member of C++ class ZenImuData." + newline + ...
    "Index of the data frame." + newline + ...
    " Unit: no unit"); % Modify help description values as needed.

%% C++ class public data member |timestamp| for C++ class |ZenImuData| 
% C++ Signature: double ZenImuData::timestamp
addProperty(ZenEventData_ImuDefinition, "timestamp", "double", ...
    "Description", "double    Data member of C++ class ZenImuData." + newline + ...
    "Sampling time of the data in seconds" + newline + ...
    " Unit: s"); % Modify help description values as needed.

%% C++ class public data member |a| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::a
addProperty(ZenEventData_ImuDefinition, "a", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Calibrated accelerometer sensor data." + newline + ...
    " Unit: m/s^2"); % Modify help description values as needed.

%% C++ class public data member |g| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::g
addProperty(ZenEventData_ImuDefinition, "g", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Calibrated gyroscope sensor data." + newline + ...
    " Unit: degree/s"); % Modify help description values as needed.

%% C++ class public data member |b| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::b
addProperty(ZenEventData_ImuDefinition, "b", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Calibrated magnetometer sensor data." + newline + ...
    " Unit: micro Tesla"); % Modify help description values as needed.

%% C++ class public data member |aRaw| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::aRaw
addProperty(ZenEventData_ImuDefinition, "aRaw", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Raw accelerometer sensor data." + newline + ...
    " Unit: m/s^2"); % Modify help description values as needed.

%% C++ class public data member |gRaw| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::gRaw
addProperty(ZenEventData_ImuDefinition, "gRaw", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Raw gyroscope sensor data." + newline + ...
    " Unit: degree/s"); % Modify help description values as needed.

%% C++ class public data member |bRaw| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::bRaw
addProperty(ZenEventData_ImuDefinition, "bRaw", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Raw magnetometer sensor data." + newline + ...
    " Unit: micro Tesla"); % Modify help description values as needed.

%% C++ class public data member |w| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::w
addProperty(ZenEventData_ImuDefinition, "w", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Angular velocity data." + newline + ...
    " This angular velocity takes into account if an orientation offset" + newline + ...
    " has been set while the g and gRaw values in this struct" + newline + ...
    " do not."); % Modify help description values as needed.

%% C++ class public data member |r| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::r
addProperty(ZenEventData_ImuDefinition, "r", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Euler angle data." + newline + ...
    " Unit: degree/s"); % Modify help description values as needed.

%% C++ class public data member |q| for C++ class |ZenImuData| 
% C++ Signature: float [4] ZenImuData::q
addProperty(ZenEventData_ImuDefinition, "q", "clib.array.LPMSAdapterLib.Float", 4, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Quaternion orientation data." + newline + ...
    " The component order is w, x, y, z" + newline + ...
    " Unit: no unit"); % Modify help description values as needed.

%% C++ class public data member |rotationM| for C++ class |ZenImuData| 
% C++ Signature: float [9] ZenImuData::rotationM
addProperty(ZenEventData_ImuDefinition, "rotationM", "clib.array.LPMSAdapterLib.Float", 9, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Orientation data as rotation matrix without offset." + newline + ...
    " Unit: no unit"); % Modify help description values as needed.

%% C++ class public data member |rotOffsetM| for C++ class |ZenImuData| 
% C++ Signature: float [9] ZenImuData::rotOffsetM
addProperty(ZenEventData_ImuDefinition, "rotOffsetM", "clib.array.LPMSAdapterLib.Float", 9, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Orientation data as rotation matrix after zeroing." + newline + ...
    " Unit: no unit"); % Modify help description values as needed.

%% C++ class public data member |pressure| for C++ class |ZenImuData| 
% C++ Signature: float ZenImuData::pressure
addProperty(ZenEventData_ImuDefinition, "pressure", "single", ...
    "Description", "single    Data member of C++ class ZenImuData." + newline + ...
    "Barometric pressure." + newline + ...
    " Unit: mPascal"); % Modify help description values as needed.

%% C++ class public data member |linAcc| for C++ class |ZenImuData| 
% C++ Signature: float [3] ZenImuData::linAcc
addProperty(ZenEventData_ImuDefinition, "linAcc", "clib.array.LPMSAdapterLib.Float", 3, ... % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Float, or single
    "Description", "clib.array.LPMSAdapterLib.Float    Data member of C++ class ZenImuData." + newline + ...
    "Linear acceleration x, y and z." + newline + ...
    " Unit: m/s^2"); % Modify help description values as needed.

%% C++ class public data member |gTemp| for C++ class |ZenImuData| 
% C++ Signature: float ZenImuData::gTemp
addProperty(ZenEventData_ImuDefinition, "gTemp", "single", ...
    "Description", "single    Data member of C++ class ZenImuData." + newline + ...
    "Gyroscope temperature." + newline + ...
    " Unit: Degree Celcius"); % Modify help description values as needed.

%% C++ class public data member |altitude| for C++ class |ZenImuData| 
% C++ Signature: float ZenImuData::altitude
addProperty(ZenEventData_ImuDefinition, "altitude", "single", ...
    "Description", "single    Data member of C++ class ZenImuData." + newline + ...
    "Altitude." + newline + ...
    " Unit: m"); % Modify help description values as needed.

%% C++ class public data member |temperature| for C++ class |ZenImuData| 
% C++ Signature: float ZenImuData::temperature
addProperty(ZenEventData_ImuDefinition, "temperature", "single", ...
    "Description", "single    Data member of C++ class ZenImuData." + newline + ...
    "Temperature." + newline + ...
    " Unit: Degree Celcius"); % Modify help description values as needed.

%% C++ class public data member |heaveMotion| for C++ class |ZenImuData| 
% C++ Signature: float ZenImuData::heaveMotion
addProperty(ZenEventData_ImuDefinition, "heaveMotion", "single", ...
    "Description", "single    Data member of C++ class ZenImuData." + newline + ...
    "heave motion" + newline + ...
    " Unit: m"); % Modify help description values as needed.

%% C++ class |AdapterImuData| with MATLAB name |clib.LPMSAdapterLib.AdapterImuData| 
AdapterImuDataDefinition = addClass(libDef, "AdapterImuData", "MATLABName", "clib.LPMSAdapterLib.AdapterImuData", ...
    "Description", "clib.LPMSAdapterLib.AdapterImuData    Representation of C++ class AdapterImuData."); % Modify help description values as needed.

%% C++ class constructor for C++ class |AdapterImuData| 
% C++ Signature: AdapterImuData::AdapterImuData()
AdapterImuDataConstructor1Definition = addConstructor(AdapterImuDataDefinition, ...
    "AdapterImuData::AdapterImuData()", ...
    "Description", "clib.LPMSAdapterLib.AdapterImuData.AdapterImuData    Constructor of C++ class AdapterImuData."); % Modify help description values as needed.
validate(AdapterImuDataConstructor1Definition);

%% C++ class constructor for C++ class |AdapterImuData| 
% C++ Signature: AdapterImuData::AdapterImuData(AdapterImuData const & input1)
AdapterImuDataConstructor2Definition = addConstructor(AdapterImuDataDefinition, ...
    "AdapterImuData::AdapterImuData(AdapterImuData const & input1)", ...
    "Description", "clib.LPMSAdapterLib.AdapterImuData.AdapterImuData    Constructor of C++ class AdapterImuData."); % Modify help description values as needed.
defineArgument(AdapterImuDataConstructor2Definition, "input1", "clib.LPMSAdapterLib.AdapterImuData", "input");
validate(AdapterImuDataConstructor2Definition);

%% C++ class public data member |zenImuData| for C++ class |AdapterImuData| 
% C++ Signature: ZenImuData AdapterImuData::zenImuData
addProperty(AdapterImuDataDefinition, "zenImuData", "clib.LPMSAdapterLib.ZenEventData_Imu", ...
    "Description", "clib.LPMSAdapterLib.ZenEventData_Imu    Data member of C++ class AdapterImuData."); % Modify help description values as needed.

%% C++ class public data member |machineTimestamp| for C++ class |AdapterImuData| 
% C++ Signature: uint64_t AdapterImuData::machineTimestamp
addProperty(AdapterImuDataDefinition, "machineTimestamp", "uint64", ...
    "Description", "uint64    Data member of C++ class AdapterImuData."); % Modify help description values as needed.

%% C++ class |LPMSImuDataList| with MATLAB name |clib.LPMSAdapterLib.LPMSImuDataList| 
LPMSImuDataListDefinition = addClass(libDef, "LPMSImuDataList", "MATLABName", "clib.LPMSAdapterLib.LPMSImuDataList", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList    Representation of C++ class LPMSImuDataList."); % Modify help description values as needed.

%% C++ class method |retrieveFloatData| for C++ class |LPMSImuDataList| 
% C++ Signature: float const * const LPMSImuDataList::retrieveFloatData(size_t imu,char const * field,size_t row,size_t col)
retrieveFloatDataDefinition = addMethod(LPMSImuDataListDefinition, ...
    "float const * const LPMSImuDataList::retrieveFloatData(size_t imu,char const * field,size_t row,size_t col)", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList.retrieveFloatData    Method of C++ class LPMSImuDataList."); % Modify help description values as needed.
defineArgument(retrieveFloatDataDefinition, "imu", "uint64");
defineArgument(retrieveFloatDataDefinition, "field", "string", "input", "nullTerminated"); % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Char,int8,string, or char
defineArgument(retrieveFloatDataDefinition, "row", "uint64");
defineArgument(retrieveFloatDataDefinition, "col", "uint64");
defineOutput(retrieveFloatDataDefinition, "RetVal", "single", ["row", "col"]); % '<MLTYPE>' can be single, or clib.array.LPMSAdapterLib.Float
validate(retrieveFloatDataDefinition);

%% C++ class method |retrieveDoubleData| for C++ class |LPMSImuDataList| 
% C++ Signature: double const * const LPMSImuDataList::retrieveDoubleData(size_t imu,char const * field,size_t row,size_t col)
retrieveDoubleDataDefinition = addMethod(LPMSImuDataListDefinition, ...
    "double const * const LPMSImuDataList::retrieveDoubleData(size_t imu,char const * field,size_t row,size_t col)", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList.retrieveDoubleData    Method of C++ class LPMSImuDataList."); % Modify help description values as needed.
defineArgument(retrieveDoubleDataDefinition, "imu", "uint64");
defineArgument(retrieveDoubleDataDefinition, "field", "string", "input", "nullTerminated"); % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Char,int8,string, or char
defineArgument(retrieveDoubleDataDefinition, "row", "uint64");
defineArgument(retrieveDoubleDataDefinition, "col", "uint64");
defineOutput(retrieveDoubleDataDefinition, "RetVal", "double", ["row", "col"]); % '<MLTYPE>' can be double, or clib.array.LPMSAdapterLib.Double
validate(retrieveDoubleDataDefinition);

%% C++ class method |retrieveUint64Data| for C++ class |LPMSImuDataList| 
% C++ Signature: uint64_t const * const LPMSImuDataList::retrieveUint64Data(size_t imu,char const * field,size_t row,size_t col)
retrieveUint64DataDefinition = addMethod(LPMSImuDataListDefinition, ...
    "uint64_t const * const LPMSImuDataList::retrieveUint64Data(size_t imu,char const * field,size_t row,size_t col)", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList.retrieveUint64Data    Method of C++ class LPMSImuDataList."); % Modify help description values as needed.
defineArgument(retrieveUint64DataDefinition, "imu", "uint64");
defineArgument(retrieveUint64DataDefinition, "field", "string", "input", "nullTerminated"); % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Char,int8,string, or char
defineArgument(retrieveUint64DataDefinition, "row", "uint64");
defineArgument(retrieveUint64DataDefinition, "col", "uint64");
defineOutput(retrieveUint64DataDefinition, "RetVal", "uint64", ["row", "col"]); % '<MLTYPE>' can be uint64, or clib.array.LPMSAdapterLib.UnsignedLongLong
validate(retrieveUint64DataDefinition);

%% C++ class method |getRecordCnt| for C++ class |LPMSImuDataList| 
% C++ Signature: size_t LPMSImuDataList::getRecordCnt(size_t imu)
getRecordCntDefinition = addMethod(LPMSImuDataListDefinition, ...
    "size_t LPMSImuDataList::getRecordCnt(size_t imu)", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList.getRecordCnt    Method of C++ class LPMSImuDataList."); % Modify help description values as needed.
defineArgument(getRecordCntDefinition, "imu", "uint64");
defineOutput(getRecordCntDefinition, "RetVal", "uint64");
validate(getRecordCntDefinition);

%% C++ class method |getFieldLength| for C++ class |LPMSImuDataList| 
% C++ Signature: size_t LPMSImuDataList::getFieldLength(char const * field)
getFieldLengthDefinition = addMethod(LPMSImuDataListDefinition, ...
    "size_t LPMSImuDataList::getFieldLength(char const * field)", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList.getFieldLength    Method of C++ class LPMSImuDataList."); % Modify help description values as needed.
defineArgument(getFieldLengthDefinition, "field", "string", "input", "nullTerminated"); % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Char,int8,string, or char
defineOutput(getFieldLengthDefinition, "RetVal", "uint64");
validate(getFieldLengthDefinition);

%% C++ class method |getFieldType| for C++ class |LPMSImuDataList| 
% C++ Signature: char const * LPMSImuDataList::getFieldType(char const * field)
getFieldTypeDefinition = addMethod(LPMSImuDataListDefinition, ...
    "char const * LPMSImuDataList::getFieldType(char const * field)", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList.getFieldType    Method of C++ class LPMSImuDataList."); % Modify help description values as needed.
defineArgument(getFieldTypeDefinition, "field", "string", "input", "nullTerminated"); % '<MLTYPE>' can be clib.array.LPMSAdapterLib.Char,int8,string, or char
defineOutput(getFieldTypeDefinition, "RetVal", "string", "nullTerminated"); % '<MLTYPE>' can be int8,clib.array.LPMSAdapterLib.Char,string, or char
validate(getFieldTypeDefinition);

%% C++ class method |getLastError| for C++ class |LPMSImuDataList| 
% C++ Signature: char const * LPMSImuDataList::getLastError()
getLastErrorDefinition = addMethod(LPMSImuDataListDefinition, ...
    "char const * LPMSImuDataList::getLastError()", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList.getLastError    Method of C++ class LPMSImuDataList."); % Modify help description values as needed.
defineOutput(getLastErrorDefinition, "RetVal", "string", "nullTerminated"); % '<MLTYPE>' can be int8,clib.array.LPMSAdapterLib.Char,string, or char
validate(getLastErrorDefinition);

%% C++ class constructor for C++ class |LPMSImuDataList| 
% C++ Signature: LPMSImuDataList::LPMSImuDataList(LPMSImuDataList const & input1)
LPMSImuDataListConstructor1Definition = addConstructor(LPMSImuDataListDefinition, ...
    "LPMSImuDataList::LPMSImuDataList(LPMSImuDataList const & input1)", ...
    "Description", "clib.LPMSAdapterLib.LPMSImuDataList.LPMSImuDataList    Constructor of C++ class LPMSImuDataList."); % Modify help description values as needed.
defineArgument(LPMSImuDataListConstructor1Definition, "input1", "clib.LPMSAdapterLib.LPMSImuDataList", "input");
validate(LPMSImuDataListConstructor1Definition);

%% C++ class |LPMSAdapter| with MATLAB name |clib.LPMSAdapterLib.LPMSAdapter| 
LPMSAdapterDefinition = addClass(libDef, "LPMSAdapter", "MATLABName", "clib.LPMSAdapterLib.LPMSAdapter", ...
    "Description", "clib.LPMSAdapterLib.LPMSAdapter    Representation of C++ class LPMSAdapter."); % Modify help description values as needed.

%% C++ class constructor for C++ class |LPMSAdapter| 
% C++ Signature: LPMSAdapter::LPMSAdapter(int imuCnt)
LPMSAdapterConstructor1Definition = addConstructor(LPMSAdapterDefinition, ...
    "LPMSAdapter::LPMSAdapter(int imuCnt)", ...
    "Description", "clib.LPMSAdapterLib.LPMSAdapter.LPMSAdapter    Constructor of C++ class LPMSAdapter."); % Modify help description values as needed.
defineArgument(LPMSAdapterConstructor1Definition, "imuCnt", "int32");
validate(LPMSAdapterConstructor1Definition);

%% C++ class method |start| for C++ class |LPMSAdapter| 
% C++ Signature: void LPMSAdapter::start()
startDefinition = addMethod(LPMSAdapterDefinition, ...
    "void LPMSAdapter::start()", ...
    "Description", "clib.LPMSAdapterLib.LPMSAdapter.start    Method of C++ class LPMSAdapter."); % Modify help description values as needed.
validate(startDefinition);

%% C++ class method |getLastError| for C++ class |LPMSAdapter| 
% C++ Signature: char const * LPMSAdapter::getLastError()
getLastErrorDefinition = addMethod(LPMSAdapterDefinition, ...
    "char const * LPMSAdapter::getLastError()", ...
    "Description", "clib.LPMSAdapterLib.LPMSAdapter.getLastError    Method of C++ class LPMSAdapter."); % Modify help description values as needed.
defineOutput(getLastErrorDefinition, "RetVal", "string", "nullTerminated"); % '<MLTYPE>' can be int8,clib.array.LPMSAdapterLib.Char,string, or char
validate(getLastErrorDefinition);

%% C++ class method |getImuBeforeTime| for C++ class |LPMSAdapter| 
% C++ Signature: LPMSImuDataList LPMSAdapter::getImuBeforeTime(uint64_t stamp)
getImuBeforeTimeDefinition = addMethod(LPMSAdapterDefinition, ...
    "LPMSImuDataList LPMSAdapter::getImuBeforeTime(uint64_t stamp)", ...
    "Description", "clib.LPMSAdapterLib.LPMSAdapter.getImuBeforeTime    Method of C++ class LPMSAdapter."); % Modify help description values as needed.
defineArgument(getImuBeforeTimeDefinition, "stamp", "uint64");
defineOutput(getImuBeforeTimeDefinition, "RetVal", "clib.LPMSAdapterLib.LPMSImuDataList");
validate(getImuBeforeTimeDefinition);

%% C++ class method |stop| for C++ class |LPMSAdapter| 
% C++ Signature: void LPMSAdapter::stop()
stopDefinition = addMethod(LPMSAdapterDefinition, ...
    "void LPMSAdapter::stop()", ...
    "Description", "clib.LPMSAdapterLib.LPMSAdapter.stop    Method of C++ class LPMSAdapter."); % Modify help description values as needed.
validate(stopDefinition);

%% C++ class method |getSingleImuData| for C++ class |LPMSAdapter| 
% C++ Signature: AdapterImuData LPMSAdapter::getSingleImuData(int idx)
getSingleImuDataDefinition = addMethod(LPMSAdapterDefinition, ...
    "AdapterImuData LPMSAdapter::getSingleImuData(int idx)", ...
    "Description", "clib.LPMSAdapterLib.LPMSAdapter.getSingleImuData    Method of C++ class LPMSAdapter."); % Modify help description values as needed.
defineArgument(getSingleImuDataDefinition, "idx", "int32");
defineOutput(getSingleImuDataDefinition, "RetVal", "clib.LPMSAdapterLib.AdapterImuData");
validate(getSingleImuDataDefinition);

%% C++ function |getMachineTimestamp| with MATLAB name |clib.LPMSAdapterLib.getMachineTimestamp|
% C++ Signature: uint64_t getMachineTimestamp()
getMachineTimestampDefinition = addFunction(libDef, ...
    "uint64_t getMachineTimestamp()", ...
    "MATLABName", "clib.LPMSAdapterLib.getMachineTimestamp", ...
    "Description", "clib.LPMSAdapterLib.getMachineTimestamp    Representation of C++ function getMachineTimestamp."); % Modify help description values as needed.
defineOutput(getMachineTimestampDefinition, "RetVal", "uint64");
validate(getMachineTimestampDefinition);

%% Validate the library definition
validate(libDef);

end
