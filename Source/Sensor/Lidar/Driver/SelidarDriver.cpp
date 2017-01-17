
#include <stdio.h>
#include <iostream>
#include "SelidarDriver.h"
#include "SelidarTypes.h"
#include "SelidarCommand.h"
#include <Time/Utils.h>

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#ifndef _countof
#define _countof(_Array) (sizeof(_Array) / sizeof(_Array[0]))
#endif

using std::cin;
using std::cout;
using std::endl;


namespace NS_Selidar {


// Serial Driver Impl

SelidarDriver::SelidarDriver()
    : _isConnected(false)
    , _isScanning(false)
    , _isSupportingMotorCtrl(false)
{
    _rxtx = new Serial();
    _cached_scan_node_count = 0;
    _cached_sampleduration_std = LEGACY_SAMPLE_DURATION;
    _cached_sampleduration_express = LEGACY_SAMPLE_DURATION;
}

SelidarDriver::~SelidarDriver()
{
    // force disconnection
    disconnect();

    delete _rxtx;
}

u_result SelidarDriver::connect(const char * port_path, unsigned int baudrate, unsigned int flag)
{
    if (isConnected()) return RESULT_ALREADY_DONE;

    if (!_rxtx) return RESULT_INSUFFICIENT_MEMORY;

    {
        boost::mutex::scoped_lock auto_lock(_lock);

        // establish the serial connection...
        if (!_rxtx->bind(port_path, baudrate)  ||  !_rxtx->open()) {
            return RESULT_INVALID_DATA;
        }

        _rxtx->flush(0);
    }

    _isConnected = true;
    //@test
    //checkMotorCtrlSupport(_isSupportingMotorCtrl);
    stopMotor();
    return RESULT_OK;
}

void SelidarDriver::disconnect()
{
    if (!_isConnected) return ;
    stop();
    _rxtx->close();
}

bool SelidarDriver::isConnected()
{
    return _isConnected;
}


u_result SelidarDriver::reset(unsigned int timeout)
{
    u_result ans;

    {
    	boost::mutex::scoped_lock auto_lock(_lock);

        if (IS_FAIL(ans = _sendCommand(SELIDAR_CMD_RESET))) {
            return ans;
        }
    }
    return RESULT_OK;
}

u_result SelidarDriver::getHealth(selidar_response_device_health_t & healthinfo, unsigned int timeout)
{
    u_result  ans;

    if (!isConnected()) return RESULT_OPERATION_FAIL;

    _disableDataGrabbing();

    {
    	boost::mutex::scoped_lock auto_lock(_lock);

        if (IS_FAIL(ans = _sendCommand(SELIDAR_CMD_GET_DEVICE_HEALTH))) {
            printf("getHealth cmd false\n");
            return ans;
        }


        selidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            printf("got timeout in getHealth\n");
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != SELIDAR_ANS_TYPE_DEVHEALTH) {
        	printf("invalid type in getHealth\n");
            return RESULT_INVALID_DATA;
        }

        unsigned int header_size = (response_header.size_q30_subtype & SELIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(selidar_response_device_health_t)) {
        	printf("invalid data in getHealth\n");
            return RESULT_INVALID_DATA;
        }

        if (_rxtx->waitfordata(header_size, timeout) != Serial::ANS_OK) {
        	printf("data timeout in getHealth\n");
            return RESULT_OPERATION_TIMEOUT;
        }
        _rxtx->recvdata(reinterpret_cast<unsigned char *>(&healthinfo), sizeof(healthinfo));
    }

    return RESULT_OK;
}

u_result SelidarDriver::getDeviceInfo(selidar_response_device_info_t & info, unsigned int timeout)
{
    u_result  ans;

    if (!isConnected()) return RESULT_OPERATION_FAIL;

    _disableDataGrabbing();

    {
    	boost::mutex::scoped_lock auto_lock(_lock);

        if (IS_FAIL(ans = _sendCommand(SELIDAR_CMD_GET_DEVICE_INFO))) {
            return ans;
        }

        selidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != SELIDAR_ANS_TYPE_DEVINFO) {
            return RESULT_INVALID_DATA;
        }

        unsigned int header_size = (response_header.size_q30_subtype & SELIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(selidar_response_device_info_t)) {
            return RESULT_INVALID_DATA;
        }

        if (_rxtx->waitfordata(header_size, timeout) != Serial::ANS_OK) {
            return RESULT_OPERATION_TIMEOUT;
        }

        _rxtx->recvdata(reinterpret_cast<unsigned char *>(&info), sizeof(info));
    }
    return RESULT_OK;
}

u_result SelidarDriver::getFrequency(bool inExpressMode, size_t count, float & frequency, bool & is4kmode)
{
    unsigned short sample_duration = inExpressMode?_cached_sampleduration_express:_cached_sampleduration_std;
    frequency = 1000000.0f/(count * sample_duration);

    if (sample_duration <= 277) {
        is4kmode = true;
    } else {
        is4kmode = false;
    }

	return RESULT_OK;
}

u_result SelidarDriver::startScanNormal(bool force, unsigned int timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop 0x81
    // have to slow down the speed of sending cmd, otherwise next cmd will be discard by radar
    NS_NaviCommon::delay(100);

    {
    	boost::mutex::scoped_lock auto_lock(_lock);
        //@test
        printf("start normal scan\n");
        //_sendCommand(0x41);
        if (IS_FAIL(ans = _sendCommand(force?SELIDAR_CMD_FORCE_SCAN:SELIDAR_CMD_SCAN))) {
            return ans;
        }

        // waiting for confirmation
        selidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        printf("verify header\n");
        printf("response_header.type: %X\n", response_header.type);
        // verify whether we got a correct header
        if (response_header.type != SELIDAR_ANS_TYPE_MEASUREMENT) {
            printf("RESULT_INVALID_DATA;\n");
            return RESULT_INVALID_DATA;
        }

        unsigned int header_size = (response_header.size_q30_subtype & SELIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(selidar_response_measurement_node_t)) {
            return RESULT_INVALID_DATA;
        }

        _isScanning = true;
        _cachethread = boost::thread(boost::bind(&SelidarDriver::_cacheScanData, this));
    }
    printf("startScanNormal OK\n");
    return RESULT_OK;
}

u_result SelidarDriver::checkExpressScanSupported(bool & support, unsigned int timeout)
{
    selidar_response_device_info_t devinfo;

    support = false;
    u_result ans = getDeviceInfo(devinfo, timeout);

    if (IS_FAIL(ans)) {cout << std::hex << ans << endl;return ans;}

    if (devinfo.firmware_version >= ((0x1<<8) | 17)) {
        support = true;
        selidar_response_sample_rate_t sample_rate;
        getSampleDuration_uS(sample_rate);
        _cached_sampleduration_express = sample_rate.express_sample_duration_us;
        _cached_sampleduration_std = sample_rate.std_sample_duration_us;
    }

    return RESULT_OK;
}

u_result SelidarDriver::startScanExpress(bool fixedAngle, unsigned int timeout)
{
    u_result ans;
    if (!isConnected()) return RESULT_OPERATION_FAIL;
    if (_isScanning) return RESULT_ALREADY_DONE;

    stop(); //force the previous operation to stop

    {
    	boost::mutex::scoped_lock auto_lock(_lock);

        selidar_payload_express_scan_t scanReq;
        scanReq.working_mode = (fixedAngle?SELIDAR_EXPRESS_SCAN_MODE_FIXANGLE:SELIDAR_EXPRESS_SCAN_MODE_NORMAL);
        scanReq.reserved = 0;

        if (IS_FAIL(ans = _sendCommand(SELIDAR_CMD_EXPRESS_SCAN,&scanReq, sizeof(scanReq)))) {
            return ans;
        }

        // waiting for confirmation
        selidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != SELIDAR_ANS_TYPE_MEASUREMENT_CAPSULED) {
            return RESULT_INVALID_DATA;
        }

        unsigned int header_size = (response_header.size_q30_subtype & SELIDAR_ANS_HEADER_SIZE_MASK);
        if (header_size < sizeof(selidar_response_capsule_measurement_nodes_t)) {
            return RESULT_INVALID_DATA;
        }

        _isScanning = true;
        _cachethread = boost::thread(boost::bind(&SelidarDriver::_cacheCapsuledScanData, this));
    }
    return RESULT_OK;
}


u_result SelidarDriver::startScan(bool force, bool autoExpressMode)
{
    bool isExpressModeSupported;
    u_result ans;

    if (autoExpressMode) {
        ans = checkExpressScanSupported(isExpressModeSupported);

        if (IS_FAIL(ans)) return ans;

        if (isExpressModeSupported) {
            return startScanExpress(false);
        }
    }
    NS_NaviCommon::delay(100);  //force to delay otherwise it is too fast to receive response message
    return startScanNormal(force);
}

u_result SelidarDriver::stop(unsigned int timeout)
{
    u_result ans;
    _disableDataGrabbing();

    {
    	boost::mutex::scoped_lock auto_lock(_lock);

        if (IS_FAIL(ans = _sendCommand(SELIDAR_CMD_STOP))) {
            return ans;
        }
    }

    return RESULT_OK;
}

u_result SelidarDriver::_cacheScanData()
{
    selidar_response_measurement_node_t      local_buf[128];
    size_t                                   count = 128;
    selidar_response_measurement_node_t      local_scan[MAX_SCAN_NODES];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));

    _waitScanData(local_buf, count); // // always discard the first data since it may be incomplete

    while(_isScanning)
    {
        if (IS_FAIL(ans=_waitScanData(local_buf, count))) {
            if (ans != RESULT_OPERATION_TIMEOUT) {
                _isScanning = false;
                return RESULT_OPERATION_FAIL;
            }
        }

        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].sync_quality & SELIDAR_RESP_MEASUREMENT_SYNCBIT)
            {

                // only publish the data when it contains a full 360 degree scan

                if ((local_scan[0].sync_quality & SELIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                    _lock.lock();
                    memcpy(_cached_scan_node_buf, local_scan, scan_count*sizeof(selidar_response_measurement_node_t));
                    _cached_scan_node_count = scan_count;
                    _dataEvt.set();
                    _lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) scan_count-=1; // prevent overflow
        }
    }
    _isScanning = false;
    return RESULT_OK;
}

void     SelidarDriver::_capsuleToNormal(const selidar_response_capsule_measurement_nodes_t & capsule, selidar_response_measurement_node_t *nodebuffer, size_t &nodeCount)
{
    nodeCount = 0;
    if (_is_previous_capsuledataRdy) {
        int diffAngle_q8;
        int currentStartAngle_q8 = ((capsule.start_angle_sync_q6 & 0x7FFF)<< 2);
        int prevStartAngle_q8 = ((_cached_previous_capsuledata.start_angle_sync_q6 & 0x7FFF) << 2);

        diffAngle_q8 = (currentStartAngle_q8) - (prevStartAngle_q8);
        if (prevStartAngle_q8 >  currentStartAngle_q8) {
            diffAngle_q8 += (360<<8);
        }

        int angleInc_q16 = (diffAngle_q8 << 3);
        int currentAngle_raw_q16 = (prevStartAngle_q8 << 8);
        for (size_t pos = 0; pos < _countof(_cached_previous_capsuledata.cabins); ++pos)
        {
            int dist_q2[2];
            int angle_q6[2];
            int syncBit[2];

            dist_q2[0] = (_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0xFFFC);
            dist_q2[1] = (_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0xFFFC);

            int angle_offset1_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 & 0xF) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_1 & 0x3)<<4));
            int angle_offset2_q3 = ( (_cached_previous_capsuledata.cabins[pos].offset_angles_q3 >> 4) | ((_cached_previous_capsuledata.cabins[pos].distance_angle_2 & 0x3)<<4));

            angle_q6[0] = ((currentAngle_raw_q16 - (angle_offset1_q3<<13))>>10);
            syncBit[0] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
            currentAngle_raw_q16 += angleInc_q16;


            angle_q6[1] = ((currentAngle_raw_q16 - (angle_offset2_q3<<13))>>10);
            syncBit[1] =  (( (currentAngle_raw_q16 + angleInc_q16) % (360<<16)) < angleInc_q16 )?1:0;
            currentAngle_raw_q16 += angleInc_q16;

            for (int cpos = 0; cpos < 2; ++cpos) {

                if (angle_q6[cpos] < 0) angle_q6[cpos] += (360<<6);
                if (angle_q6[cpos] >= (360<<6)) angle_q6[cpos] -= (360<<6);

                selidar_response_measurement_node_t node;

                node.sync_quality = (syncBit[cpos] | ((!syncBit[cpos]) << 1));
                if (dist_q2[cpos]) node.sync_quality |= (0x2F << SELIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);

                node.angle_q6_checkbit = (1 | (angle_q6[cpos]<<1));
                //printf("%d\n", dist_q2[cpos]);
                //if (dist_q2[cpos] > 10000 || dist_q2[cpos] < 0) dist_q2[cpos] = 0.0;
                node.distance_q2 = dist_q2[cpos];

                nodebuffer[nodeCount++] = node;
             }

        }
    }

    _cached_previous_capsuledata = capsule;
    _is_previous_capsuledataRdy = true;
}


u_result SelidarDriver::_cacheCapsuledScanData()
{
    selidar_response_capsule_measurement_nodes_t    capsule_node;
    selidar_response_measurement_node_t      local_buf[128];
    size_t                                   count = 128;
    selidar_response_measurement_node_t      local_scan[MAX_SCAN_NODES];
    size_t                                   scan_count = 0;
    u_result                                 ans;
    memset(local_scan, 0, sizeof(local_scan));

    _waitCapsuledNode(capsule_node); // // always discard the first data since it may be incomplete

    while(_isScanning)
    {
        if (IS_FAIL(ans=_waitCapsuledNode(capsule_node))) {
            if (ans != RESULT_OPERATION_TIMEOUT && ans != RESULT_INVALID_DATA) {
                _isScanning = false;
                return RESULT_OPERATION_FAIL;
            } else {
                // current data is invalid, do not use it.
                continue;
            }
        }

        _capsuleToNormal(capsule_node, local_buf, count);

        for (size_t pos = 0; pos < count; ++pos)
        {
            if (local_buf[pos].sync_quality & SELIDAR_RESP_MEASUREMENT_SYNCBIT)
            {
                // only publish the data when it contains a full 360 degree scan

                if ((local_scan[0].sync_quality & SELIDAR_RESP_MEASUREMENT_SYNCBIT)) {
                    _lock.lock();
                    memcpy(_cached_scan_node_buf, local_scan, scan_count*sizeof(selidar_response_measurement_node_t));
                    _cached_scan_node_count = scan_count;
                    _dataEvt.set();
                    _lock.unlock();
                }
                scan_count = 0;
            }
            local_scan[scan_count++] = local_buf[pos];
            if (scan_count == _countof(local_scan)) scan_count-=1; // prevent overflow
        }
    }
    _isScanning = false;

    return RESULT_OK;
}

u_result SelidarDriver::grabScanData(selidar_response_measurement_node_t * nodebuffer, size_t & count, unsigned int timeout)
{
    switch (_dataEvt.wait(timeout/1000))
    {
    case NS_NaviCommon::Condition::COND_TIMEOUT:
        count = 0;
        printf("RESULT_OPERATION_TIMEOUT in grabScanData\n");
        return RESULT_OPERATION_TIMEOUT;
    case NS_NaviCommon::Condition::COND_OK:
        {
            if(_cached_scan_node_count == 0) return RESULT_OPERATION_TIMEOUT; //consider as timeout

            boost::mutex::scoped_lock auto_lock(_lock);

            size_t size_to_copy = min(count, _cached_scan_node_count);

            memcpy(nodebuffer, _cached_scan_node_buf, size_to_copy*sizeof(selidar_response_measurement_node_t));
            count = size_to_copy;
            _cached_scan_node_count = 0;
        }
        return RESULT_OK;

    default:
        count = 0;
        return RESULT_OPERATION_FAIL;
    }
}

u_result SelidarDriver::ascendScanData(selidar_response_measurement_node_t * nodebuffer, size_t count)
{
    float inc_origin_angle = 360.0/count;
    size_t i = 0;

    //Tune head
    for (i = 0; i < count; i++) {
        if(nodebuffer[i].distance_q2 == 0) {
            continue;
        } else {
            while(i != 0) {
                i--;
                float expect_angle = (nodebuffer[i+1].angle_q6_checkbit >> SELIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f - inc_origin_angle;
                if (expect_angle < 0.0f) expect_angle = 0.0f;
                unsigned short checkbit = nodebuffer[i].angle_q6_checkbit & SELIDAR_RESP_MEASUREMENT_CHECKBIT;
                nodebuffer[i].angle_q6_checkbit = (((unsigned short)(expect_angle * 64.0f)) << SELIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
            }
            break;
        }
    }

    // all the data is invalid
    if (i == count) return RESULT_OPERATION_FAIL;

    //Tune tail
    for (i = count - 1; i >= 0; i--) {
        if(nodebuffer[i].distance_q2 == 0) {
            continue;
        } else {
            while(i != (count - 1)) {
                i++;
                float expect_angle = (nodebuffer[i-1].angle_q6_checkbit >> SELIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f + inc_origin_angle;
                if (expect_angle > 360.0f) expect_angle -= 360.0f;
                unsigned short checkbit = nodebuffer[i].angle_q6_checkbit & SELIDAR_RESP_MEASUREMENT_CHECKBIT;
                nodebuffer[i].angle_q6_checkbit = (((unsigned short)(expect_angle * 64.0f)) << SELIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
            }
            break;
        }
    }

    //Fill invalid angle in the scan
    float frontAngle = (nodebuffer[0].angle_q6_checkbit >> SELIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    for (i = 1; i < count; i++) {
        if(nodebuffer[i].distance_q2 == 0) {
            float expect_angle =  frontAngle + i * inc_origin_angle;
            if (expect_angle > 360.0f) expect_angle -= 360.0f;
            unsigned short checkbit = nodebuffer[i].angle_q6_checkbit & SELIDAR_RESP_MEASUREMENT_CHECKBIT;
            nodebuffer[i].angle_q6_checkbit = (((unsigned short)(expect_angle * 64.0f)) << SELIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
        }
    }

    // Reorder the scan according to the angle value
    for (i = 0; i < (count-1); i++){
        for (size_t j = (i+1); j < count; j++){
            if(nodebuffer[i].angle_q6_checkbit > nodebuffer[j].angle_q6_checkbit){
                selidar_response_measurement_node_t temp = nodebuffer[i];
                nodebuffer[i] = nodebuffer[j];
                nodebuffer[j] = temp;
            }
        }
    }

    return RESULT_OK;
}

u_result SelidarDriver::_waitNode(selidar_response_measurement_node_t * node, unsigned int timeout)
{
    int  recvPos = 0;
    unsigned int startTs = NS_NaviCommon::getMs();
    unsigned char  recvBuffer[sizeof(selidar_response_measurement_node_t)];
    unsigned char *nodeBuffer = (unsigned char*)node;
    unsigned int waitTime;

   while ((waitTime = NS_NaviCommon::getMs() - startTs) <= timeout) {
        size_t remainSize = sizeof(selidar_response_measurement_node_t) - recvPos;
        size_t recvSize;

        int ans = _rxtx->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if (ans == Serial::ANS_DEV_ERR)
            return RESULT_OPERATION_FAIL;
        else if (ans == Serial::ANS_TIMEOUT)
            return RESULT_OPERATION_TIMEOUT;

        if (recvSize > remainSize) recvSize = remainSize;

        _rxtx->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
            unsigned char currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync bit and its reverse in this byte
                {
                    unsigned char tmp = (currentByte>>1);
                    if ( (tmp ^ currentByte) & 0x1 ) {
                        // pass
                    } else {
                        continue;
                    }

                }
                break;
            case 1: // expect the highest bit to be 1
                {
                    if (currentByte & SELIDAR_RESP_MEASUREMENT_CHECKBIT) {
                        // pass
                    } else {
                        recvPos = 0;
                        continue;
                    }
                }
                break;
            }
            nodeBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(selidar_response_measurement_node_t)) {
                return RESULT_OK;
            }
        }
    }

    return RESULT_OPERATION_TIMEOUT;
}


u_result SelidarDriver::_waitScanData(selidar_response_measurement_node_t * nodebuffer, size_t & count, unsigned int timeout)
{
    if (!_isConnected) {
        count = 0;
        return RESULT_OPERATION_FAIL;
    }

    size_t   recvNodeCount =  0;
    unsigned int     startTs = NS_NaviCommon::getMs();
    unsigned int     waitTime;
    u_result ans;

    while ((waitTime = NS_NaviCommon::getMs() - startTs) <= timeout && recvNodeCount < count) {
        selidar_response_measurement_node_t node;
        if (IS_FAIL(ans = _waitNode(&node, timeout - waitTime))) {
            return ans;
        }

        nodebuffer[recvNodeCount++] = node;

        if (recvNodeCount == count) return RESULT_OK;
    }
    count = recvNodeCount;
    return RESULT_OPERATION_TIMEOUT;
}


u_result SelidarDriver::_waitCapsuledNode(selidar_response_capsule_measurement_nodes_t & node, unsigned int timeout)
{
    int  recvPos = 0;
    unsigned int startTs = NS_NaviCommon::getMs();
    unsigned char  recvBuffer[sizeof(selidar_response_capsule_measurement_nodes_t)];
    unsigned char *nodeBuffer = (unsigned char*)&node;
    unsigned int waitTime;

   while ((waitTime = NS_NaviCommon::getMs() - startTs) <= timeout) {
        size_t remainSize = sizeof(selidar_response_capsule_measurement_nodes_t) - recvPos;
        size_t recvSize;

        int ans = _rxtx->waitfordata(remainSize, timeout-waitTime, &recvSize);
        if (ans == Serial::ANS_DEV_ERR)
            return RESULT_OPERATION_FAIL;
        else if (ans == Serial::ANS_TIMEOUT)
            return RESULT_OPERATION_TIMEOUT;

        if (recvSize > remainSize) recvSize = remainSize;

        _rxtx->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
            unsigned char currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0: // expect the sync bit 1
                {
                    unsigned char tmp = (currentByte>>4);
                    if ( tmp == SELIDAR_RESP_MEASUREMENT_EXP_SYNC_1 ) {
                        // pass
                    } else {
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }

                }
                break;
            case 1: // expect the sync bit 2
                {
                    unsigned char tmp = (currentByte>>4);
                    if (tmp == SELIDAR_RESP_MEASUREMENT_EXP_SYNC_2) {
                        // pass
                    } else {
                        recvPos = 0;
                        _is_previous_capsuledataRdy = false;
                        continue;
                    }
                }
                break;
            }
            nodeBuffer[recvPos++] = currentByte;

            if (recvPos == sizeof(selidar_response_capsule_measurement_nodes_t)) {
                // calc the checksum ...
                unsigned char checksum = 0;
                unsigned char recvChecksum = ((node.s_checksum_1 & 0xF) | (node.s_checksum_2<<4));
                for (size_t cpos = offsetof(selidar_response_capsule_measurement_nodes_t, start_angle_sync_q6);
                    cpos < sizeof(selidar_response_capsule_measurement_nodes_t); ++cpos)
                {
                    checksum ^= nodeBuffer[cpos];
                }
                if (recvChecksum == checksum)
                {
                    // only consider vaild if the checksum matches...
                    if (node.start_angle_sync_q6 & SELIDAR_RESP_MEASUREMENT_EXP_SYNCBIT)
                    {
                        // this is the first capsule frame in logic, discard the previous cached data...
                        _is_previous_capsuledataRdy = false;
                        return RESULT_OK;
                    }
                    return RESULT_OK;
                }
                _is_previous_capsuledataRdy = false;
                return RESULT_INVALID_DATA;
            }
        }
    }
    _is_previous_capsuledataRdy = false;
    return RESULT_OPERATION_TIMEOUT;
}


u_result SelidarDriver::_sendCommand(unsigned char cmd, const void * payload, size_t payloadsize)
{
    unsigned char pkt_header[10];
    unsigned char pkg[128] = {0};
    int pkg_size = 0;
    selidar_cmd_packet_t * header = reinterpret_cast<selidar_cmd_packet_t * >(pkt_header);
    unsigned char checksum = 0;

    if (!_isConnected) return RESULT_OPERATION_FAIL;

    if (payloadsize && payload) {
        cmd |= SELIDAR_CMDFLAG_HAS_PAYLOAD;
    }

    header->syncByte = SELIDAR_CMD_SYNC_BYTE;
    header->cmd_flag = cmd;

    // send header first
    //_rxtx->senddata(pkt_header, 2) ;
    memcpy(pkg + pkg_size, pkt_header, 2);
    pkg_size += 2;
    if (cmd & SELIDAR_CMDFLAG_HAS_PAYLOAD) {
        checksum ^= SELIDAR_CMD_SYNC_BYTE;
        checksum ^= cmd;
        checksum ^= (payloadsize & 0xFF);

        // calc checksum
        for (size_t pos = 0; pos < payloadsize; ++pos) {
            checksum ^= ((unsigned char *)payload)[pos];
        }

        // send size
        unsigned char sizebyte = payloadsize;
        //_rxtx->senddata(&sizebyte, 1);
        memcpy(pkg + pkg_size, &sizebyte, 1);
        pkg_size += 1;

        // send payload
       if (sizebyte > 0)
       {
    	   //_rxtx->senddata((const unsigned char *)payload, sizebyte);
    	   memcpy(pkg + pkg_size, payload, sizebyte);
    	   pkg_size += sizebyte;
       }

        // send checksum
        //_rxtx->senddata(&checksum, 1);
       memcpy(pkg + pkg_size, &checksum, 1);
       pkg_size += 1;
    }
    _rxtx->senddata(pkg, pkg_size);

    return RESULT_OK;
}


u_result SelidarDriver::_waitResponseHeader(selidar_ans_header_t * header, unsigned int timeout)
{
    int  recvPos = 0;
    unsigned int startTs = NS_NaviCommon::getMs();
    unsigned char  recvBuffer[sizeof(selidar_ans_header_t)];
    unsigned char  *headerBuffer = reinterpret_cast<unsigned char *>(header);
    unsigned int waitTime;

    while ((waitTime = NS_NaviCommon::getMs() - startTs) <= timeout) {
        size_t remainSize = sizeof(selidar_ans_header_t) - recvPos;
        size_t recvSize;
        int ans = _rxtx->waitfordata(remainSize, timeout - waitTime, &recvSize);
        if (ans == Serial::ANS_DEV_ERR) {
            printf("ANS_DEV_ERR\n");
            return RESULT_OPERATION_FAIL;
        }
        else if (ans == Serial::ANS_TIMEOUT) {
            printf("ANS_TIMEOUT\n");
            return RESULT_OPERATION_TIMEOUT;
        }

        if(recvSize > remainSize) recvSize = remainSize;

        _rxtx->recvdata(recvBuffer, recvSize);

        for (size_t pos = 0; pos < recvSize; ++pos) {
            unsigned char currentByte = recvBuffer[pos];
            switch (recvPos) {
            case 0:
                if (currentByte != SELIDAR_ANS_SYNC_BYTE1) {
                   continue;
                }

                break;
            case 1:
                if (currentByte != SELIDAR_ANS_SYNC_BYTE2) {
                    recvPos = 0;
                    continue;
                }
                break;
            }
            headerBuffer[recvPos++] = currentByte;
            if (recvPos == sizeof(selidar_ans_header_t)) {
                return RESULT_OK;
            }
        }
    }
    return RESULT_OPERATION_TIMEOUT;
}



void SelidarDriver::_disableDataGrabbing()
{
    _isScanning = false;
    _cachethread.join();
}

u_result SelidarDriver::getSampleDuration_uS(selidar_response_sample_rate_t & rateInfo, unsigned int timeout)
{
    if (!isConnected()) return RESULT_OPERATION_FAIL;

    _disableDataGrabbing();

    selidar_response_device_info_t devinfo;
    // 1. fetch the device version first...
    u_result ans = getDeviceInfo(devinfo, timeout);

    rateInfo.express_sample_duration_us = _cached_sampleduration_express;
    rateInfo.std_sample_duration_us = _cached_sampleduration_std;

    if (devinfo.firmware_version < ((0x1<<8) | 17)) {
        // provide fake data...

        return RESULT_OK;
    }


    {
    	boost::mutex::scoped_lock auto_lock(_lock);

        if (IS_FAIL(ans = _sendCommand(SELIDAR_CMD_GET_SAMPLERATE))) {
            return ans;
        }

        selidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != SELIDAR_ANS_TYPE_SAMPLE_RATE) {
            return RESULT_INVALID_DATA;
        }

        unsigned int header_size = (response_header.size_q30_subtype & SELIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(selidar_response_sample_rate_t)) {
            return RESULT_INVALID_DATA;
        }

        if (_rxtx->waitfordata(header_size, timeout) != Serial::ANS_OK) {
            return RESULT_OPERATION_TIMEOUT;
        }
        _rxtx->recvdata(reinterpret_cast<unsigned char *>(&rateInfo), sizeof(rateInfo));
    }
    return RESULT_OK;
}

u_result SelidarDriver::checkMotorCtrlSupport(bool & support, unsigned int timeout)
{
    u_result  ans;
    support = false;

    if (!isConnected()) return RESULT_OPERATION_FAIL;

    _disableDataGrabbing();

    {
    	boost::mutex::scoped_lock auto_lock(_lock);

        selidar_payload_acc_board_flag_t flag;
        flag.reserved = 0;

        if (IS_FAIL(ans = _sendCommand(SELIDAR_CMD_GET_ACC_BOARD_FLAG, &flag, sizeof(flag)))) {
            return ans;
        }
        selidar_ans_header_t response_header;
        if (IS_FAIL(ans = _waitResponseHeader(&response_header, timeout))) {
            return ans;
        }

        // verify whether we got a correct header
        if (response_header.type != SELIDAR_ANS_TYPE_ACC_BOARD_FLAG) {
            return RESULT_INVALID_DATA;
        }

        unsigned int header_size = (response_header.size_q30_subtype & SELIDAR_ANS_HEADER_SIZE_MASK);
        if ( header_size < sizeof(selidar_response_acc_board_flag_t)) {
            return RESULT_INVALID_DATA;
        }

        if (_rxtx->waitfordata(header_size, timeout) != Serial::ANS_OK) {
            return RESULT_OPERATION_TIMEOUT;
        }
        selidar_response_acc_board_flag_t acc_board_flag;
        _rxtx->recvdata(reinterpret_cast<unsigned char *>(&acc_board_flag), sizeof(acc_board_flag));

        if (acc_board_flag.support_flag & SELIDAR_RESP_ACC_BOARD_FLAG_MOTOR_CTRL_SUPPORT_MASK) {
            support = true;
        }
    }

    return RESULT_OK;
}

u_result SelidarDriver::setMotorPWM(unsigned short pwm)
{
    u_result ans;
    selidar_payload_motor_pwm_t motor_pwm;
    motor_pwm.pwm_value = pwm;

    {
    	boost::mutex::scoped_lock auto_lock(_lock);

        if (IS_FAIL(ans = _sendCommand(SELIDAR_CMD_SET_MOTOR_PWM,(const unsigned char *)&motor_pwm, sizeof(motor_pwm)))) {
            return ans;
        }
    }

    return RESULT_OK;
}

u_result SelidarDriver::startMotor()
{
    if (_isSupportingMotorCtrl) { // SELIDAR A2
        setMotorPWM(DEFAULT_MOTOR_PWM);
        NS_NaviCommon::delay(500);
        return RESULT_OK;
    } else { // SELIDAR A1
    	boost::mutex::scoped_lock auto_lock(_lock);
        _rxtx->clearDTR();
        NS_NaviCommon::delay(500);
        return RESULT_OK;
    }
}

u_result SelidarDriver::stopMotor()
{
    if (_isSupportingMotorCtrl) { // SELIDAR A2
        setMotorPWM(0);
        NS_NaviCommon::delay(500);
        return RESULT_OK;
    } else { // SELIDAR A1
    	boost::mutex::scoped_lock auto_lock(_lock);
        _rxtx->setDTR();
        NS_NaviCommon::delay(500);
        return RESULT_OK;
    }
}

}
