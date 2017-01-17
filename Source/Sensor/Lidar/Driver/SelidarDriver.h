
#ifndef _SELIDAR_DRIVER_H_
#define _SELIDAR_DRIVER_H_

#include <Thread/Condition.h>
#include "SelidarTypes.h"
#include "SelidarCommand.h"
#include "Serial.h"

namespace NS_Selidar {

class SelidarDriver
{
public:

	enum {
		DEFAULT_TIMEOUT = 2000, //2000 ms
	};

	enum {
		DRIVER_TYPE_SERIALPORT = 0x0,
	};

    enum {
        MAX_SCAN_NODES = 2048,
    };

    enum {
        LEGACY_SAMPLE_DURATION = 476,
    };

    SelidarDriver();
    virtual ~SelidarDriver();

public:
    virtual u_result connect(const char * port_path, unsigned int baudrate, unsigned int flag);
    virtual void disconnect();
    virtual bool isConnected();

    virtual u_result reset(unsigned int timeout = DEFAULT_TIMEOUT);

    virtual u_result getHealth(selidar_response_device_health_t &, unsigned int timeout = DEFAULT_TIMEOUT);
    virtual u_result getDeviceInfo(selidar_response_device_info_t &, unsigned int timeout = DEFAULT_TIMEOUT);
    virtual u_result getSampleDuration_uS(selidar_response_sample_rate_t & rateInfo, unsigned int timeout = DEFAULT_TIMEOUT);

    virtual u_result setMotorPWM(unsigned short pwm);
    virtual u_result startMotor();
    virtual u_result stopMotor();
    virtual u_result checkMotorCtrlSupport(bool & support, unsigned int timeout = DEFAULT_TIMEOUT);
	virtual u_result getFrequency(bool inExpressMode, size_t count, float & frequency, bool & is4kmode);

    virtual u_result startScan(bool force = false, bool autoExpressMode = true);
    virtual u_result startScanNormal(bool force, unsigned int timeout = DEFAULT_TIMEOUT);
    virtual u_result startScanExpress(bool fixedAngle, unsigned int timeout = DEFAULT_TIMEOUT);
    virtual u_result checkExpressScanSupported(bool & support, unsigned int timeout = DEFAULT_TIMEOUT);

    virtual u_result stop(unsigned int timeout = DEFAULT_TIMEOUT);
    virtual u_result grabScanData(selidar_response_measurement_node_t * nodebuffer, size_t & count, unsigned int timeout = DEFAULT_TIMEOUT);
    virtual u_result ascendScanData(selidar_response_measurement_node_t * nodebuffer, size_t count);

protected:
    u_result _waitNode(selidar_response_measurement_node_t * node, unsigned int timeout = DEFAULT_TIMEOUT);
    u_result _waitScanData(selidar_response_measurement_node_t * nodebuffer, size_t & count, unsigned int timeout = DEFAULT_TIMEOUT);
	u_result _cacheScanData();
    void     _capsuleToNormal(const selidar_response_capsule_measurement_nodes_t & capsule, selidar_response_measurement_node_t *nodebuffer, size_t &nodeCount);
    u_result _waitCapsuledNode(selidar_response_capsule_measurement_nodes_t & node, unsigned int timeout = DEFAULT_TIMEOUT);
    u_result  _cacheCapsuledScanData();
    u_result _sendCommand(unsigned char cmd, const void * payload = NULL, size_t payloadsize = 0);
    u_result _waitResponseHeader(selidar_ans_header_t * header, unsigned int timeout = DEFAULT_TIMEOUT);
    u_result _waitSampleRate(selidar_response_sample_rate_t * res, unsigned int timeout = DEFAULT_TIMEOUT);

    void     _disableDataGrabbing();

    bool     _isConnected;
    bool     _isScanning;
    bool     _isSupportingMotorCtrl;

	boost::mutex _lock;
    NS_NaviCommon::Condition _dataEvt;
    Serial* _rxtx;
    selidar_response_measurement_node_t      _cached_scan_node_buf[2048];
    size_t                                   _cached_scan_node_count;

    unsigned short                    _cached_sampleduration_std;
    unsigned short                    _cached_sampleduration_express;

    selidar_response_capsule_measurement_nodes_t _cached_previous_capsuledata;
    bool                                         _is_previous_capsuledataRdy;

	boost::thread _cachethread;
};


}

#endif
