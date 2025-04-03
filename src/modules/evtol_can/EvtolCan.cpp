#include "EvtolCan.hpp"

static MW_H7CAN_DEVICE h7can;

EvtolCan *EvtolCan::_instance;


EvtolCan::EvtolCan(MW_H7CAN_DEVICE& h7can_device) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::uavcan),
	_h7can_device(h7can_device),
	_canesc(_h7can_device),
	_canservo(_h7can_device),
	_dcdc(_node_mutex, _h7can_device)
{
	int res = pthread_mutex_init(&_node_mutex, nullptr);
	_h7can_device.init(0, _param_db_can_rate.get(), 0);
	_h7can_device.init(1, _param_db_can_rate.get(), 0);

	if (res < 0) {
		std::abort();
	}
}

EvtolCan::~EvtolCan()
{
	canRxIdAssigner = 0;
	_h7can_device.close(0);
	_h7can_device.close(1);
	pthread_mutex_destroy(&_node_mutex);

	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

/**
 * @brief 为指定ID分配全局缓冲区
 *
 * 为指定的CAN模块和ID分配全局接收缓冲区，并初始化相关参数。
 *
 * @param CANModule CAN模块号 从0开始
 * @param id CAN消息ID
 * @param idType ID类型，0为标准ID，1为扩展ID
 */
void EvtolCan::AssignGlobalBufferForID(uint8_T CANModule, uint32_T id, uint8_T idType)
{
	globalCANRxBuffer[canRxIdAssigner].CANModule = CANModule;
	globalCANRxBuffer[canRxIdAssigner].ID = id;
	globalCANRxBuffer[canRxIdAssigner].IDType = idType;
	globalCANRxBuffer[canRxIdAssigner].Valid = 0;
	canRxIdAssigner = canRxIdAssigner + 1U;
	if (canRxIdAssigner>MW_NUM_CAN_RECEIVE_RAW)
	{
		canRxIdAssigner = MW_NUM_CAN_RECEIVE_RAW;
	}
}

/**
 * @brief 通过CAN ID接收消息
 *
 * 根据指定的CAN ID从CAN模块接收消息，并将接收到的数据存储在提供的缓冲区中。
 *
 * @param CANModule CAN模块编号，从0开始
 * @param rxData 用于存储接收到的数据的缓冲区指针
 * @param id 要接收的CAN消息的ID
 * @param idType ID类型，0为标准ID，1为扩展ID
 * @param remote 指向远程帧标志的指针
 * @param length 指向消息长度的指针
 *
 * @return 接收成功返回0，失败返回4
 */
uint8_t EvtolCan::ReceiveMessages_By_ID(uint8_T CANModule, uint8_T* rxData, uint32_T id, uint8_T idType, uint8_T *remote, uint8_T *length)
{
	/* idType 0: std, 1: extended*/
	int rxStatus = 0;
	//struct can_msg_s rxmsg;
	int msgIdx,idx;
	uint8_T 	rx_data[8] ={0};
	uint8_T 	rx_idType=0;
	uint32_T 	rx_id=0;
	uint8_T 	rx_length=0;
	uint8_T 	rx_remote=0;
	/* Read all CAN messages available, till CAN read errors*/
	while(rxStatus>=0)
	{
		rxStatus = _h7can_device.receiveMessage(CANModule, &rx_data[0], &rx_id, &rx_idType, &rx_remote, &rx_length);// call read fn here read(canHandleMap[CANModule], &rxmsg, sizeof(rxmsg));
		if (rxStatus ==4)
		{
			rxStatus= -1; /*CAN Receive failure*/
		}else {
			/* Update Global Receive Buffer if CAN Receive is successfull*/
			/* Store in global buffer for Raw Data Type CAN Receive block */
			for(msgIdx=0;msgIdx<MW_NUM_CAN_RECEIVE_RAW;msgIdx++)
			{
				if( (globalCANRxBuffer[msgIdx].Valid==0) && (globalCANRxBuffer[msgIdx].ID == rx_id) && (globalCANRxBuffer[msgIdx].IDType == rx_idType) && (globalCANRxBuffer[msgIdx].CANModule == CANModule))
				{
					globalCANRxBuffer[msgIdx].Length = rx_length;
					globalCANRxBuffer[msgIdx].Remote = rx_remote;
					globalCANRxBuffer[msgIdx].Valid = 1;
					memcpy(&globalCANRxBuffer[msgIdx].Data[0], &rx_data[0], rx_length);
					break;
				}
			}
		}
	}

	/* Read from Buffer */
	for(msgIdx = 0; msgIdx < MW_NUM_CAN_RECEIVE_RAW; msgIdx++)
	{
		if((id == globalCANRxBuffer[msgIdx].ID) && (idType == globalCANRxBuffer[msgIdx].IDType) && (CANModule == globalCANRxBuffer[msgIdx].CANModule) && (globalCANRxBuffer[msgIdx].Valid ==1 ))
		{
			for(idx = 0; idx < 8 ;idx++)
			{
				rxData[idx] = globalCANRxBuffer[msgIdx].Data[idx];
				globalCANRxBuffer[msgIdx].Data[idx] =0;
			}
			globalCANRxBuffer[msgIdx].Valid = 0;
			*length = globalCANRxBuffer[msgIdx].Length;
			*remote = globalCANRxBuffer[msgIdx].Remote;
			rxStatus = 0; /* Read Sucess */
			break;
		}
	}

    	if(rxStatus<0){
		return (uint8_t)4; /*CAN Receive failure*/
	}else{
		return (uint8_t)0; /*CAN Receive Success*/
	}
}
bool EvtolCan::init()
{
	size_t br_msg_id_num = 0;
	size_t msg_id_num_sum = 0;
	int ret = 0;
	ICanSensorBridge::make_all(_can_sensor_bridges);

	for (const auto &br : _can_sensor_bridges)
	{
		ret = br->init();

		//当前br的msg_id添加到globalCANRxBuffer
		br_msg_id_num = br->get_msg_id_num();
		msg_id_num_sum += br_msg_id_num;
		//消息类型总数不超过预定义量则继续添加
		if(msg_id_num_sum <= MW_NUM_CAN_RECEIVE_RAW)
		{
			for(size_t i = 0; i<br_msg_id_num; i++)
			{
				AssignGlobalBufferForID(br->get_can_module(), br->get_msg_id()[i], 1);
			}
		}else{
			mavlink_log_warning(&_mavlink_log_pub, "CAN receive buffer %u is too low", MW_NUM_CAN_RECEIVE_RAW);
		}

		if (ret < 0) {
			PX4_ERR("cannot init sensor bridge '%s' (%d)", br->get_name(), ret);
			return ret;
		}
		PX4_DEBUG("sensor bridge '%s' init ok", br->get_name());
	}


	printf("EvtolCan init success!\n");

	return true;
}

void EvtolCan::print_info()
{
	(void)pthread_mutex_lock(&_node_mutex);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);

	(void)pthread_mutex_unlock(&_node_mutex);
}

void EvtolCan::Run()
{
	size_t br_msg_id_num = 0;
	size_t msg_id_num_sum = 0;
	uint8_t rxData[8] = {0,};
	uint8_t remote;
	uint8_t Length;
	uint32_t _msg_id = 0;

	if(!_node_init){
		_instance->init();
		_node_init = true;
	}

	pthread_mutex_lock(&_node_mutex);
	for (auto &br : _can_sensor_bridges) {

		//检测当前br所需要的消息是否更新
		br_msg_id_num = br->get_msg_id_num();
		msg_id_num_sum += br_msg_id_num;
		// PX4_INFO("msg id num %u", br_msg_id_num);
		if(msg_id_num_sum <= MW_NUM_CAN_RECEIVE_RAW)
		{
			for(size_t i = 0; i<br_msg_id_num; i++)
			{
				_msg_id = br->get_msg_id()[i];
				if(!ReceiveMessages_By_ID(br->get_can_module(), &rxData[0], _msg_id, 1, &remote, &Length))
				{
					br->msg_cb(_msg_id, &rxData[0], Length);
				}
			}
		}
		// else{
		// 	mavlink_log_warning(&_mavlink_log_pub, "CAN receive buffer is too low, excessive messages will not be received");
		// }
	}
	pthread_mutex_unlock(&_node_mutex);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);
}

int EvtolCan::start()
{
	if (_instance != nullptr) {
		PX4_WARN("Already started");
		return -1;
	}

	// if (h7can == nullptr)
	// {
	// 	h7can = new MW_H7CAN_DEVICE();

	// 	if (h7can == nullptr) {  // We don't have exceptions so bad_alloc cannot be thrown
	// 		PX4_ERR("Out of memory");
	// 		return -1;
	// 	}
	// }

	_instance = new EvtolCan(h7can);

	if (_instance == nullptr) {
		PX4_ERR("Out of memory");
		return -1;
	}

	_instance->ScheduleOnInterval(10_ms);

	_instance->_can_interface_esc.ScheduleNow();
	_instance->_can_interface_servo.ScheduleNow();
	_instance->_dcdc.ScheduleOnInterval(1_s);
	return 0;
}

/*
 * App entry point
 */
static void print_usage()
{
	PX4_INFO("usage: \n"
		 "\tevtol_can {start|status|stop}\n");
}

extern "C" __EXPORT int evtol_can_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	int32_t uavcan_enable = 1;
	(void)param_get(param_find("UAVCAN_ENABLE"), &uavcan_enable);

	if (!std::strcmp(argv[1], "start") && uavcan_enable==0) {
		if (EvtolCan::instance()) {
			// Already running, no error
			PX4_INFO("already started");
			::exit(0);
		}

		// // Node ID
		// int32_t node_id = 1;
		// (void)param_get(param_find("UAVCAN_NODE_ID"), &node_id);

		// if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
		// 	PX4_ERR("Invalid Node ID %" PRId32, node_id);
		// 	::exit(1);
		// }

		// // CAN bitrate
		// int32_t bitrate = 1000000;
		// (void)param_get(param_find("UAVCAN_BITRATE"), &bitrate);

		// Start
		// PX4_INFO("Node ID %" PRIu32 ", bitrate %" PRIu32, node_id, bitrate);
		return EvtolCan::start();
	}

	/* commands below require the app to be started */
	EvtolCan *const inst = EvtolCan::instance();

	if (!inst) {
		errx(1, "application not running");
	}

	if (!std::strcmp(argv[1], "status") || !std::strcmp(argv[1], "info")) {
		inst->print_info();
		::exit(0);
	}

	if (!std::strcmp(argv[1], "stop")) {
		delete inst;
		::exit(0);
	}

	print_usage();
	::exit(1);
}
