# 新增CAN传感器
1.要新增传感器请基于模板文件 sensor_template.hpp修改
(包含类名，假设新增传感器为sensor_template、NAME常量、要检测的CAN消息ID)

2.并在文件CanSensorBridge.cpp中的make_all方法中添加
```c++
void ICanSensorBridge::make_all(List<ICanSensorBridge *> &list)
{
	list.add(new esc_status());
	list.add(new sensor_template());
}
```

3.在CMakeLists.txt文件中添加新增的文件，如上例则添加 sensor_template.hpp

在运行的时候，src/modules/evtol_can/EvtolCan.cpp会自动调用sensor_template.hpp中的方法
```c++
void sensor_template::msg_cb(uint32_t msg_id, uint8_t *rxData, uint8_t len)
{

}
```
用户需要完成上面方法实现对所接收CAN消息的处理，其中msg_id可能的值均需在msg_id_list中定义

3.如果can接收缓存不足，需要在CMakeLists.txt中修改-DMW_NUM_CAN_RECEIVE_RAW为更大的数值

## UAVCAN传感器

对于基于uavcan协议的传感器，解析复杂一些。因为uavcan消息定义增加了自定义位宽数据，和连续帧的概念。因此处理起来相比can要复杂。目前在代码里新增了类，专门用于处理这类消息。

用户应使用
类lib_uavcan_parser(src/modules/evtol_can/sensors/lib_uavcan_parser.hpp)来完成。

类`lib_uavcan_parser`基于类`lib_uavcan_buffer`(src/modules/evtol_can/sensors/lib_uavcan_buffer.hpp)和`lib_uavcan_field_extractor`(src/modules/evtol_can/sensors/lib_uavcan_field_extractor.hpp)实现。

具体应用代码参考src/modules/evtol_can/sensors/redundancy_detector.hpp中的实现。
里面实现对raw_command消息的解析，这个消息是飞控发给电调的油门数据，每个油门使用uint14来表示，按照uavcan方式紧凑排列。

首先在类声明中声明必要的存储数组和结构体数组，同时初始化对象 `_uavcan_parser`
```c++
uint8_t buffer[20] = {0,};
uint8_t buff_len_max{20};
uavcan_field_info_s _field_info[4] = {
	{14,0},
	{14,0},
	{14,0},
	{14,0}
};
uint8_t _field_info_size{4};
lib_uavcan_parser _uavcan_parser{buffer, buff_len_max, _field_info, _field_info_size};
```

在回调函数中，每次收到对应类型帧就运行对象`_uavcan_parser`的run方法，run方法会在完整处理完毕一组连续帧uavcan消息后返回1，此时就可以从`_field_info`读取数据，做后续处理
```c++
if(msg_id == msg_id_list[0])
{
	if(_uavcan_parser.run(rxData, len))
	{
		//从_field_info中提取解析后的数据
	}
}
```


