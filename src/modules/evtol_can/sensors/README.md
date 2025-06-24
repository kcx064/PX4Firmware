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

在运行的时候，src/modules/evtol_can/EvtolCan.cpp会自动调用test.hpp中的方法
```c++
void sensor_template::msg_cb(uint32_t msg_id, uint8_t *rxData, uint8_t len)
{

}
```
用户需要完成上面方法实现对所接收CAN消息的处理，其中msg_id可能的值均在msg_id_list中定义

3.如果can接收缓存不足，需要在CMakeLists.txt中修改-DMW_NUM_CAN_RECEIVE_RAW为更大的数值


