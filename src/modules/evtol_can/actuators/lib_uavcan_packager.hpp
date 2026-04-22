#pragma once

#include <cstdint>

class uavcan_packager
{
private:
	#pragma pack(push,1)
	typedef union crc16
	{
		uint16_t crc_val;
		uint8_t crc16_byte[2];
	}crc16_u;
	#pragma pack(pop)
	uint8_t *_buffer;
	uint16_t _len;
	crc16_u _crc;

	uint8_t transferred_data{0};
	uint8_t remain_byte{0};
	uint8_t _need_crc{0};
	/* Tail byte的标记位 */
	bool start_of_transfer{false};
	bool end_of_transfer{false};
	uint8_t toggle{0};
	uint8_t transfer_id{0};//width 5bit
public:
	/**
	 * @brief 向对象 uavcan_packager提供buffer/len/crc，通过方法的get_package函数获取打包后的数据，通过refersh刷新内部状态用于下一轮get_package
	 * @param buffer
	 * @param len
	 * @param crc
	*/
	uavcan_packager(uint8_t* buffer, uint16_t len) :
		_buffer{buffer},
		_len{len}
	{
		if(_len > 7)
		{
			_need_crc = 1;
		}
	}

	uint8_t init_packager()
	{
		start_of_transfer = true;
		transferred_data = 0;
		toggle = 0;
		return 0;
	}

	uint8_t set_crc(uint16_t crc)
	{
		_crc.crc_val = crc;
		return 0;
	}

	uint8_t set_len(uint16_t len)
	{
		_len = len;
		if(_len > 7)
		{
			_need_crc = 1;
		}
		return 0;
	}

	uint8_t get_package(uint8_t *pkg_buffer, uint8_t *pkg_len)
	{
		if(transferred_data == _len)
		{
			return 1;
		}

		if(_need_crc){//需要CRC的时候说明数据包字节数大于7字节，需要多帧, 且计算CRC
			if(start_of_transfer)
			{
				memcpy(pkg_buffer, _crc.crc16_byte, 2); //赋值CRC校验码
				memcpy(&pkg_buffer[2], _buffer, 5);
				transferred_data = 5;

				end_of_transfer = false;
				toggle = false;
				transfer_id += 8;
				pkg_buffer[7] = static_cast<uint8_t>(
					(start_of_transfer << 7) |
					(end_of_transfer << 6) |
					(toggle << 5) |
					(transfer_id >> 3)
				);
				toggle = !toggle;
				*pkg_len = 8;
				start_of_transfer = false;
				return 0;//返回0表示数据未全部打包完毕
			}else{//TODO: 开始处理第二帧第三帧...
				remain_byte = _len - transferred_data;
				if(remain_byte > 7){//说明不是最后一帧
					memcpy(pkg_buffer, &_buffer[transferred_data], 7);
					transferred_data += 7;

					end_of_transfer = false;
					pkg_buffer[7] = static_cast<uint8_t>(
						(start_of_transfer << 7) |
						(end_of_transfer << 6) |
						(toggle << 5) |
						(transfer_id >> 3)
					);
					toggle = !toggle;
					*pkg_len = 8;
					return 0;//返回0表示数据未全部打包完毕
				}else{//说明是最后一帧
					memcpy(pkg_buffer, &_buffer[transferred_data], remain_byte);
					transferred_data += remain_byte;

					end_of_transfer = true;

					pkg_buffer[remain_byte] = static_cast<uint8_t>(
						(start_of_transfer << 7) |
						(end_of_transfer << 6) |
						(toggle << 5) |
						(transfer_id >> 3)
					);
					toggle = !toggle;
					*pkg_len = remain_byte + 1;
					return 0;
				}
			}

		}else{//不需要CRC时候，说明数据包小于等于7字节，那么数据包内容为负载+尾字节
			memcpy(pkg_buffer, _buffer, _len);
			transferred_data = _len;

			transfer_id += 8;
			end_of_transfer = true;
			toggle = false;

			pkg_buffer[_len] = static_cast<uint8_t>(
				(start_of_transfer << 7) |
				(end_of_transfer << 6) |
				(toggle << 5) |
				(transfer_id >> 3)
			);
			*pkg_len = _len + 1;
			start_of_transfer = false;

			return 0;
		}
	}
};



