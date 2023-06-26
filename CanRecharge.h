#include "linux_interop.h"
#include "PCANBasic.h"
#include <set>
#include <queue>
#include <string>
#include <sstream>
#include <array>
#include <cstdio>
#include <errno.h>
#include <stdio.h>
#include <mutex>
#include <condition_variable>

#include <fstream>
#include <sys/time.h>
#include <chrono>


const char *const FILE_VERSION = "03.00";
const char *const CAN_NAME = "CAN";
const char *const CAN_FD_NAME = "CANFD";

const int32_t CANBUS_MESSAGE_LENGTH = 8;	 // according to ISO-11891-1
const int32_t CANBUS_FD_MESSAGE_LENGTH = 64; // according to ISO-11891-1

#pragma pack(push)
#pragma pack(1)
typedef struct DataInfo
{
	DataInfo(const char *can_name)
	{
		memset(name, 0, 8);
		if (can_name)
			memcpy(name, can_name, strlen(can_name));

		memset(file_version, 0, 8);
		memcpy(file_version, FILE_VERSION, strlen(FILE_VERSION));
	}

	char name[8];		  // "CAN"
	char file_version[8]; // "03.00"
} DataInfo;

// 6-byte socket destination
typedef struct SocketDest
{
	uint32_t recv_addr = 0; // the ip address of receiver, "239.0.0.1"
	uint16_t recv_port = 0; // the port of receiver, 11013
} SocketDest;

// 128 Byte Header
struct CANFileHeader
{
	CANFileHeader(const char *can_name = NULL)
		: info(can_name) {}

	DataInfo info;		  // name & version for the file data
	SocketDest dest;	  // socket destination property
	uint8_t padding[106]; // further use, customized by each app
};

//  40 Byte Header
struct CANTransactionHeader
{
	uint64_t recv_timestamp_ptp; // little-endian the PTP timestamp (ns) when CAN Rx receives the UDP packet
	uint64_t send_timestamp_ptp; // little-endian the timestamp (ns) when MCU sends the UDP packet
	uint64_t recv_timestamp_utc; // little-endian the UTC timestamp (ns) when CAN Rx receives the UDP packet
	uint64_t send_timestamp_utc; // same as recv_timestamp_utc
	uint64_t size;				 // little-endian, payload size in byte
	char payload[0];			 // the actual length of the payload is defined by size
};

// the max packet's size is 1460 Bytes for every packet
struct CANPacketHeader
{
	uint64_t time_stamp; // MCU to SoC: Time in ns when MCU sends the UDP packet to SoC.
	// SoC to MCU: Time in ns when SoC sends the UDP packet to MCU.
	uint32_t counter;	 // Incremented by each packet, from 0 to 2^32-1
	uint8_t packet_type; // 0: CAN Data, 1: CAN FD Data
	uint8_t msg_number;	 // Number of messages contained in this packet (N)
	uint8_t senderID;	 // 1: S1, 3:S3, 5:MCU1, 6:MCU2
	uint8_t reserved;	 // 0x00, reserved for alignment
	char payload[0];	 // the actual can/canfd msg
};

enum class CAN_CAHNNEL : uint8_t
{
	ADAS_CAN = 0x0,
	CHASSIS_1_CAN = 0x1,
	CHASSIS_2_CAN = 0x2,
	FLM_CAN = 0x3,
	RADAR_MR_CAN = 0x4,
	RADAR_FC_CAN = 0x5,
	RADAR_RC_CAN = 0x6
};

// the can msg size is 28 Byte
struct CANFlushMsg
{
	uint64_t time_stamp; // MCU to SoC: Time in ns when MCU receives the CAN message.
	// SoC to MCU: Time in ns when SoC receives the ProtoBuf message.

	uint32_t can_msg_id; // can message ID
	CAN_CAHNNEL channel;
	uint8_t rolling_counter = 0x0; // Incremented by each message, continuous for packets
	uint8_t reserved = 0x0;		   // reserved for alignment
	uint8_t length;				   // Length of payload in bytes
	char payload[CANBUS_MESSAGE_LENGTH];
	int16_t e2e;			   // From Timestamp to Payload, - CRC16_CCITT_FALSE
	uint16_t padding = 0xFFFF; // 0xFFFF, to make the message size aligned to 32-bit
};

// the canfd msg size is 84 Byte
struct CANFDFlushMsg
{
	uint64_t time_stamp; // MCU to SoC: Time in ns when MCU receives the CAN message.
	// SoC to MCU: Time in ns when SoC receives the ProtoBuf message.

	uint32_t can_msg_id; // can message ID
	CAN_CAHNNEL channel;
	uint8_t rolling_counter = 0x0; // Incremented by each message, continuous for packets
	uint8_t reserved = 0x0;		   // reserved for alignment
	uint8_t length;				   // Length of payload in bytes
	char payload[CANBUS_FD_MESSAGE_LENGTH];
	int16_t e2e;			   // From Timestamp to Payload, - CRC16_CCITT_FALSE
	uint16_t padding = 0xFFFF; // 0xFFFF, to make the message size aligned to 32-bit
};
#pragma pack(pop)

class CanRecharge
{
private:
	// const TPCANHandle PcanHandle = PCAN_USBBUS1;
	TPCANHandle PcanHandle;
	const bool IsFD = true;
	const TPCANBaudrate Bitrate = PCAN_BAUD_500K;
	TPCANBitrateFD BitrateFD = const_cast<LPSTR>("f_clock=80000000,nom_brp=2,nom_tseg1=63,nom_tseg2=16,nom_sjw=16,data_brp=2,data_tseg1=15,data_tseg2=4,data_sjw=4");
	// const int TimerInterval = 250;
	int TimerInterval;
	bool m_DLLFound;
	std::thread *m_hTimer;
	std::thread *th_run;
	bool m_TimerOn;
	std::mutex mu;
	std::condition_variable m_cv;
	bool m_isPause;

	FILE *fp;
	std::set<uint32_t> id_set_;
	std::queue<uint64_t> que_time;
	std::queue<uint32_t> que_id;
	std::queue<uint8_t> que_len;
	std::queue<std::array<char, CANBUS_FD_MESSAGE_LENGTH>> que_message;
	uint64_t TIME = 0;
	uint64_t TS = 0;
	std::string File_path;

	char* out_path;
	int flag_out;
	int Random;

	uint64_t start_time;
	uint64_t can_start;
	uint64_t now_time;
	uint64_t can_time;

public:
// ts 时间辍 发送从ts开始的消息
// file_path 要发送的文件位置
	CanRecharge(TPCANHandle can_handle,char* file_path,uint64_t ts,char* outPath,int random);
	void stop();
	void pause();
	void resume();
	void RunThread();
	~CanRecharge();
	int status=1;
	void Init(TPCANHandle can,TPCANBitrateFD BitrateFD);
	void LoadMessage();
	bool CheckStop();
	

private:
	void RechargeThread();
	void RunProcess();
	
	void WriteMessages();
	TPCANStatus WriteMessageFD();

	bool CheckForLibrary();
	void ShowStatus(TPCANStatus status);
	void FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD);
	void GetTPCANHandleName(TPCANHandle handle, LPSTR buffer);
	void GetFormattedError(TPCANStatus error, LPSTR buffer);

	uint64_t GetTime();

};
