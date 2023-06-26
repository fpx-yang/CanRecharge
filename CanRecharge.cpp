#include "CanRecharge.h"

#ifdef __unix
#define fopen_s(pFile, filename, mode) ((*(pFile)) = fopen((filename), (mode))) == NULL
#endif

void CanRecharge::stop(){
	m_TimerOn = false;
}

void CanRecharge::pause(){
	std::unique_lock<std::mutex> lock(mu);
    m_isPause = true;
    m_cv.notify_one();
}

void CanRecharge::resume()
{
    std::unique_lock<std::mutex> lock(mu);
    m_isPause = false;
    m_cv.notify_one();
}


CanRecharge::CanRecharge(TPCANHandle can_handle,char* file_path,uint64_t ts,char* outPath,int random)
{
	PcanHandle = can_handle;
	out_path = outPath;
	Random = random;

	TS = ts;
	File_path = file_path;
	m_TimerOn = true;
	m_isPause = false;

	char* path = (char*) File_path.data();
	// char* file_path = "./canfd.dat";
	error_t err = fopen_s(&fp, path, "rb");
	if (err)
	{
		std::cout << "open can file failed" << std::endl;
	}
	// read file header
	CANFileHeader file_header;
	char buf[128] = {0};
	auto rtn = fread(&file_header, 1, sizeof(CANFileHeader), fp);
	if (rtn != sizeof(file_header))
	{
		std::cout << "(ERROR)"
				  << " read file header failed " << rtn << "  " << sizeof(file_header) << std::endl;
		return;
	}
	// Checks if PCANBasic.dll is available, if not, the program terminates
	m_DLLFound = CheckForLibrary();
	if (!m_DLLFound)
		return;

	TPCANStatus stsResult;
	// Initialization of the selected channel
	if (IsFD)
	{
		std::cout << "is FD\n";
	}


	
}

void CanRecharge::Init(TPCANHandle can,TPCANBitrateFD BitrateFD){
	TPCANStatus stsResult;
	stsResult = CAN_InitializeFD(can,BitrateFD);
	if (stsResult != PCAN_ERROR_OK)
	{
		std::cout<< can << ":" << "Can not initialize. Please check the defines in the code.\n";
	}else{
		// Reading messages...
		std::cout<< "Successfully initialized.\n";
	}
}



void CanRecharge::RunThread(){
	m_TimerOn = true;
	m_hTimer = new std::thread(&CanRecharge::RunProcess, this);
	m_hTimer->detach();
	// RunProcess();
}



void CanRecharge::RunProcess(){
	RechargeThread();
	std::cout << "Started Can Recharge...\n";
}


CanRecharge::~CanRecharge()
{
	m_TimerOn = false;
	// delete m_hTimer;
	delete th_run;
	if (m_DLLFound)
		CAN_Uninitialize(PCAN_NONEBUS);
}

void CanRecharge::LoadMessage()
{
	while (!feof(fp))
	{
		CANTransactionHeader transaction_header;
		auto rtn = fread(&transaction_header, 1, sizeof(transaction_header), fp);
		if (rtn != sizeof(transaction_header))
		{
			std::cout << "(ERROR)"
						<< " read transaction_header failed " << std::endl;
			for (auto it : id_set_)
			{
				std::cout << "(ERROR)" << it << " " << std::hex << it << std::endl;
			}
			stop();
			break;
		}
		// read packet header
		auto &tran_size = transaction_header.size; // MAX1460-16(packet header)
		char *buf = new char[tran_size];
		memset(buf, 0, tran_size);
		rtn = fread(buf, 1, tran_size, fp);
		if (rtn != tran_size)
		{
			std::cout << "(ERROR)"
						<< "read pack and msg failed " << tran_size << "  " << rtn;
			delete[] buf;
			for (auto it : id_set_)
			{
				std::cout << "(ERROR)" << it << " " << std::hex << it << std::endl;
			}
			stop();
			break;
		}
		CANPacketHeader packet_header;
		memcpy(&packet_header, buf, sizeof(packet_header));
		// parse message
		int index = 0;
		char *msg_cursor = buf + sizeof(packet_header);
		for (; index < packet_header.msg_number; index++)
		{
			// CAN_FD
			if (packet_header.packet_type == 1)
			{
				CANFDFlushMsg can_msg;
				memcpy(&can_msg, msg_cursor, sizeof(CANFDFlushMsg));
				msg_cursor += sizeof(CANFDFlushMsg);
				
				if (can_msg.time_stamp<TS){
					continue;
				}
				id_set_.insert(can_msg.can_msg_id);
				que_time.push(can_msg.time_stamp);
				que_len.push((uint32_t)can_msg.length);
				que_id.push(can_msg.can_msg_id);
				que_message.push(std::array<char, CANBUS_FD_MESSAGE_LENGTH>{});
				std::copy(std::begin(can_msg.payload), std::end(can_msg.payload), std::begin(que_message.back()));
			}
		}
		delete[] buf;
	}
	TIME = que_time.front();
	can_start = que_time.front();
	can_time = can_start;
}

bool CanRecharge::CheckStop(){
	return m_TimerOn;
}


uint64_t CanRecharge::GetTime()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	long time = tv.tv_sec * 0xF4240 + tv.tv_usec;
	return time;
}


void CanRecharge::RechargeThread()
{
	while (m_TimerOn)
	{
		std::unique_lock<std::mutex> lock(mu);
        m_cv.wait(lock, [this] {return !m_isPause; });

		start_time = GetTime();

		while (!que_id.empty())
		{
			WriteMessages();
		}
		if(que_id.empty()){
			m_TimerOn=false;
		}
	}

}

void CanRecharge::WriteMessages()
{
	TPCANStatus stsResult;
	stsResult = WriteMessageFD();
	if (stsResult != PCAN_ERROR_OK){
		ShowStatus(stsResult);
	}
		

}


TPCANStatus CanRecharge::WriteMessageFD()
{
	// Sends a CAN-FD message with standard ID, 64 data bytes, and bitrate switch
	TPCANMsgFD msgCanMessageFD;
	std::stringstream ss;
	int tmp_id = que_id.front()+Random;
	// int tmp_id = que_id.front();
	ss << std::hex << tmp_id << std::endl;
	ss >> msgCanMessageFD.ID;
	que_id.pop();
	uint64_t msg_time = que_time.front();
	que_time.pop();
	msgCanMessageFD.DLC = que_len.front();
	que_len.pop();
	msgCanMessageFD.MSGTYPE = PCAN_MESSAGE_FD | PCAN_MESSAGE_BRS;
	for (BYTE i = 0; i < msgCanMessageFD.DLC; i++)
	{
		msgCanMessageFD.DATA[i] = que_message.front()[i];
	}
	que_message.pop();

	TimerInterval = (msg_time - TIME) % 0x3B9ACA00 / 0x3E8;
	TIME = msg_time;
	if (TimerInterval > 0x3E8)
	{
		usleep(TimerInterval-0x3E8);
	}
	long Dif_time = (msg_time-can_time)/0x3E8;
	// 1s
	if(Dif_time>=0xF4240){
		now_time = GetTime();
		std::cout<<"diff_time:"<<(msg_time-can_start)/0x3E8<<"   "<<now_time-start_time<<std::endl;
		if(now_time-start_time<(msg_time-can_start)/0x3E8){
			usleep((msg_time-can_start)/0x3E8-now_time+start_time);
		}
		can_time = msg_time;
	}
	
	return CAN_WriteFD(PcanHandle, &msgCanMessageFD);
}



bool CanRecharge::CheckForLibrary()
{
	// Check for dll file
	try
	{
		CAN_Uninitialize(PCAN_NONEBUS);
		return true;
	}
	catch (const std::exception &)
	{
		std::cout << ("Unable to find the library: PCANBasic::dll !\n");
		std::cout << ("Closing...\n");
		std::cout << "Press any key to continue...\n";
		_getch();
	}

	return false;
}

void CanRecharge::ShowStatus(TPCANStatus status)
{
	std::cout << "=========================================================================================\n";
	char buffer[MAX_PATH];
	GetFormattedError(status, buffer);
	std::cout << buffer << "\n";
	std::cout << "=========================================================================================\n";
}

void CanRecharge::FormatChannelName(TPCANHandle handle, LPSTR buffer, bool isFD)
{
	TPCANDevice devDevice;
	BYTE byChannel;

	// Gets the owner device and channel for a PCAN-Basic handle
	if (handle < 0x100)
	{
		devDevice = (TPCANDevice)(handle >> 4);
		byChannel = (BYTE)(handle & 0xF);
	}
	else
	{
		devDevice = (TPCANDevice)(handle >> 8);
		byChannel = (BYTE)(handle & 0xFF);
	}

	// Constructs the PCAN-Basic Channel name and return it
	char handleBuffer[MAX_PATH];
	GetTPCANHandleName(handle, handleBuffer);
	if (isFD)
		sprintf_s(buffer, MAX_PATH, "%s:FD %d (%Xh)", handleBuffer, byChannel, handle);
	else
		sprintf_s(buffer, MAX_PATH, "%s %d (%Xh)", handleBuffer, byChannel, handle);
}

void CanRecharge::GetTPCANHandleName(TPCANHandle handle, LPSTR buffer)
{
	strcpy_s(buffer, MAX_PATH, "PCAN_NONE");
	switch (handle)
	{
	case PCAN_PCIBUS1:
	case PCAN_PCIBUS2:
	case PCAN_PCIBUS3:
	case PCAN_PCIBUS4:
	case PCAN_PCIBUS5:
	case PCAN_PCIBUS6:
	case PCAN_PCIBUS7:
	case PCAN_PCIBUS8:
	case PCAN_PCIBUS9:
	case PCAN_PCIBUS10:
	case PCAN_PCIBUS11:
	case PCAN_PCIBUS12:
	case PCAN_PCIBUS13:
	case PCAN_PCIBUS14:
	case PCAN_PCIBUS15:
	case PCAN_PCIBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_PCI");
		break;

	case PCAN_USBBUS1:
	case PCAN_USBBUS2:
	case PCAN_USBBUS3:
	case PCAN_USBBUS4:
	case PCAN_USBBUS5:
	case PCAN_USBBUS6:
	case PCAN_USBBUS7:
	case PCAN_USBBUS8:
	case PCAN_USBBUS9:
	case PCAN_USBBUS10:
	case PCAN_USBBUS11:
	case PCAN_USBBUS12:
	case PCAN_USBBUS13:
	case PCAN_USBBUS14:
	case PCAN_USBBUS15:
	case PCAN_USBBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_USB");
		break;

	case PCAN_LANBUS1:
	case PCAN_LANBUS2:
	case PCAN_LANBUS3:
	case PCAN_LANBUS4:
	case PCAN_LANBUS5:
	case PCAN_LANBUS6:
	case PCAN_LANBUS7:
	case PCAN_LANBUS8:
	case PCAN_LANBUS9:
	case PCAN_LANBUS10:
	case PCAN_LANBUS11:
	case PCAN_LANBUS12:
	case PCAN_LANBUS13:
	case PCAN_LANBUS14:
	case PCAN_LANBUS15:
	case PCAN_LANBUS16:
		strcpy_s(buffer, MAX_PATH, "PCAN_LAN");
		break;

	default:
		strcpy_s(buffer, MAX_PATH, "UNKNOWN");
		break;
	}
}

void CanRecharge::GetFormattedError(TPCANStatus error, LPSTR buffer)
{
	// Gets the text using the GetErrorText API function. If the function success, the translated error is returned.
	// If it fails, a text describing the current error is returned.
	if (CAN_GetErrorText(error, 0x09, buffer) != PCAN_ERROR_OK)
		sprintf_s(buffer, MAX_PATH, "An error occurred. Error-code's text (%Xh) couldn't be retrieved", error);
}










