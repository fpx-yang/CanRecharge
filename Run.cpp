#include "CanRecharge.h"

int main()
{
	TPCANHandle can1 = PCAN_USBBUS1;
	TPCANHandle can2 = PCAN_USBBUS2;
	TPCANHandle can3 = PCAN_USBBUS3;
	TPCANHandle can4 = PCAN_USBBUS4;
	TPCANHandle can5 = PCAN_USBBUS5;
	TPCANHandle can6 = PCAN_USBBUS6;


	uint64_t ts = 0;
	char* file_path = "./canfd.dat";
	char* out_path1 = "./out1.txt";
	char* out_path2 = "./out2.txt";
	char* out_path3 = "./out3.txt";
	char* out_path4 = "./out4.txt";
	char* out_path5 = "./out5.txt";
	char* out_path6 = "./out6.txt";



	TPCANBitrateFD BitrateFD = const_cast<LPSTR>("f_clock=80000000,nom_brp=2,nom_tseg1=63,nom_tseg2=16,nom_sjw=16,data_brp=2,data_tseg1=15,data_tseg2=4,data_sjw=4");
	

	// CanRecharge* start1 = new CanRecharge(can1,file_path1,ts,out_path1,17);

	CanRecharge start1(can1,file_path,ts,out_path1,0);
	CanRecharge start2(can2,file_path,ts,out_path2,10);
	CanRecharge start3(can3,file_path,ts,out_path3,20);
	CanRecharge start4(can4,file_path,ts,out_path4,30);
	CanRecharge start5(can5,file_path,ts,out_path5,40);
	CanRecharge start6(can6,file_path,ts,out_path6,50);

	start1.Init(can1,BitrateFD);
	start1.LoadMessage();
	start2.Init(can2,BitrateFD);
	start2.LoadMessage();
	start3.Init(can3,BitrateFD);
	start3.LoadMessage();
	start4.Init(can4,BitrateFD);
	start4.LoadMessage();
	start5.Init(can5,BitrateFD);
	start5.LoadMessage();
	start6.Init(can6,BitrateFD);
	start6.LoadMessage();
	std::cout<<"load over"<<std::endl;

	start1.RunThread();
	start2.RunThread();
	start3.RunThread();
	start4.RunThread();
	start5.RunThread();
	start6.RunThread();

	while (start1.CheckStop())
	{
	}
	std::cout<<"over"<<std::endl;

	// _getch();
	

	// std::thread t1(start.RunProcess);
	// sleep(2);
	// start.pause();
	// sleep(2);
	// start.resume();
	// sleep(10);
	// start.stop();
}