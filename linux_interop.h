#include <iostream>
#include <thread>
#define UINT32 uint32_t
#include <limits.h>
#define MAX_PATH PATH_MAX
#define sprintf_s snprintf
#include <cstring>
#define strcpy_s(destination, destination_size, source) strcpy(destination, source)

#ifndef _getch_stub
#define _getch_stub
#include <termios.h>
#include <unistd.h>
static int _getch() {
	struct termios config;
	tcgetattr(STDIN_FILENO, &config);
	struct termios saved_config = config;
	config.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &config);
	int res = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &saved_config);
	return res;
}
#endif
