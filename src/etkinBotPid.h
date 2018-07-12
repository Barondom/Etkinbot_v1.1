
#ifndef etkinBotPid_h
#define etkinBotPid_h

#include <Arduino.h>
///@brief Class for EtkinClass
class EtkinPidClass
{
	public:
		EtkinPidClass();
    void
	private:
		int level;
		int line_left;
		int line_right;
		double distance;
		long duration;
};

#endif
