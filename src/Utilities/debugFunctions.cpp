#include "Utilities/debugFunctions.h"
#include "main.h"

namespace debug {
	void printOnController(std::string message) {
		// Controller1.Screen.clearScreen();
		Controller1.Screen.clearLine(3);
		Controller1.Screen.setCursor(3, 1);
		Controller1.Screen.print(message.c_str());
	}

	// Returns a string with leading and trailing zeroes
	std::string leadTrailZero(int leadLen, int trailLen, double num) {
		// Create format
		char formatTmp[4 + (int) log10(leadLen - 1) + 3 + (int) log10(trailLen) + 1];
		sprintf(formatTmp, "%%c%%0%dd%%.%df", leadLen - 1, trailLen);
		const char *form = ((std::string) formatTmp).c_str();
		// free(formatTmp);

		// Create string with leading and trailing zeroes
		char tmp[1 + leadLen + 1 + trailLen];
		char sign = (((num > 0) - (num < 0) > 0) ? '+' : '-');
		num = fabs(num);
		sprintf(tmp, form, sign, (int) num / 10, num - (int) (num / 10) * 10);
		std::string ret = tmp;

		// Return
		return ret;
	}
}
