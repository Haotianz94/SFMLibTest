#include "Logger.h"
#include <ctime>

using namespace std;

Logger* Logger::m_logger = NULL;

Logger::Logger()
{
	time_t now = time(0);
	char logname[50];
	sprintf_s(logname, "log/log%ld.txt", now);
	m_logStream.open(logname);
}

Logger::~Logger()
{
	m_logStream << flush;
	m_logStream.close();
	if(m_logger)
		delete m_logger;
}

Logger* Logger::getLogger()
{
	if (!m_logger)
	  m_logger = new Logger();
	return m_logger;
}

Logger& operator << (Logger& logger, const char* str)
{
#ifdef PRINT_CONSOLE
	cout << str;
#endif
	logger.m_logStream << str;
	logger.m_logStream << flush;
	return logger;
}

Logger& operator << (Logger& logger, const char c)
{
#ifdef PRINT_CONSOLE
	cout << c;
#endif
	logger.m_logStream << c;
	logger.m_logStream << flush;
	return logger;
}

Logger& operator << (Logger& logger, const int num)
{
#ifdef PRINT_CONSOLE
	cout << num;
#endif
	logger.m_logStream << num;
	logger.m_logStream << flush;
	return logger;
}

Logger& operator << (Logger& logger, const unsigned num)
{
#ifdef PRINT_CONSOLE
	cout << num;
#endif
	logger.m_logStream << num;
	logger.m_logStream << flush;
	return logger;
}

Logger& operator << (Logger& logger, const double num)
{
#ifdef PRINT_CONSOLE
	cout << num;
#endif
	logger.m_logStream << num;
	logger.m_logStream << flush;
	return logger;
}

Logger& operator << (Logger& logger, const float num)
{
#ifdef PRINT_CONSOLE
	cout << num;
#endif
	logger.m_logStream << num;
	logger.m_logStream << flush;
	return logger;
}

Logger& operator << (Logger& logger, const string& str)
{
#ifdef PRINT_CONSOLE
	cout << str;
#endif
	logger.m_logStream << str;
	logger.m_logStream << flush;
	return logger;
}