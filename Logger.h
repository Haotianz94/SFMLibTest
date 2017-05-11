#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <iostream>
#include <fstream>
#include <string>

//utility
#define REPORT(X) *Logger::getLogger() << #X << ": " << X << "\n"
#define PRINT(X) *Logger::getLogger() << X << "\n"
#define REPORTP(X) printf("%.3f\n", X); 
#define LOG *Logger::getLogger()
#define PRINT_CONSOLE 1;

class Logger
{
public:
	Logger();
	~Logger();
	static Logger* getLogger();

	friend Logger& operator << (Logger& logger, const char* str);
	friend Logger& operator << (Logger& logger, const char c);
	friend Logger& operator << (Logger& logger, const int num);
	friend Logger& operator << (Logger& logger, const unsigned num);
	friend Logger& operator << (Logger& logger, const double num);
	friend Logger& operator << (Logger& logger, const float num);
	friend Logger& operator << (Logger& logger, const std::string& str);

private:
	std::ofstream m_logStream;
	static Logger* m_logger;
};

#endif