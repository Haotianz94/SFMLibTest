#include "Configer.h"
#include <fstream>

using namespace std;

#define STRING_LEN 80
const string Configer::m_configFile = "config.ini";
Configer* Configer::m_configer = NULL;

const string* ConfigSection::findValue(const char* key) const
{
	for(auto& pair : m_pairs)
	{
		if(!strcmp(pair.key.c_str(), key))
		{
			return &pair.value;
		}
	}
	return NULL;
}

void ConfigSection::addKeyPair(const ConfigKeyPair& pair)
{
	m_pairs.push_back(pair);
}

Configer::~Configer()
{
	for(auto& sec : m_sections)
		delete sec;
	m_sections.clear();
	if(m_configer)
		delete m_configer;
}

Configer* Configer::getConfiger()
{
	if(!m_configer)
		m_configer = new Configer();
	return m_configer;
}

void Configer::init(const char* configFile)
{
	const char* realPath = configFile? configFile : m_configFile.c_str();
	ifstream fin(realPath);

	string buffer;
	while(!fin.eof())
	{
		getline(fin, buffer);
		parseIntoSections(buffer);
	}
	fin.close();
}

void Configer::parseIntoSections(string& line)
{
	const char* tmpBuff = strchr(line.c_str(), '[');

	if (tmpBuff)
	{
		const char* tmpEnd = strchr(tmpBuff, ']');
		unsigned startPos = static_cast<unsigned>(tmpBuff - line.c_str()) + 1;
		unsigned numChar = static_cast<unsigned>(tmpEnd - line.c_str()) - 1;
		string secName = line.substr(startPos, numChar);
		ConfigSection* newSec = new ConfigSection(secName);
		m_sections.push_back(newSec);
	}
	else
	{
		const char* tmpPairBuff = strchr(line.c_str(), '=');
		if (tmpPairBuff)
		{	
			ConfigKeyPair pair;
			unsigned numChar = static_cast<unsigned>(tmpPairBuff - line.c_str());
			pair.key = line.substr(0, numChar);

			const char* valueField = tmpPairBuff + 1;
			unsigned startPos = static_cast<unsigned>(valueField - line.c_str());
			numChar = static_cast<unsigned>(line.length() - startPos);

			pair.value = line.substr(startPos, numChar);
			if (m_sections.size())
			{
				ConfigSection* currentSection = m_sections.back();
				currentSection->addKeyPair(pair);
			}

		}
	}
}

const ConfigSection* Configer::findSection(const char* section) const
{
	for(auto& sec: m_sections)
	{
		if(!strcmp(sec->getName().c_str(), section))
		{
			return sec;
		}
	}
	return NULL;
}

bool Configer::getString(const char* section, const char* key, char* text) const
{
	const ConfigSection* sec = findSection(section);
	if(!sec)
		return false;
	const string* stringValue = sec->findValue(key);
	if(!stringValue)
		return false;
	strncpy(text, stringValue->c_str(), STRING_LEN);
	return true;
}

bool Configer::getInt(const char* section, const char* key, int& value) const
{
	char text[STRING_LEN];
	if(getString(section, key, text))
	{
		value = atoi(text);
		return true;
	}
	return false;
}

bool Configer::getBool(const char* section, const char* key, bool& value) const
{
	char text[STRING_LEN];
	if(getString(section, key, text))
	{
		value = ( atoi(text) == 1);
		return true;
	}
	return false;
}

bool Configer::getDouble(const char* section, const char* key, double& value) const
{
	char text[STRING_LEN];
	if(getString(section, key, text))
	{
		value = atof(text);
		return true;
	}
	return false;
}

bool Configer::getString(const char* section, const char* key, string& value) const
{
	char text[STRING_LEN];
	if(getString(section, key, text))
	{
		value = string(text);
		return true;
	}
	return false;
}

bool Configer::getArray(const char* section, const char* key, std::vector<int>& value) const
{
	char text[STRING_LEN];
	if(getString(section, key, text))
	{
		int s = 1, t = 1;
		while(text[t] != '}')
		{
			if(text[t] == ',')
			{
				value.push_back(atoi(text + s));
				s = ++t;
			}
			else
				t++;
		}
		value.push_back(atoi(text + s));
		return true;
	}
	return false;
}

bool Configer::getArray(const char* section, const char* key, std::vector<double>& value) const
{
	char text[STRING_LEN];
	if(getString(section, key, text))
	{
		int s = 1, t = 1;
		while(text[t] != '}')
		{
			if(text[t] == ',')
			{
				value.push_back(atof(text + s));
				s = ++t;
			}
			else
				t++;
		}
		value.push_back(atof(text + s));
		return true;
	}
	return false;
}
