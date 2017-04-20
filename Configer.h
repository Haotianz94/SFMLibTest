#ifndef _CONFIGER_H_
#define _CONFIGER_H_

#include <string>
#include <vector>

struct ConfigKeyPair
{
	std::string key;
	std::string value;
};

class ConfigSection
{
private:
	std::string m_secName;
	std::vector<ConfigKeyPair> m_pairs;

public:
	ConfigSection(std::string name): m_secName(name){}
	const std::string& getName() 
	{ 
		return m_secName;
	}
	void addKeyPair(const ConfigKeyPair& pair);
	const std::string* findValue(const char* key) const;
};

class Configer
{
private:
	Configer() 
	{
		init();
	}
	~Configer();

	static Configer* m_configer;
	const static std::string m_configFile;
	std::vector<ConfigSection*> m_sections;

	const ConfigSection* findSection(const char* section) const;
	bool getString(const char* section, const char* key, char* text) const;
	void parseIntoSections(std::string& buffer);

public:
	static Configer* getConfiger();
	void init(const char* configFile = NULL);

	bool getInt(const char* section, const char* key, int& value) const;
	bool getBool(const char* section, const char* key, bool& value) const;
	bool getDouble(const char* section, const char* key, double& value) const;
	bool getString(const char* section, const char* key, std::string& value) const;
};


#endif