/**
 *       @file  Wave.cpp
 *      @brief  读写、处理wav数据
 *
 *     @author  张卫强
 *
 *   @internal
 *     Created  2006年10月01日
 *    Revision  004
 *    Compiler  VC/g++
 *     Company  MAST, Dept. of E.E., Tsinghua Univ.
 *   Copyright  Copyright (c) 2010,  All rights reserved.
 *
 *   Revision history:
 *   001. 2006/10/1 by 张卫强
 *         创建初始版本。
 *   002. 2010/6/9 by 杨径舟
 *         加写注释。
 *   003. 2010/7/3 by 单煜翔
 *         修改FCC宏，在gcc 4.5下消除所有编译警告。
 *   004. 2010/7/28 by 蔡猛
 *         添加从数据生成CWave类的构造函数和Decimate2函数。
 * =====================================================================================
 */

#include "StdAfx.h"
#include "Wave.h"
#include "G711c.h"
#include <string.h>
#include <limits.h>
using namespace asdk;

namespace asdk
{

/** @brief Construction */
CWave::CWave(InfoSinkConfig &isc /* = defaultSinkConfig */):
    m_nFs(0), m_nSample(0), m_pData(NULL), m_nOrigChannel(0)
{
    m_ia = InfoCenter::Register("CWave", isc);
}

/** @brief Construct a wave from another wave */
CWave::CWave(short *pData, int nSample, int nFs, InfoSinkConfig &isc):
    m_nFs(nFs), m_nSample(nSample)
{
    m_ia = InfoCenter::Register("CWave", isc);

    m_pData = new short[nSample];
    memcpy(m_pData, pData, 	nSample * sizeof(short));
}

CWave::~CWave()
{
	delete []m_pData;
    InfoCenter::UnRegister(m_ia);
}

bool CWave::Read(const char *szFileName, Channel channel, bool bHead, Type type, int nFs, int nChannel, int nMaxSample /* = -1 */)
{
    assert(szFileName != NULL);

    ////ifstream fs(szFileName, ios::binary);
    AutoFilePtr fp(fopen(szFileName, "rb"));


    if (fp == NULL)
    {
        m_le = m_ia->Error(READ_FAILED, "Cannot open the wav file.");
        return false;
    }

    if (bHead)
    {
        if (!ReadWav(fp, channel, nMaxSample))
        {
            ////fclose(fp);
            return false;
        }
    }
    else
    {
        m_nFs = nFs;
        fseek(fp, 0L, SEEK_END);
        long len = ftell(fp);
        fseek(fp, 0L, SEEK_SET);
        int nBytePerSample = ((type == PCM) ? 2 : 1);
        m_nSample = len / nBytePerSample;
        if (nMaxSample > 0 && m_nSample > nMaxSample)
        {
             m_nSample = nMaxSample;
        }
        if (!ReadRaw(fp, type, m_nSample, nChannel, channel))
        {
            return false;
        }
        m_nOrigChannel = nChannel;
    }

    m_le = SUCCESS;
    return true;
}

bool CWave::Write(const char *szFileName)
{
    assert(szFileName != NULL);

    AutoFilePtr fp(fopen(szFileName, "wb"));

    if (fp == NULL)
    {
        m_le = m_ia->Error(WRITE_FAILED, "Cannot write in the file.");
        return false;
    }

    WriteWav(fp);
    //if (CheckExtName(szFileName, "wav"))
    //{
    //    WriteWav(fs);
    //}
    //else
    //{
    //    cout << "unimplemented." << endl;
    // m_ia->Info(UNIMPLEMENTED, "unimplemented.\n");
    //}

    m_le = SUCCESS;
    return true;
}

int CWave::DelSame(int nContinuseSample)
{
	if(nContinuseSample <= 1)
	{
		return -1;
	}
	int nDel = 0;
	int i = 0;
	int nSample = m_nSample - nContinuseSample;
	while(i < nSample)
	{
		if(m_pData[i+1] == m_pData[i])
		{
			bool bDel = false;
			int nLen;
			for(int j=i+2; j<nSample; j++)
			{
				if((m_pData[i] != m_pData[j]) || (j == nSample - 1))
				{
					nLen = j - i;
					if(nLen >= nContinuseSample)
					{
						bDel = true;
					}
					break;
				}
			}
			if(bDel)
			{
				memmove(&m_pData[i+1], &m_pData[i+nLen], sizeof(short) * (nSample-i-nLen));
				nLen--;
				nDel += nLen;
				nSample -= nLen;
				m_nSample -= nLen;
			}
		}
		i++;
	}
	return nDel;
}

#define WAVE_FORMAT_PCM        0x0001
#define WAVE_FORMAT_ADPCM      0x0002
#define WAVE_FORMAT_ALAW       0x0006
#define WAVE_FORMAT_MULAW      0x0007

//
// structures for manipulating RIFF headers
//
/*
#define FCC(ch4) ((((DWORD)(ch4) & 0xFF) << 24) |     \
	(((DWORD)(ch4) & 0xFF00) << 8) |    \
	(((DWORD)(ch4) & 0xFF0000) >> 8) |  \
	(((DWORD)(ch4) & 0xFF000000) >> 24))
*/

#define FCC(ch1, ch2, ch3, ch4) ((((DWORD)(ch4)) << 24) |     \
	(((DWORD)(ch3)) << 16) |    \
	(((DWORD)(ch2)) << 8) |  \
	(((DWORD)(ch1))))

bool CWave::ReadWav(AutoFilePtr &fp, Channel channel, int nMaxSample)
{
    RIFFFormat riffChunk;
    PCMWaveFormat waveChunk;
    DataFormat dataChunk;

    if (!fread((char*)&riffChunk, sizeof(RIFFFormat), 1, fp))
    {
        m_le = m_ia->Error(READ_FAILED, "Cannot read the RiffChunk.");
        return false;
    }
    if ((riffChunk.RIFF != FCC('R', 'I', 'F', 'F')) ||
        (riffChunk.WAVE != FCC('W', 'A', 'V', 'E')) ||
        (riffChunk.fmt != FCC('f', 'm', 't', ' ')))
    {
        m_le = m_ia->Error(READ_FAILED, "wav format seems wrong.");
        return false;
    }

    if (!fread((char*)&waveChunk, sizeof(PCMWaveFormat), 1, fp))
    {
        m_le = m_ia->Error(READ_FAILED, "cannot read the WavChunk.");
        return false;
    }

    while (1)
    {
        if (feof(fp))
        {
            //// cout << "Wav format seems wrong!" << endl;
            m_le = m_ia->Error(READ_FAILED, "wav format seems wrong.");
            return false;
        }

        char szData[4] = {'d', 'a', 't', 'a'};
        char ch;

        if (fread(&ch, sizeof(char), 1, fp) != 1)
        {
            m_le = m_ia->Error(READ_FAILED, "read wav failed.");
            return false;
        }
        if (ch == szData[0])
        {
            if (fread(&ch, sizeof(char), 1, fp) != 1)
            {
                m_le = m_ia->Error(READ_FAILED, "read wav failed.");
                return false;
            }
            if (ch == szData[1])
            {
                if (fread(&ch, sizeof(char), 1, fp) != 1)
                {
                    m_le = m_ia->Error(READ_FAILED, "read wav failed.");
                    return false;
                }
                if (ch == szData[2])
                {
                    if (fread(&ch, sizeof(char), 1, fp) != 1)
                    {
                        m_le = m_ia->Error(READ_FAILED, "read wav failed.");
                        return false;
                    }
                    if (ch == szData[3])
                    {
                        break;
                    }
                }
            }
        }
        memcpy((char*)&dataChunk.data, szData, sizeof(dataChunk.data));
    }

    if (fread((char*)&dataChunk.nDataSize, sizeof(dataChunk.nDataSize), 1, fp) != 1)
    {
        m_le = m_ia->Error(READ_FAILED, "read wav failed.");
        return false;
    }

    unsigned long loc = ftell(fp);
    fseek(fp, 0, SEEK_END);
    unsigned long len = ftell(fp);    /** @brief< 文件长度 */
    fseek(fp, loc, SEEK_SET);

    if (len <= 44)
    {
        m_le = m_ia->Error(READ_FAILED, "wav format is wrong. The size of the wav is too short");
        return false;
    }

    /** @brief 标准文件应该有dataChunk.nDataSize + loc == riffChunk.nSize_8 + 8, 此处稍微放松要求 */
    if (dataChunk.nDataSize + loc != riffChunk.nSize_8 + 8)    /** @brief< 按两个标示分别计算文件大小 */
    {
        //dataChunk.nDataSize = riffChunk.nSize_8 + 8 - loc;
        //// cout << "Wav format seems wrong!" << endl;
        m_le = m_ia->Error(READ_FAILED, "wav format seems wrong.");
		return false;
    }
    if (dataChunk.nDataSize + loc != len || dataChunk.nDataSize > UINT_MAX - loc)
    {
		if (len - loc <= 0)
		{
			m_le = m_ia->Error(READ_FAILED, "wav format is wrong.");
			return false;
		}
		dataChunk.nDataSize = len - loc;
        //// cout << "Wav format seems wrong!" << endl;
        m_le = m_ia->Error(READ_FAILED, "wav format seems wrong. The size of the wav is short than the length it says.\nUse the true size.");
    }

    m_nFs = waveChunk.nSamplesPerSec;
    m_nOrigChannel = waveChunk.nChannels;
	// zwq 20090911
	if (m_nFs <= 0)
	{
		m_le = m_ia->Error(READ_FAILED, "wav sampling rate <= 0.");
		return false;
	}

	if (m_nOrigChannel <= 0 || m_nOrigChannel > 2)
	{
		m_le = m_ia->Error(READ_FAILED, "wav channel <=0 || > 2.");
		return false;
	}
    int nBytePerSample = waveChunk.wBitsPerSample / 8;
    if (waveChunk.nAvgBytesPerSec != (nBytePerSample * m_nOrigChannel * m_nFs))
    {
        m_le = m_ia->Error(READ_FAILED, "wav format is wrong!");
        return false;
    }

    m_nSample = dataChunk.nDataSize / nBytePerSample;
    dataChunk.nDataSize = m_nSample * nBytePerSample;    /**< 发现有时dataChunk.nDataSize比m_nSample * nBytePerSample多1个Byte */

    ////////if (waveChunk.nChannels != 1)
    ////////{
    ////////    // cout << "Stereo unimplemented." << endl;
 ////////       m_le = m_ia->Info(UNIMPLEMENTED, "Stereo unimplemented.\n");
    ////////    return false;
    ////////}

    if (m_nSample <= 0)
    {
        m_le = m_ia->Error(READ_FAILED, "wav format is wrong!");
        return false;
    }

    if (m_pData != NULL)
    {
        delete []m_pData;
        m_pData = NULL;
    }

    if (nMaxSample > 0 && m_nSample > nMaxSample)
    {
        m_nSample = nMaxSample;
    }

    m_pData = new short[m_nSample];

    switch(waveChunk.wFormatTag)
    {
    case WAVE_FORMAT_PCM:
        if (waveChunk.wBitsPerSample == 16)
        {
            if (fread((char*)m_pData, dataChunk.nDataSize, 1, fp) != 1)
            {
                m_le = m_ia->Error(READ_FAILED, "read wav failed.");
                return false;
            }
        }
        else if (waveChunk.wBitsPerSample == 8)	// zwq 20091020
        {
            // 先将数据存储在m_pData的后一半空间内, 然后进行转换
            unsigned char* pcData = (unsigned char* )m_pData + dataChunk.nDataSize;
            if(fread((char*)pcData, dataChunk.nDataSize, 1, fp) != 1)
            {
                m_le = m_ia->Error(READ_FAILED, "read wav failed.");
                return false;
            }

            for (int i=0; i<m_nSample; i++)
            {
                m_pData[i] = ((short)pcData[i] - 127) * 256;
            }
        }
        else
        {
            //// cout << "Wav format seems wrong!" << endl;
            m_le = m_ia->Error(READ_FAILED, "Wav format seems wrong.");
            return false;
        }
        break;

    case WAVE_FORMAT_ALAW:
        if (waveChunk.wBitsPerSample == 8)
        {
            /** @brief 先将数据存储在m_pData的后一半空间内, 然后进行转换 */
            unsigned char* pcData = (unsigned char* )m_pData + dataChunk.nDataSize;
            if (fread((char*)pcData, dataChunk.nDataSize, 1, fp) != 1)
            {
                m_le = m_ia->Error(READ_FAILED, "read wav failed.");
                return false;
            }

            for (int i=0; i<m_nSample; i++)
            {
                m_pData[i] = alaw2linear(pcData[i]);
            }
        }
        else
        {
            //// cout << "Wav format seems wrong!" << endl;
            m_le = m_ia->Error(READ_FAILED, "Wav format seems wrong.");
            return false;
        }
        break;

    case WAVE_FORMAT_MULAW:
        if (waveChunk.wBitsPerSample == 8)
        {
            /** @brief 先将数据存储在m_pData的后一半空间内, 然后进行转换 */
            unsigned char* pcData = (unsigned char* )m_pData + dataChunk.nDataSize;
            if (fread((char*)pcData, dataChunk.nDataSize, 1, fp) != 1)
            {
                m_le = m_ia->Error(READ_FAILED, "read wav failed.");
                return false;
            }

            for (int i=0; i<m_nSample; i++)
            {
                m_pData[i] = ulaw2linear(pcData[i]);
            }
        }
        else
        {
            //// cout << "Wav format seems wrong!" << endl;
            m_le = m_ia->Error(READ_FAILED, "Wav format seems wrong.");
            return false;
        }
        break;

    default:
        //// cout << "Wav format seems wrong!" << endl;
        m_le = m_ia->Error(READ_FAILED, "Wav format seems wrong.");
        return false;
        break;
    }

    /** @brief Determining whether the wav file is mono channel */
    if (waveChunk.nChannels == 1)
    {
        return true;
    }
    else if (waveChunk.nChannels == 2)
    {
        int nHalfSamples;
        short *pBuf = NULL;

        switch (channel)
        {
        case LEFT_CHANNEL:
            m_nSample = (m_nSample >> 1);
            for (int i = 1; i < m_nSample; i++)
            {
                m_pData[i] = m_pData[i << 1];
            }
            break;
        case RIGHT_CHANNEL:
            m_nSample = (m_nSample >> 1);
            for (int i = 0; i < m_nSample; i++)
            {
                m_pData[i] = m_pData[(i << 1) + 1];
            }
            break;
        case SUM_CHANNEL:
            m_nSample = (m_nSample >> 1);
            for (int i = 0; i < m_nSample; i++)
            {
                int i2 = i << 1;
                int sum = (int)m_pData[i2] + m_pData[i2 + 1];
                if (sum > SHRT_MAX)
                {
                    m_pData[i] = SHRT_MAX;
                }
                else if (sum < SHRT_MIN)
                {
                    m_pData[i] = SHRT_MIN;
                }
                else
                {
                    m_pData[i] = (short) sum;
                }
            }
            break;
        case CAT_CHANNEL:
            nHalfSamples = (m_nSample >> 1);
            pBuf = new short[max(nHalfSamples, 1)];

            for (int i = 0; i < nHalfSamples; i++)
            {
                m_pData[i] = m_pData[i << 1];
                pBuf[i] = m_pData[(i << 1) + 1];
            }
            for (int i = 0; i < nHalfSamples; i++)
            {
                m_pData[i + nHalfSamples] = pBuf[i];
            }
            delete []pBuf;
            pBuf = NULL;
            break;
        default:
            m_le = m_ia->Info(UNIMPLEMENTED, "Unimplemented.\n");
            return false;
        }

        return true;
    }
    else
    {
        m_le = m_ia->Info(UNIMPLEMENTED, "Unimplemented.\n");
        return false;
    }
}

bool CWave::WriteWav(AutoFilePtr &fp)
{
    RIFFFormat riffChunk;
    PCMWaveFormat waveChunk;
    DataFormat dataChunk;

    riffChunk.RIFF = FCC('R', 'I', 'F', 'F');   // FCC('RIFF');
    riffChunk.nSize_8 = m_nSample * 2 + sizeof(PCMWaveFormat) + sizeof(RIFFFormat);
    riffChunk.WAVE = FCC('W', 'A', 'V', 'E');  // FCC('WAVE');
    riffChunk.fmt = FCC('f', 'm', 't', ' ');   // FCC('fmt ');
    riffChunk.nFmtSize = sizeof(PCMWaveFormat);

    waveChunk.wFormatTag = WAVE_FORMAT_PCM;
    waveChunk.nChannels = 1;
    waveChunk.nSamplesPerSec = m_nFs;
    waveChunk.nAvgBytesPerSec = m_nFs * 2;
    waveChunk.nBlockAlign = 2;
    waveChunk.wBitsPerSample = 16;

    dataChunk.data = FCC('d', 'a', 't', 'a');  // FCC('data');
    dataChunk.nDataSize = m_nSample * 2;

    if (!fwrite((char*)&riffChunk, sizeof(RIFFFormat), 1, fp))
    {
        return false;
    }

    if (!fwrite((char*)&waveChunk, sizeof(PCMWaveFormat), 1, fp))
    {
        return false;
    }

    if (!fwrite((char*)&dataChunk, sizeof(DataFormat), 1, fp))
    {
        return false;
    }

    if (!fwrite((char*)m_pData, m_nSample * 2, 1, fp))
    {
        return false;
    }

    return true;
}

bool CWave::WriteWav(const char *szFileName, short *pData, int nSample, int nFs)
{
    assert(szFileName != NULL);

    AutoFilePtr fp(fopen(szFileName, "wb"));

    if (fp == NULL)
    {
        //m_le = m_ia->Error(WRITE_FAILED, "Cannot write in the file.");
        return false;
    }

    RIFFFormat riffChunk;
    PCMWaveFormat waveChunk;
    DataFormat dataChunk;

    riffChunk.RIFF = FCC('R', 'I', 'F', 'F');   // FCC('RIFF');
    riffChunk.nSize_8 = nSample * 2 + sizeof(PCMWaveFormat) + sizeof(RIFFFormat);
    riffChunk.WAVE = FCC('W', 'A', 'V', 'E');   // FCC('WAVE');
    riffChunk.fmt = FCC('f', 'm', 't', ' ');   // FCC('fmt ');
    riffChunk.nFmtSize = sizeof(PCMWaveFormat);

    waveChunk.wFormatTag = WAVE_FORMAT_PCM;
    waveChunk.nChannels = 1;
    waveChunk.nSamplesPerSec = nFs;
    waveChunk.nAvgBytesPerSec = nFs * 2;
    waveChunk.nBlockAlign = 2;
    waveChunk.wBitsPerSample = 16;

    dataChunk.data = FCC('d', 'a', 't', 'a');   // FCC('data');
    dataChunk.nDataSize = nSample * 2;

    if (!fwrite((char*)&riffChunk, sizeof(RIFFFormat), 1, fp))
    {
        return false;
    }

    if (!fwrite((char*)&waveChunk, sizeof(PCMWaveFormat), 1, fp))
    {
        return false;
    }

    if (!fwrite((char*)&dataChunk, sizeof(DataFormat), 1, fp))
    {
        return false;
    }

    if (!fwrite((char*)pData, nSample * 2, 1, fp))
    {
        return false;
    }

    return true;
}

#undef WAVE_FORMAT_PCM
#undef WAVE_FORMAT_ADPCM
#undef WAVE_FORMAT_ALAW
#undef WAVE_FORMAT_MULAW
#undef FCC


bool CWave::ReadRaw(AutoFilePtr &fp, Type type, int nSmaple, int nChannel, Channel channel)
{
    delete []m_pData;
    m_pData = new short[m_nSample];
    unsigned char* pcData;

    switch(type)
    {
    case PCM:
        if (fread((char*)m_pData, nSmaple * 2, 1, fp) != 1)
        {
            m_le = m_ia->Error(READ_FAILED, "read wav failed.");
            return false;
        }
        break;

    case A_LAW:
        /** @brief 先将数据存储在m_pData的后一半空间内, 然后进行转换 */
        pcData = (unsigned char* )m_pData + nSmaple;
        if (fread((char*)pcData, nSmaple, 1, fp) != 1)
        {
            m_le = m_ia->Error(READ_FAILED, "read wav failed.");
            return false;
        }

        for (int i=0; i<m_nSample; i++)
        {
            m_pData[i] = alaw2linear(pcData[i]);
        }
        break;

    case MU_LAW:
        /** @brief 先将数据存储在m_pData的后一半空间内, 然后进行转换 */
        pcData = (unsigned char* )m_pData + nSmaple;
        if (fread((char*)pcData, nSmaple, 1, fp) != 1)
        {
            m_le = m_ia->Error(READ_FAILED, "read wav failed.");
            return false;
        }

        for (int i=0; i<m_nSample; i++)
        {
            m_pData[i] = ulaw2linear(pcData[i]);
        }
        break;

    default:
        //// cout << "Type format seems wrong!" << endl;
        m_le = m_ia->Error(READ_FAILED, "Type format seems wrong.");
        return false;
        break;
    }

    /** @brief Determining whether the wav file is mono channel */
    if (nChannel == 1)
    {
        return true;
    }
    else if (nChannel == 2)
    {
        int nHalfSamples;
        short *pBuf = NULL;

        switch (channel)
        {
        case LEFT_CHANNEL:
            m_nSample = (m_nSample >> 1);
            for (int i = 1; i < m_nSample; i++)
            {
                m_pData[i] = m_pData[i << 1];
            }
            break;
        case RIGHT_CHANNEL:
            m_nSample = (m_nSample >> 1);
            for (int i = 0; i < m_nSample; i++)
            {
                m_pData[i] = m_pData[(i << 1) + 1];
            }
            break;
        case SUM_CHANNEL:
            m_nSample = (m_nSample >> 1);
            for (int i = 0; i < m_nSample; i++)
            {
                int i2 = i << 1;
                int sum = (int)m_pData[i2] + m_pData[i2 + 1];
                if (sum > SHRT_MAX)
                {
                    m_pData[i] = SHRT_MAX;
                }
                else if (sum < SHRT_MIN)
                {
                    m_pData[i] = SHRT_MIN;
                }
                else
                {
                    m_pData[i] = (short) sum;
                }
            }
            break;
        case CAT_CHANNEL:
            nHalfSamples = (m_nSample >> 1);
            pBuf = new short[max(nHalfSamples, 1)];

            for (int i = 0; i < nHalfSamples; i++)
            {
                m_pData[i] = m_pData[i << 1];
                pBuf[i] = m_pData[(i << 1) + 1];
            }
            for (int i = 0; i < nHalfSamples; i++)
            {
                m_pData[i + nHalfSamples] = pBuf[i];
            }
            delete []pBuf;
            pBuf = NULL;
            break;
        default:
            m_le = m_ia->Info(UNIMPLEMENTED, "Unimplemented.\n");
            return false;
        }

        return true;
    }
    else
    {
        m_le = m_ia->Info(UNIMPLEMENTED, "Unimplemented.\n");
        return false;
    }
}

bool CWave::TruncData(int IniSample, int nSample)
{
    if (m_pData == NULL)
    {
        m_le = m_ia->Error(NULL_DATA, "Please read the wav data first.");
        return false;
    }

	if (IniSample >= m_nSample)
	{
		m_le = m_ia->Error(NULL_DATA, "IniSample >= m_nSample.");
		return false;
	}

	if (nSample <= 0)
	{
		m_le = m_ia->Error(NULL_DATA, "nSample <= 0.");
		return false;
	}

	/** @brief Determining the length of sampling points */
    if (IniSample + nSample > m_nSample)
    {
        m_nSample -= IniSample;
    }
    else
    {
        m_nSample = nSample;
    }

    /** @brief Truncating the data */
    memmove(m_pData, (m_pData + IniSample), (sizeof(short) * m_nSample));

    return true;
}

bool CWave::ConData(CWave &wave)
{
    if (m_pData == NULL)
    {
        m_le = m_ia->Error(NULL_DATA, "Please read the wav data first.");
        return false;
    }

    int SamplingRate;
    SamplingRate = wave.GetFs();
    if (m_nFs != SamplingRate)
    {
        m_le = m_ia->Error(UNIMPLEMENTED, "Cannot concatenate the data with different sampling rate.");
        return false;
    }


    /** @brief Getting the pointer of the data */
    short *pData = wave.GetDataPtr();
    if (pData == NULL)
    {
        m_le = m_ia->Error(NULL_DATA, "Please read the wav data first.");
        return false;
    }

    /** @brief Getting the length of data */
    int nData = wave.GetSampleNum();
    if (nData == 0)
    {
        return true;
    }

    /** @brief Concatenating two wave data */
    short *pConData = new short[m_nSample + nData];
    memcpy(pConData, m_pData, (sizeof(short) * m_nSample));
    memcpy((pConData + m_nSample), pData, (sizeof(short) * nData));
    delete []m_pData;
    m_pData = pConData;
    m_nSample += nData; /**< @brief renew the data length*/

    return true;
}

bool CWave::Decimate2()
{
    const int filterSize = 181;
    float filter[filterSize] ={
        (float)0.000200 ,
        (float)-0.000150,
        (float)-0.000279,
        (float)0.000024 ,
        (float)0.000309 ,
        (float)0.000125 ,
        (float)-0.000276,
        (float)-0.000273,
        (float)0.000172 ,
        (float)0.000392 ,
        (float)0.000000 ,
        (float)-0.000445,
        (float)-0.000222,
        (float)0.000398 ,
        (float)0.000454 ,
        (float)-0.000230,
        (float)-0.000637,
        (float)-0.000054,
        (float)0.000706 ,
        (float)0.000416 ,
        (float)-0.000603,
        (float)-0.000779,
        (float)0.000302 ,
        (float)0.001044 ,
        (float)0.000175 ,
        (float)-0.001105,
        (float)-0.000751,
        (float)0.000885 ,
        (float)0.001294 ,
        (float)-0.000361,
        (float)-0.001645,
        (float)-0.000408,
        (float)0.001655 ,
        (float)0.001280 ,
        (float)-0.001229,
        (float)-0.002046,
        (float)0.000367 ,
        (float)0.002474 ,
        (float)0.000811 ,
        (float)-0.002365,
        (float)-0.002071,
        (float)0.001616 ,
        (float)0.003103 ,
        (float)-0.000270,
        (float)-0.003581,
        (float)-0.001462,
        (float)0.003255 ,
        (float)0.003220 ,
        (float)-0.002024,
        (float)-0.004560,
        (float)0.000000 ,
        (float)0.005045 ,
        (float)0.002477 ,
        (float)-0.004362,
        (float)-0.004880,
        (float)0.002427 ,
        (float)0.006587 ,
        (float)0.000550 ,
        (float)-0.007017,
        (float)-0.004056,
        (float)0.005777 ,
        (float)0.007335 ,
        (float)-0.002801,
        (float)-0.009525,
        (float)-0.001577,
        (float)0.009831 ,
        (float)0.006610 ,
        (float)-0.007726,
        (float)-0.011228,
        (float)0.003121 ,
        (float)0.014203 ,
        (float)0.003530 ,
        (float)-0.014373,
        (float)-0.011205,
        (float)0.010877 ,
        (float)0.018398 ,
        (float)-0.003365,
        (float)-0.023275,
        (float)-0.007871,
        (float)0.023848 ,
        (float)0.021885 ,
        (float)-0.018065,
        (float)-0.037169,
        (float)0.003519 ,
        (float)0.051875 ,
        (float)0.024193 ,
        (float)-0.064095,
        (float)-0.080483,
        (float)0.072178 ,
        (float)0.309445 ,
        (float)0.425024 ,
        (float)0.309445 ,
        (float)0.072178 ,
        (float)-0.080483,
        (float)-0.064095,
        (float)0.024193 ,
        (float)0.051875 ,
        (float)0.003519 ,
        (float)-0.037169,
        (float)-0.018065,
        (float)0.021885 ,
        (float)0.023848 ,
        (float)-0.007871,
        (float)-0.023275,
        (float)-0.003365,
        (float)0.018398 ,
        (float)0.010877 ,
        (float)-0.011205,
        (float)-0.014373,
        (float)0.003530 ,
        (float)0.014203 ,
        (float)0.003121 ,
        (float)-0.011228,
        (float)-0.007726,
        (float)0.006610 ,
        (float)0.009831 ,
        (float)-0.001577,
        (float)-0.009525,
        (float)-0.002801,
        (float)0.007335 ,
        (float)0.005777 ,
        (float)-0.004056,
        (float)-0.007017,
        (float)0.000550 ,
        (float)0.006587 ,
        (float)0.002427 ,
        (float)-0.004880,
        (float)-0.004362,
        (float)0.002477 ,
        (float)0.005045 ,
        (float)0.000000 ,
        (float)-0.004560,
        (float)-0.002024,
        (float)0.003220 ,
        (float)0.003255 ,
        (float)-0.001462,
        (float)-0.003581,
        (float)-0.000270,
        (float)0.003103 ,
        (float)0.001616 ,
        (float)-0.002071,
        (float)-0.002365,
        (float)0.000811 ,
        (float)0.002474 ,
        (float)0.000367 ,
        (float)-0.002046,
        (float)-0.001229,
        (float)0.001280 ,
        (float)0.001655 ,
        (float)-0.000408,
        (float)-0.001645,
        (float)-0.000361,
        (float)0.001294 ,
        (float)0.000885 ,
        (float)-0.000751,
        (float)-0.001105,
        (float)0.000175 ,
        (float)0.001044 ,
        (float)0.000302 ,
        (float)-0.000779,
        (float)-0.000603,
        (float)0.000416 ,
        (float)0.000706 ,
        (float)-0.000054,
        (float)-0.000637,
        (float)-0.000230,
        (float)0.000454 ,
        (float)0.000398 ,
        (float)-0.000222,
        (float)-0.000445,
        (float)0.000000 ,
        (float)0.000392 ,
        (float)0.000172 ,
        (float)-0.000273,
        (float)-0.000276,
        (float)0.000125 ,
        (float)0.000309 ,
        (float)0.000024 ,
        (float)-0.000279,
        (float)-0.000150,
        (float)0.000200
    };


    int newSampleNum = m_nSample >> 1;
    int newFs = m_nFs >> 1;

    short *newData = new short [newSampleNum];

    for (int i=0; i<newSampleNum; i++)
    {
        int j, k;
        int ii = i << 1;
        j = min( ii, m_nSample-1 );
        k = ii - j;
        float sum = 0;
        for (; j >= max( 0, ii-filterSize+1 ) ; j--, k++)
        {
            sum += m_pData[j] * filter[k];
        }

        if (sum > SHRT_MAX)
        {
            newData[i] = SHRT_MAX;
        }
        else if (sum < SHRT_MIN)
        {
            newData[i] = SHRT_MIN;
        }
        else
        {
            newData[i] = (short) sum;
        }
    }

    delete [] m_pData;
    m_pData = newData;
    m_nSample = newSampleNum;
    m_nFs = newFs;

    return true;
}

}

