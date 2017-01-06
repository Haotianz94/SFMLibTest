/**
 *       @file  Wave.h
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

#ifndef __WAVE_H__
#define __WAVE_H__

#include "DllApi.h"
#include "InfoCenter.h"
#include "Shell.h"

#ifdef __GNUC__
#include <unistd.h>
#endif /* __GNUC__ */

using namespace std;

namespace asdk
{
/**
* @class CWave
* @brief Wave类，wav容器
*/
class ASDK_DLL CWave
{
	friend class CVadG723;
	friend class CVadOsf;

public:
    enum Type{PCM, A_LAW, MU_LAW};
    enum Channel{LEFT_CHANNEL, RIGHT_CHANNEL, SUM_CHANNEL, CAT_CHANNEL};
    /** @brief 从文件读取语音。
     *  @param szFileName 文件名
     *  @param channel = LEFT_CHANNEL表示读入左声道，RIGHT_CHANNEL表示读入右声道。
     *  @param bHead = true表示有头, 格式按文件头进行解析, 后面的参数失效。
               bHead = false表示无头, 格式按给定参数进行。
     *  @param type 表示wav数据的类型，可为PCM, A_LAW 和MU_LAW。
     *  @param nFs 表示采样率。
     *  @param nChannel 表示声道数，只对无头文件有效。
     */
    bool Read(const char *szFileName, Channel channel = LEFT_CHANNEL, bool bHead = true, Type type = PCM, int nFs = 8000, int nChannel = 1, int nMaxSample = -1);

    /** @brief 写入文件
      * @param szFileName 文件名
      */
    bool Write(const char *szFileName);

    /** @brief 删除相同数据，其长度大于等于nContinuseSample时删除
      * @param nContinuseSample表示采样数据的长度，默认长度为50
      */
    int DelSame(int nContinuseSample = 50);

    /** @brief Truncating the Data.
      * @param IniSample represent the initial sampling point.
      * @param nSample represent the length of the sampling points.
      */
    bool TruncData(int IniSample, int nSample);

    /** @brief Concatenating the Data */
    bool ConData(CWave &wave);

    /** @brief 将采样频率降低至原先的一半 */
    bool Decimate2();

    /** @brief 把给定的数据写入文件
      * @param szFileName 表示需要写入的文件名。
      * @param pData 表示数据的起始指针。
      * @param nSample 表示数据的采样长度。
      * @param nFs 表示采样率。
      */
    static bool WriteWav(const char *szFileName, short *pData, int nSample, int nFs = 8000);

    CWave(InfoSinkConfig &isc = defaultSinkConfig);
    CWave(short *pData, int nSample, int nFs, InfoSinkConfig &isc = defaultSinkConfig);
    virtual ~CWave();

    /** @brief 获得数据的采样长度 */
    int GetSampleNum() const
    {
        return m_nSample;
    }

    /** @brief 获得数据的采样率 */
    int GetFs() const
    {
        return m_nFs;
    }

    /** @brief 获得数据起始指针 */
    short* GetDataPtr() const
    {
        return m_pData;
    }

    /** @brief 获得错误号 */
    int GetLastError()
    {
        return m_le;
    }

    /** @brief 获得原始声道数 */
    int GetOrigChannelNum()
    {
        return m_nOrigChannel;
    }
protected:
    int m_nFs;  /**< @brief Sampling Rate */

    int m_nSample;  /**< @brief Number of Samples */
    short *m_pData; /**< @brief Speech Data Pointer */

    InfoAgent *m_ia;    /**< @brief Error Information */
    int m_le;           /**< @brief Error Number */
    int m_nOrigChannel; /**< @brief The original channel number */

    enum ErrorCode
    {
        SUCCESS          = 0,
        READ_FAILED      = 1,
        WRITE_FAILED     = 2,
        UNIMPLEMENTED    = 3,
        VAD_UNPROCESSED  = 4,
        NULL_DATA        = 5,

        ERROR_COUNT
    };

    //struct WavHead     /**< old wav header*/
    //{
    //    char    Riff[4];
    //    long    FileLength;
    //    char    Wave[4];
    //    char    fmt[4];
    //    long    TempBytes;
    //    short    PCM;
    //    short    nChannel;
    //    long    SampleRate;
    //    long    TransRate;
    //    short    Adjust;
    //    short    nBitsPerSample;
    //    char    cData[4];
    //    long    nBytes;
    //};

    //typedef unsigned long DWORD;
    typedef unsigned int DWORD;
    typedef unsigned short WORD;

    struct RIFFFormat
    {
        DWORD RIFF;         /**< ="RIFF" */
        DWORD nSize_8;      /**< = filesize - 8 */
        DWORD WAVE;         /**< ="WAVE" */
        DWORD fmt;          /**< ="fmt " */
        DWORD nFmtSize;     /**< =下一个结构体的大小 */
    };

    struct WaveFormat
    {
        WORD    wFormatTag;        /**< format type */
        WORD    nChannels;         /**< number of channels (i.e. mono, stereo...) */
        DWORD   nSamplesPerSec;    /**< sample rate */
        DWORD   nAvgBytesPerSec;   /**< for buffer estimation */
        WORD    nBlockAlign;       /**< block size of data */
    };

    /** specific waveform format structure for PCM data */
    struct PCMWaveFormat
    {
        WORD    wFormatTag;        /**< format type */
        WORD    nChannels;         /**< number of channels (i.e. mono, stereo...) */
        DWORD   nSamplesPerSec;    /**< sample rate */
        DWORD   nAvgBytesPerSec;   /**< for buffer estimation */
        WORD    nBlockAlign;       /**< block size of data */
        WORD    wBitsPerSample;
    };

    struct DataFormat
    {
        DWORD data;                /**< ="data";*/
        DWORD nDataSize;           /**< 纯数据长度*/
    };

private:
    /** @brief 读取Wav文件
      * @param fp 表示文件指针
      * @param channel 表示读取的声道
      */
    bool ReadWav(AutoFilePtr &fp, Channel channel, int nMaxSample);

    /** @brief 写入Wav文件
      * @param fp 表示文件指针
      */
    bool WriteWav(AutoFilePtr &fp);

    /** @brief 读取无头文件的Wav文件,此时只能读入单声道
      * @param fp   表示文件指针
      * @param type 数据类型，可为PCM, A_LAW 和MU_LAW。
      * @param nSmaple  表示数据采样长度
      */
    bool ReadRaw(AutoFilePtr &fp, Type type, int nSmaple, int nChannel, Channel channel = LEFT_CHANNEL);

};

}

#endif /* __WAVE_H__ */

