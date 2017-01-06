/**
 *       @file  Wave.h
 *      @brief  ��д������wav����
 *
 *     @author  ����ǿ
 *
 *   @internal
 *     Created  2006��10��01��
 *    Revision  004
 *    Compiler  VC/g++
 *     Company  MAST, Dept. of E.E., Tsinghua Univ.
 *   Copyright  Copyright (c) 2010,  All rights reserved.
 *
 *   Revision history:
 *   001. 2006/10/1 by ����ǿ
 *         ������ʼ�汾��
 *   002. 2010/6/9 by ���
 *         ��дע�͡�
 *   003. 2010/7/3 by ������
 *         �޸�FCC�꣬��gcc 4.5���������б��뾯�档
 *   004. 2010/7/28 by ����
 *         ��Ӵ���������CWave��Ĺ��캯����Decimate2������
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
* @brief Wave�࣬wav����
*/
class ASDK_DLL CWave
{
	friend class CVadG723;
	friend class CVadOsf;

public:
    enum Type{PCM, A_LAW, MU_LAW};
    enum Channel{LEFT_CHANNEL, RIGHT_CHANNEL, SUM_CHANNEL, CAT_CHANNEL};
    /** @brief ���ļ���ȡ������
     *  @param szFileName �ļ���
     *  @param channel = LEFT_CHANNEL��ʾ������������RIGHT_CHANNEL��ʾ������������
     *  @param bHead = true��ʾ��ͷ, ��ʽ���ļ�ͷ���н���, ����Ĳ���ʧЧ��
               bHead = false��ʾ��ͷ, ��ʽ�������������С�
     *  @param type ��ʾwav���ݵ����ͣ���ΪPCM, A_LAW ��MU_LAW��
     *  @param nFs ��ʾ�����ʡ�
     *  @param nChannel ��ʾ��������ֻ����ͷ�ļ���Ч��
     */
    bool Read(const char *szFileName, Channel channel = LEFT_CHANNEL, bool bHead = true, Type type = PCM, int nFs = 8000, int nChannel = 1, int nMaxSample = -1);

    /** @brief д���ļ�
      * @param szFileName �ļ���
      */
    bool Write(const char *szFileName);

    /** @brief ɾ����ͬ���ݣ��䳤�ȴ��ڵ���nContinuseSampleʱɾ��
      * @param nContinuseSample��ʾ�������ݵĳ��ȣ�Ĭ�ϳ���Ϊ50
      */
    int DelSame(int nContinuseSample = 50);

    /** @brief Truncating the Data.
      * @param IniSample represent the initial sampling point.
      * @param nSample represent the length of the sampling points.
      */
    bool TruncData(int IniSample, int nSample);

    /** @brief Concatenating the Data */
    bool ConData(CWave &wave);

    /** @brief ������Ƶ�ʽ�����ԭ�ȵ�һ�� */
    bool Decimate2();

    /** @brief �Ѹ���������д���ļ�
      * @param szFileName ��ʾ��Ҫд����ļ�����
      * @param pData ��ʾ���ݵ���ʼָ�롣
      * @param nSample ��ʾ���ݵĲ������ȡ�
      * @param nFs ��ʾ�����ʡ�
      */
    static bool WriteWav(const char *szFileName, short *pData, int nSample, int nFs = 8000);

    CWave(InfoSinkConfig &isc = defaultSinkConfig);
    CWave(short *pData, int nSample, int nFs, InfoSinkConfig &isc = defaultSinkConfig);
    virtual ~CWave();

    /** @brief ������ݵĲ������� */
    int GetSampleNum() const
    {
        return m_nSample;
    }

    /** @brief ������ݵĲ����� */
    int GetFs() const
    {
        return m_nFs;
    }

    /** @brief ���������ʼָ�� */
    short* GetDataPtr() const
    {
        return m_pData;
    }

    /** @brief ��ô���� */
    int GetLastError()
    {
        return m_le;
    }

    /** @brief ���ԭʼ������ */
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
        DWORD nFmtSize;     /**< =��һ���ṹ��Ĵ�С */
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
        DWORD nDataSize;           /**< �����ݳ���*/
    };

private:
    /** @brief ��ȡWav�ļ�
      * @param fp ��ʾ�ļ�ָ��
      * @param channel ��ʾ��ȡ������
      */
    bool ReadWav(AutoFilePtr &fp, Channel channel, int nMaxSample);

    /** @brief д��Wav�ļ�
      * @param fp ��ʾ�ļ�ָ��
      */
    bool WriteWav(AutoFilePtr &fp);

    /** @brief ��ȡ��ͷ�ļ���Wav�ļ�,��ʱֻ�ܶ��뵥����
      * @param fp   ��ʾ�ļ�ָ��
      * @param type �������ͣ���ΪPCM, A_LAW ��MU_LAW��
      * @param nSmaple  ��ʾ���ݲ�������
      */
    bool ReadRaw(AutoFilePtr &fp, Type type, int nSmaple, int nChannel, Channel channel = LEFT_CHANNEL);

};

}

#endif /* __WAVE_H__ */

