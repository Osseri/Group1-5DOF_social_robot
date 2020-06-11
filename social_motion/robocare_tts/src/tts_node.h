#ifndef TTS_NODE_H_
#define TTS_NODE_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <robocare_msgs/TTSSetProperties.h>
#include <robocare_msgs/TTSViseme.h>
#include <robocare_msgs/TTSMake.h>

#include "Exception.h"
#include <TTS/PowerTTS_M.h>
#include <boost/thread/mutex.hpp>

using namespace std;

namespace tts {

  class TTSInfo {
  public:
    int language;
  	int speaker;
  	int volume;
  	int speed;
  	int pitch;
  };


  typedef struct {
  	char   szRiff[4];
  	long   lFileSize;
  	char   szWave[8];
  	long   lUnknown;
  	short  nFormat;
  	short  nChannels;
  	long   lSamplesPerSec;
  	long   lAvgBytesPerSec;
  	short  lBlockAlign;
  	short  nBits;
  	char   szID3[4];
  	long   lDataSize;
  } WAVEFILEHEADER;



  #define WAVE_FORMAT_UNKNOWN      0X0000;
  #define WAVE_FORMAT_PCM          0X0001;
  #define WAVE_FORMAT_MS_ADPCM     0X0002;
  #define WAVE_FORMAT_IEEE_FLOAT   0X0003;
  #define WAVE_FORMAT_ALAW         0X0006;
  #define WAVE_FORMAT_MULAW        0X0007;
  #define WAVE_FORMAT_IMA_ADPCM    0X0011;
  #define WAVE_FORMAT_YAMAHA_ADPCM 0X0016;
  #define WAVE_FORMAT_GSM          0X0031;
  #define WAVE_FORMAT_ITU_ADPCM    0X0040;
  #define WAVE_FORMAT_MPEG         0X0050;
  #define WAVE_FORMAT_EXTENSIBLE   0XFFFE;

  typedef struct
  {
  	unsigned char ChunkID[4];    // Contains the letters "RIFF" in ASCII form
  	unsigned int ChunkSize;      // This is the size of the rest of the chunk following this number
  	unsigned char Format[4];     // Contains the letters "WAVE" in ASCII form
  } RIFF;

  //-------------------------------------------
  // [Channel]
  // - streo     : [left][right]
  // - 3 channel : [left][right][center]
  // - quad      : [front left][front right][rear left][reat right]
  // - 4 channel : [left][center][right][surround]
  // - 6 channel : [left center][left][center][right center][right][surround]
  //-------------------------------------------
  typedef struct
  {
  	unsigned char  ChunkID[4];    // Contains the letters "fmt " in ASCII form
  	unsigned int   ChunkSize;     // 16 for PCM.  This is the size of the rest of the Subchunk which follows this number.
  	unsigned short AudioFormat;   // PCM = 1
  	unsigned short NumChannels;   // Mono = 1, Stereo = 2, etc.
  	unsigned int   SampleRate;    // 8000, 44100, etc.
  	unsigned int   AvgByteRate;   // SampleRate * NumChannels * BitsPerSample/8
  	unsigned short BlockAlign;    // NumChannels * BitsPerSample/8
  	unsigned short BitPerSample;  // 8 bits = 8, 16 bits = 16, etc
  } FMT;


  typedef struct
  {
  	char          ChunkID[4];    // Contains the letters "data" in ASCII form
  	unsigned int  ChunkSize;     // NumSamples * NumChannels * BitsPerSample/8
  } DATA;


  typedef struct
  {
  	RIFF Riff;
  	FMT	 Fmt;
  	DATA Data;
  } WAVE_HEADER;

  class TTSNode {
  private:
    static int m_TTSResTotal;

    FILE *fp;
    vector<VISEMES> m_svVisemes;
    boost::mutex ttsMutex;

    TTSInfo ttsInfo;
    string database_location;
    ros::NodeHandle nh;
    ros::ServiceServer makesrv;
    ros::Subscriber setPropertiesSub;

    void initTTS();
    void loadDB();
    void loadEngine(const char* name, int id, char* path);
    void initROS();

    void makeTTS(const string& sentence, const string& filePath)  throw (Exception);
    void initDropWav(const string& filepath ) throw (Exception);
    static int PCMCallBack(void *pInParam, SYNRES *pSynRes);
    int processPCMCallback(SYNRES* pSynRes, TTSNode* pTTS);
    bool makeWaveHeader(FILE *fp, int DURATION, int SAMPLE_RATE, int CHANNEL);

  public:
    TTSNode();
    virtual ~TTSNode();
    bool makeCB(robocare_msgs::TTSMake::Request& request, robocare_msgs::TTSMake::Response& response);
    void setProertiesCB(const robocare_msgs::TTSSetProperties::ConstPtr& properties);
  };
}

#endif // TTS_NODE_H_
