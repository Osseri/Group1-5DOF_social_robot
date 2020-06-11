/*
	PowerTTS_M Header

	Copyright (C) 2001-2013 HCILAB
*/

#define	MAX_SPEAKER		20	//10			// 최대 화자수

//#define _LOAD_SPEAKER_RESPECTIVELY
//<==

#define	SAMPLE_8K		8000
#define SAMPLE_16K		16000

#define KO_KR		0
#define EN_US		1
#define ZH_CN		2
#define JA_JP		3
#define ES_MX		4	// 2014.02.24. separk.
#define EN_GB		5	// 2014.10.22. separk.
#define PT_BR		6	// 2014.11.07. separk.
#define FR_CA		7	// 2014.12.15. separk.

#define MAX_LANGUAGE	FR_CA	// 2014.10.28. separk. // 2015.01.21. separk.

//==> 다중화자 지원. 2013.01.04. separk.
// KO_KR
#define PTTS_SPKID_YUJIN	0
#define PTTS_SPKID_SUJIN	4
#define PTTS_SPKID_MIJIN	6
#define PTTS_SPKID_MINJUN	1
#define PTTS_SPKID_GICHAN	3   // at 2015-09-11 added by hypark
#define PTTS_SPKID_ARAM	    2
//#define PTTS_SPKID_HEAJIN	10
#define PTTS_SPKID_HYEJIN	10	// fixed. 2016.02.01. separk.
#define PTTS_SPKID_KYUNGJIN	8
// EN_US
#define PTTS_SPKID_CHRIS	0   // at 2015-09-08 added by hypark
#define PTTS_SPKID_JUDY 	2   // at 2015-09-14 added by hypark
#define PTTS_SPKID_SARAH	4
#define PTTS_SPKID_RICHARD	1
// ZH_CN
#define PTTS_SPKID_XIAOLING	0
#define PTTS_SPKID_JIAOLING	1
// JA_JP
#define PTTS_SPKID_NAOMI	0
#define PTTS_SPKID_OTOHA	2
#define PTTS_SPKID_EITA	1
// ES_MX
#define PTTS_SPKID_VERONICA	0
// EN_GB
#define PTTS_SPKID_CLAIRE	0	// Claire. 2014.10.22. separk.
// PT_BR
#define PTTS_SPKID_MONICA	0	// Monica. 2014.11.07. separk.
// FR_CA
#define PTTS_SPKID_ESTELLE	0	// Estelle. 2014.12.15. separk.
//<==

#define	E_PTTS_FILEOPEN			-1	// FILE OPEN 에러
#define	E_PTTS_LACKOFMEMORY		-2	// Working Memory 부족
#define E_PTTS_TTSTHREADFAIL	-3	// TTS Thread Generation 실패
#define E_PTTS_DATEEXPIRED		-4	// 사용 가능일 만료
#define E_PTTS_ENGINELOADFAIL   -5  // 엔진 로드 실패
#define E_PTTS_INVALIDPARAM		-6	// invalid parameter 사용
#define E_PTTS_INVALIDSPEAKER	-7	// invalid speaker 사용

#define E_PTTS_INVALIDCHANNEL	-8	// 유효하지 않은 채널임
#define E_PTTS_NOMORECHANNEL	-9	// 허용채널초과
#define E_PTTS_NOENGINELOAD		-10	// 엔진이 로드되어 있지 않음
#define E_PTTS_OUTOFRANGE		-11	// Specified argument is out of range
#define E_PTTS_NOTSUPPORTED		-12	// 지원하지 않는 기능

#define E_PTTS_CHARSETMISMATCH	-21	// Character Set이 맞지 않는 경우
#define E_PTTS_VXMLTAGERROR		-22	// VXML TAG가 잘못된 경우

#define E_PTTS_FORMATMISMATCH	-31	// AudioMixing에 사용되는 두 개의 파일의 Format이 서로 다른 경우

#define	E_PTTS_INITIALIZE		-100	// TTS Engine Initialize가 안됨

#define E_PTTS_LICENSE_KEY_NOT_FOUND		-101
#define E_PTTS_LICENSE_DATE_EXPIRED			-102
#define E_PTTS_LICENSE_INVALID_SYSTEM		-103
#define E_PTTS_LICENSE_INVALID_KEY			-104
#define E_PTTS_LICENSE_INVALID_OEMKEY		-105

#define CODING_PCM				0	// TextToFile()의 Format
#define CODING_LINEAR8			1
#define CODING_MULAW			2
#define CODING_ALAW				3
#define CODING_VOX				4
#define CODING_ADPCM			5	// 4-bit CCITT g.721 ADPCM
#define CODING_WAVE				6

#define PTTS_FORMAT_WAV		    0	// TextToFileEx()의 Format
#define PTTS_FORMAT_AU			1
#define PTTS_FORMAT_PCM			2

#define PTTS_ENCODING_LINEAR16	0	// TextToFileEx()의 Encoding 
#define PTTS_ENCODING_LINEAR8	1	 
#define PTTS_ENCODING_MULAW		2
#define PTTS_ENCODING_ALAW		3
#define PTTS_ENCODING_VOX		4
#define PTTS_ENCODING_ADPCM		5	// 4-bit CCITT g.721 ADPCM

#define	PTTS_PLAY_START			1		// SOUND CARD API 에서
#define	PTTS_PLAY_PAUSE			2		// 합성기가 사용자에게 uUsrMsg 와 함께 wParam 에 주는 정보
#define PTTS_PLAY_RESTART		3
#define PTTS_PLAY_END			4
#define	PTTS_PLAY_ERROR			-1

#define	PTTS_PLAY_DONE		100		// for HTS. 2012.08.28. separk.

#define	E_PTTS_USRDERR_LOAD			11	// 이미 사전이 로드되어 있는 경우
#define	E_PTTS_USRDERR_FILEOPEN		12	// 사용자 사전이 Open 되지 않는 경우
#define	E_PTTS_USRDERR_MEMALLOC		13	// 메모리가 없는 경우
#define	E_PTTS_USRDERR_INVALIDPARA	14	// 함수의 parameter 값이 정상적이지 않은 경우
#define	E_PTTS_USRDERR_ENTRYNUM		15	// 허용 Entry수 = MAX_USRDICT을 넘는 경우
#define	E_PTTS_USRDERR_BADFORMAT	16	// '{원단어리스트} {새단어리스트}' 형식이 아닌 경우
#define	E_PTTS_USRDERR_BADORGWORD1	17	// 원단어리스트가 USRDICT_WORDLIST_LEN(128)의 길이를 넘는 경우
#define	E_PTTS_USRDERR_BADORGWORD2	18	// 원단어리스트내 단어가 USRDICT_WORD_LEN(40)의 길이를 넘는 경우
#define	E_PTTS_USRDERR_BADORGWORD3	19	// 원단어리스트내 다른 에러가 있는 경우
#define	E_PTTS_USRDERR_BADNEWWORD1	20	// 새단어리스트가 USRDICT_WORDLIST_LEN(128)의 길이를 넘는 경우
#define	E_PTTS_USRDERR_BADNEWWORD2	21	// 새단어리스트내 단어가 USRDICT_WORD_LEN(40)의 길이를 넘는 경우
#define	E_PTTS_USRDERR_BADNEWWORD3	22	// 새단어리스트내 다른 에러가 있는 경우

/**/
#define PHONE_TYPE_DEFAULT	0	// 립정보-폰단위 일때
#define PHONE_TYPE_CVC		1	// 립정보-음절단위 일때, 음절구조
#define PHONE_TYPE_CV		2
#define PHONE_TYPE_VC		3
#define PHONE_TYPE_V		4
#define PHONE_TYPE_PAUSE	5

/* TTS Mode */	// 2010.04.08. separk.
#define PTTS_BASIC_MODE			0	// default
#define PTTS_NAVIGATION_MODE	1

typedef struct tagPHONEID
{
    short    PhoneID;    // 폰
    int      nStPos;     // 시작 위치 : msec 단위
    int      nDur;       // 폰 길이   : msec 단위
    char     chStress;   // stress 여부 ; 0, 1
} PHONEID;

typedef struct tagVISEMES
{
    short   VisemeID;    // 폰
    int     nStPos;      // 시작 위치 : msec 단위
    int     nDur;        // 폰 길이   : msec 단위
	char	chStress;    // stress 여부 ; 0, 1
} VISEMES;

typedef struct tagVOICESTATUS
{
	unsigned long	ulWordPos;		  // Character offset of the beginning of the word being synthesized
	unsigned long	ulWordLen;		  // Character length of the word in the current input stream being synthesized
	unsigned long	ulSentPos;		  // Character offset of the beginning of the sentence being synthesized
	unsigned long	ulSentLen;		  // Character length of the sentence in the current input stream being synthesized
	long	        lBookmarkID;      // 미구현
	unsigned int	uIPhonemeNum;	  // Word내의 Phoneme의 개수
	PHONEID	        *PhonemeID;	      // Phoneme 정보
	VISEMES	        *VisemeID;	      // Viseme 정보 
	unsigned int	dwReserved1;      // Reserved for future expansion
	unsigned int	dwReserved2;      // Reserved for future expansion
} VOICESTATUS;

typedef struct tagHLIGHTINFO {
    int    nTextPos_Start;    // 텍스트 시작 위치
    int    nTextPos_End;      // 텍스트 끝 위치
    int    nAudioPos_Start;   // 사운드 출력 위치 (byte)
    int    nAudioPos_End;     // 사운드 끝 위치   (byte)
} HLIGHTINFO;

typedef struct tagMARKINFO {
	HLIGHTINFO	*pLight;			// Highlight 정보
    VOICESTATUS *pVoiceStatus;    // SAPI 지원을 위한 구조체(viseme 정보 포함)
    void        *pReserved;       // Reserved for future expansion
} MARKINFO;

typedef struct tagSYNRES {			// 합성기에서 만들어내는 결과 [사용자정의 callback에서 사용]
	unsigned int   NumBytes;		// 합성음의 크기
	void        *pData;				// 합성음
	void        *pReserved;			//
	short		Status;				// Buffer의 상태를 나타냄 (0:Pcm Buffer 생성중, 1:사용자가 선택한 TTS_READ_MODE의 끝, 2:전체 텍스트 합성 종료, 3:STOP 명령에 의한 종료)
	int			nEvent;				// MarkInfo 타입 정의
} SYNRES;

typedef struct {
	char	szProduct[40];
	char	szVersion[40];
	char	szLanguage[20];
	char	chCodingScheme;
	char	szAPIVersion[40];
} PowerTTSENGINEINFO;

typedef struct {
	int		nSpeakerID;
	char	szSpeaker[20];
	char	szLanguage[20];
	int		nSamplingRate;
} PowerTTSDBINFO;

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* Basic API */
int PTTS_SetOemKey(const char *pKey);	// 0: Fail, 1: Success

int	 PTTS_Initialize();
void PTTS_UnInitialize();

#ifdef _LOAD_SPEAKER_RESPECTIVELY
int  PTTS_LoadEngine(int Language, char *DbDir, int bLoadInMemory, int iSpeaker);	// 다중화자 지원. 2013.01.04. separk.
#else
int  PTTS_LoadEngine(int Language, char *DbDir, int bLoadInMemory);
#endif
void PTTS_UnLoadEngine(int Language);

void *PTTS_CreateThread(void *pInParam, int (*CallBack)(void *pInParam, SYNRES *pResult), int Language, int Speaker);
void PTTS_DeleteThread(void *pTTSThread);
int  PTTS_TextToSpeech(void *pTTSThread, char *Text, int bTaggedText);
int  PTTS_TextToPcmBuf(void *pTTSThread, char *Text, int bTaggedText, int OutBufSize);
int  PTTS_StopTextToSpeech(void *pTTSThread);
int  PTTS_ChangeAttr(void *pTTSThread, char *tagString);

/* Extension Functions */
int PTTS_GetPitchDefault(int Language, int speaker, int *Min, int *Max);
int PTTS_GetVolumeDefault(int Language, int *Min, int *Max);
int PTTS_GetSpeedDefault(int Language, int *Min, int *Max);
int PTTS_ChangeSpeaker(void *pTTSThread, int Speaker);
void PTTS_GetEngineInfo(int Language, PowerTTSENGINEINFO *st_TTSEngineInfo);
int PTTS_SetSpeed(void *pTTSThread, int Speed);
int PTTS_SetVolume(void *pTTSThread, int Volume);
int	PTTS_SetPitch(void *pTTSThread, int Pitch);

/* Extension Functions 2 for New Server*/
void PTTS_GetChannelInfo(int Language, int *UsingCh, int *MaxCh);
void PTTS_GetDBInfo(int Language, int SpeakerID, PowerTTSDBINFO *st_TTSDBInfo);
int PTTS_GetError();

/*===========================================================================*/
/* File API */
int PTTS_TextToFile(int Language, int Speaker, int Format, int Encoding, int SamplingRate,
					  char *TextBuf, char *OutFileName, char *tagString, char *UserDictFileName);

/*===========================================================================*/
/* 추가 설정 함수 */
int PTTS_SetTextFormat(void *pTTSThread, char chTextFormat);	// 0 : naormal text(default),	1 : VXML
int PTTS_SetCharSet(void *pTTSThread, char chCharSet);			// 0 : KSC-5601+확장완성형 (default), 1: UTF-8
int PTTS_SetLipSync(void *pTTSThread, char chMode);			// 0 : Off (default), 1: On
int PTTS_SetHighLight(void *pTTSThread, char chMode);			// 0 : Off (default), 1: On
int PTTS_SetByteSwap(void *pTTSThread, char chMode);			// 0 : Off (default), 1: On
int	PTTS_SetReadGwalho(void *pTTSThread, int bGwalho);	// 2008.09.08. separk.
int	PTTS_SetReadParenthesis(void *pTTSThread, int bParenthesis);	// 2016.01.04. sbseo add
int  PTTS_SetPinyinMode(void *pTTSThread, int bPinyinMode);	// 2008.09.18. separk.
int  PTTS_SetTTSMode(int bTTSMode);	// 2010.04.08. separk.
//==> 2016.01.19. separk.
int PTTS_SetCallbackMode(void *pTTSThread, int nMode);
int PTTS_GetCallbackMode(void *pTTSThread);
/* TTS Callback Mode */
#define PTTS_CALLBACK_BY_SENT    1
#define PTTS_CALLBACK_BY_WORD    2
//<==


/*===========================================================================*/
/* SOUND CARD API */
#if defined(WIN32)
int PTTS_PlayTTS(HWND hUsrWnd, UINT uUsrMsg, char *TextBuf, char *tagString, int Language, int Speaker);
int PTTS_PlayTTSPron(HWND hUsrWnd, UINT uUsrMsg, char *TextBuf, wchar_t *Pron, char *tagString, int Language, int Speaker);
int PTTS_StopTTS(void);
int PTTS_PauseTTS(void);
int PTTS_RestartTTS(void);
int PTTS_GetFirstUtteranceTime(void);
#else
//int PTTS_PlayTTS(char *TextBuf, char *tagString, int Language, int Speaker);
int PTTS_PlayTTS(char *TextBuf, char *tagString, int Language, int Speaker, int (*CallBack)(HLIGHTINFO *pInfo));	// 2009.05.22. separk.
int PTTS_StopTTS(void);
int PTTS_PauseTTS(void);
int PTTS_RestartTTS(void);
#endif

//==> 2013.01.03. separk.
int  PTTS_GetPlaybackStatus(void);

#define PTTS_PLAYBACK_STOP	0
#define PTTS_PLAYBACK_PLAY	1
#define PTTS_PLAYBACK_PAUSE	2
//<==

int PTTS_LoadUserDict(void *pTTSThread, char *UsrDictFileName, int *nErrorType, int *nErrorLine);
void PTTS_UnLoadUserDict(void *pTTSThread);
int PTTS_ChangeUserDict(void *pTTSThread, char *UsrDictFileName, int *nErrorType, int *nErrorLine);

void PTTS_RegisterUserDict(char *UsrDictFileName);
void PTTS_DeleteUserDict();

//==> These functions are deprecated. 2014.07.08. separk.
#if defined(WIN32)
char *PTTS_UNICODEtoGB2312(unsigned char* inChar);
char *PTTS_UNICODEtoSHIFTJIS(unsigned char* inChar);
char *PTTS_UNICODEtoKSC5601(unsigned char* inChar);
#else
//==> renewal for iPod. 2009.09.18. separk.
char *PTTS_UNICODEtoGB2312(unsigned char* inChar, int nLen);
char *PTTS_UNICODEtoSHIFTJIS(unsigned char* inChar, int nLen);
char *PTTS_UNICODEtoKSC5601(unsigned char* inChar, int nLen);
//<==
#endif
//<==
//==> instead... 2014.07.08. separk.
int PTTS_UNICODEtoASIANCODE(unsigned char* inChar, char* outChar, int iLang, int strLen);
int PTTS_UNICODEtoUTF8(unsigned char* inChar, char* outChar, int strLen);
//<==

#ifdef __cplusplus
};
#endif
