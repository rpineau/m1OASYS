//
//  m1OASYS.h
//  Cm1OASYS
//
//  Created by Rodolphe Pineau on 2017-2-23
//  m1OASYS X2 plugin

#ifndef __m1_OASYS__
#define __m1_OASYS__
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

#ifdef SB_WIN_BUILD
#include <time.h>
#endif

#include <string>

#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"


#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 5000
#define ND_LOG_BUFFER_SIZE 256

#define M1_DEBUG 2
// error codes
enum m1OASYSErrors {RoR_OK=0, NOT_CONNECTED, RoR_CANT_CONNECT, RoR_BAD_CMD_RESPONSE, COMMAND_FAILED};

// Error code
enum m1OASYSShutterState {OPEN=1, OPENING, CLOSED, CLOSING, SHUTTER_ERROR, UNKNOWN};

class Cm1OASYS
{
public:
    Cm1OASYS();
    ~Cm1OASYS();

    int        Connect(const char *szPort);
    void        Disconnect(void);
    bool        IsConnected(void) { return bIsConnected; }

    void        SetSerxPointer(SerXInterface *p) { pSerx = p; }
    void        setSleeper(SleeperInterface *pSleeper) { mSleeper = pSleeper; };

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double newAz);
    int openShutter();
    int closeShutter();

    // command complete functions
    int isGoToComplete(bool &complete);
    int isOpenComplete(bool &complete);
    int isCloseComplete(bool &complete);
    int isParkComplete(bool &complete);
    int isUnparkComplete(bool &complete);
    int isFindHomeComplete(bool &complete);
    int isCalibratingComplete(bool &complete);

    int abortCurrentCommand();

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();

    void setDebugLog(bool enable);

protected:

    int             readResponse(char *respBuffer, unsigned int bufferLen);
    int             getDomeAz(double &domeAz);
    int             getDomeEl(double &domeEl);
    int             getShutterState(int &state);

    int             domeCommand(const char *cmd, char *result, int resultMaxLen);
    int             enableSensors(void);
    
    SleeperInterface    *mSleeper;
    
    bool            bDebugLog;

    bool            bIsConnected;
    bool            mShutterOpened;

    double          mCurrentAzPosition;
    double          mCurrentElPosition;

    SerXInterface   *pSerx;

    int             mShutterState;
    char            mLogBuffer[ND_LOG_BUFFER_SIZE];

#ifdef M1_DEBUG
    std::string m_sLogfilePath;
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;      // LogFile
#endif

};

#endif
