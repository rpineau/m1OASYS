//
//  m1OASYS.cpp
//  Cm1OASYS
//
//  Created by Rodolphe Pineau on 2017-2-23
//  m1OASYS X2 plugin

#include "m1OASYS.h"
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif

Cm1OASYS::Cm1OASYS()
{
    // set some sane values
    bDebugLog = true;

    pSerx = NULL;
    bIsConnected = false;


    mCurrentAzPosition = 0.0;
    mCurrentElPosition = 0.0;


    mShutterOpened = false;

    memset(mLogBuffer,0,ND_LOG_BUFFER_SIZE);
}

Cm1OASYS::~Cm1OASYS()
{

}

int Cm1OASYS::Connect(const char *szPort)
{
    int err;
    int state;

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] Trying to connect to %s.\n", szPort);
        mLogger->out(mLogBuffer);
    }

    // 9600 8N1
    if(pSerx->open(szPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        bIsConnected = true;
    else
        bIsConnected = false;

    if(!bIsConnected)
        return ERR_COMMNOLINK;

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] Connected.\n");
        mLogger->out(mLogBuffer);

        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] Getting Firmware.\n");
        mLogger->out(mLogBuffer);
    }
    // if this fails we're not properly connected.
    err = getShutterState(state);
    if(err) {
        bIsConnected = false;
        return ERR_COMMNOLINK;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] Got RoR state.\n");
        mLogger->out(mLogBuffer);
    }

    syncDome(mCurrentAzPosition,mCurrentElPosition);

    return SB_OK;
}


void Cm1OASYS::Disconnect()
{
    if(bIsConnected) {
        pSerx->purgeTxRx();
        pSerx->close();
    }
    bIsConnected = false;
}


int Cm1OASYS::readResponse(char *respBuffer, int bufferLen)
{
    int err = RoR_OK;
    unsigned long nBytesRead = 0;
    unsigned long totalBytesRead = 0;
    char *bufPtr;

    memset(respBuffer, 0, (size_t) bufferLen);
    bufPtr = respBuffer;

    do {
        err = pSerx->readFile(bufPtr, 1, nBytesRead, MAX_TIMEOUT);
        if(err) {
            if (bDebugLog) {
                snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::readResponse] readFile error.\n");
                mLogger->out(mLogBuffer);
            }
            return err;
        }

        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::readResponse] respBuffer = %s\n",respBuffer);
            mLogger->out(mLogBuffer);
        }

        if (nBytesRead !=1) {// timeout
            if (bDebugLog) {
                snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::readResponse] readFile Timeout.\n");
                mLogger->out(mLogBuffer);
            }
            err = RoR_BAD_CMD_RESPONSE;
            break;
        }
        totalBytesRead += nBytesRead;
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::readResponse] nBytesRead = %lu\n",nBytesRead);
            mLogger->out(mLogBuffer);
        }
    } while (*bufPtr++ != 0x0D && totalBytesRead < bufferLen );

    *bufPtr = 0; //remove the \r
    return err;
}


int Cm1OASYS::domeCommand(const char *cmd, char *result, int resultMaxLen)
{
    int err = RoR_OK;
    char resp[SERIAL_BUFFER_SIZE];
    unsigned long  nBytesWrite;

    pSerx->purgeTxRx();
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::domeCommand] Sending %s\n",cmd);
        mLogger->out(mLogBuffer);
    }
    err = pSerx->writeFile((void *)cmd, strlen(cmd), nBytesWrite);
    pSerx->flushTx();
    if(err)
        return err;
    // read response
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[CRigelDome::domeCommand] Getting response.\n");
        mLogger->out(mLogBuffer);
    }
    err = readResponse(resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(result)
        strncpy(result, &resp[1], resultMaxLen);

    return err;

}


int Cm1OASYS::getDomeAz(double &domeAz)
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;


    // convert Az string to double
    domeAz = mCurrentAzPosition;
    return err;
}

int Cm1OASYS::getDomeEl(double &domeEl)
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;

    if(!mShutterOpened)
    {
        domeEl = 0.0;
        return err;
    }

    // convert El string to double
    domeEl = mCurrentElPosition;

    return err;
}


int Cm1OASYS::getShutterState(int &state)
{
    int err = RoR_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = domeCommand("09xx00100B6\r\n", resp,  SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strstr(resp,"open")) {
        state = OPEN;
    } else if (strstr(resp,"close")) {
        state = CLOSED;
    }
    return err;
}


void Cm1OASYS::setDebugLog(bool enable)
{
    bDebugLog = enable;
}


int Cm1OASYS::syncDome(double dAz, double dEl)
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;

    mCurrentAzPosition = dAz;
    return err;
}

int Cm1OASYS::parkDome()
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;

    return err;

}

int Cm1OASYS::unparkDome()
{
    syncDome(mCurrentAzPosition,mCurrentElPosition);
    return 0;
}

int Cm1OASYS::gotoAzimuth(double newAz)
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;

    mCurrentAzPosition = newAz;

    return err;
}

int Cm1OASYS::openShutter()
{
    if(!bIsConnected)
        return NOT_CONNECTED;

    return (domeCommand("09tn00100C4\r\n", NULL, SERIAL_BUFFER_SIZE));
}

int Cm1OASYS::closeShutter()
{
    if(!bIsConnected)
        return NOT_CONNECTED;


    return (domeCommand("09tn00200C3\r\n", NULL, SERIAL_BUFFER_SIZE));
}


int Cm1OASYS::isGoToComplete(bool &complete)
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;
    complete = true;

    return err;
}

int Cm1OASYS::isOpenComplete(bool &complete)
{
    int err = RoR_OK;
    int state;

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(state);
    if(err)
        return ERR_CMDFAILED;
    if(state == OPEN){
        mShutterOpened = true;
        complete = true;
        mCurrentElPosition = 90.0;
    }
    else {
        mShutterOpened = false;
        complete = false;
        mCurrentElPosition = 0.0;
    }

    return err;
}

int Cm1OASYS::isCloseComplete(bool &complete)
{
    int err = RoR_OK;
    int state;

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(state);
    if(err)
        return ERR_CMDFAILED;
    if(state == CLOSED){
        mShutterOpened = false;
        complete = true;
        mCurrentElPosition = 0.0;
    }
    else {
        mShutterOpened = true;
        complete = false;
        mCurrentElPosition = 90.0;
    }

    return err;
}


int Cm1OASYS::isParkComplete(bool &complete)
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;

    complete = true;
    return err;
}

int Cm1OASYS::isUnparkComplete(bool &complete)
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;

    complete = true;

    return err;
}

int Cm1OASYS::isFindHomeComplete(bool &complete)
{
    int err = RoR_OK;

    if(!bIsConnected)
        return NOT_CONNECTED;
    complete = true;

    return err;

}

int Cm1OASYS::abortCurrentCommand()
{
    if(!bIsConnected)
        return NOT_CONNECTED;

    return RoR_OK;
}

#pragma mark - Getter / Setter


double Cm1OASYS::getCurrentAz()
{
    if(bIsConnected)
        getDomeAz(mCurrentAzPosition);

    return mCurrentAzPosition;
}

double Cm1OASYS::getCurrentEl()
{
    if(bIsConnected)
        getDomeEl(mCurrentElPosition);

    return mCurrentElPosition;
}

int Cm1OASYS::getCurrentShutterState()
{
    if(bIsConnected)
        getShutterState(mShutterState);

    return mShutterState;
}

