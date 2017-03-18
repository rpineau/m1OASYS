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
    mShutterState = UNKNOWN;

    memset(mLogBuffer,0,ND_LOG_BUFFER_SIZE);
}

Cm1OASYS::~Cm1OASYS()
{

}

int Cm1OASYS::Connect(const char *szPort)
{
    int err;
    
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

        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] Getting shutter state.\n");
        mLogger->out(mLogBuffer);
    }

    // get the current shutter state just to check the connection, we don't care about the state for now.
    err = getShutterState(mShutterState);
    if(err) {
        bIsConnected = false;
        return ERR_COMMNOLINK;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] m1OASYS init done.\n");
        mLogger->out(mLogBuffer);
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] bIsConnected = %u.\n", bIsConnected);
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


int Cm1OASYS::readResponse(char *respBuffer, unsigned int bufferLen)
{
    int err = RoR_OK;
    unsigned long nBytesRead = 0;
    unsigned int totalBytesRead = 0;
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

        if (nBytesRead !=1) {// timeout
            err = RoR_BAD_CMD_RESPONSE;
            break;
        }
        totalBytesRead += nBytesRead;
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
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::domeCommand] Sending %s\n",cmd);
        mLogger->out(mLogBuffer);
    }
    err = pSerx->writeFile((void *)cmd, strlen(cmd), nBytesWrite);
    pSerx->flushTx();
    if(err)
        return err;
    // read response
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::domeCommand] Getting response.\n");
        mLogger->out(mLogBuffer);
    }
    err = readResponse(resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(result)
        strncpy(result, &resp[1], resultMaxLen);

    return err;

}

int Cm1OASYS::enableSensors()
{
    //11xx005sensoron0042
    int err = RoR_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::enableSensors]\n");
        mLogger->out(mLogBuffer);
    }

    err = domeCommand("11xx005sensoron0042\r\n", resp,  SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    // check for Secure
    if(!strstr(resp,"ecure")) {
        err = COMMAND_FAILED;
    }

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

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::getShutterState]\n");
        mLogger->out(mLogBuffer);
    }

    err = domeCommand("09xx00100B6\r\n", resp,  SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    if(strstr(resp,"open")) {
        state = OPEN;
    } else if (strstr(resp,"close")) {
        state = CLOSED;
    }
    else {
        state = UNKNOWN;
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
    int err = RoR_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::openShutter]\n");
        mLogger->out(mLogBuffer);
    }

    if(!bIsConnected) {
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::openShutter] NOT CONNECTED !!!!\n");
            mLogger->out(mLogBuffer);
        }
        return NOT_CONNECTED;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::openShutter] Sending sensor on.\n");
        mLogger->out(mLogBuffer);
    }
    err = enableSensors();
    if(err) {
        err = COMMAND_FAILED;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::openShutter] Opening RoR.\n");
        mLogger->out(mLogBuffer);
    }

    err = domeCommand("09tn00100C4\r\n", resp, SERIAL_BUFFER_SIZE);
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::getShutterState] response res = %s\n", resp);
        mLogger->out(mLogBuffer);
    }

    if(err)
        return err;

    // check returned data to make sure the command was processed
    if(!strstr(resp,"ATC001000D7")) { // 0ACC010100E7 ?
        err = COMMAND_FAILED;
    }

    return err;
}

int Cm1OASYS::closeShutter()
{
    int err = RoR_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter]\n");
        mLogger->out(mLogBuffer);
    }

    if(!bIsConnected) {
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter] NOT CONNECTED !!!!\n");
            mLogger->out(mLogBuffer);
        }
        return NOT_CONNECTED;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter] Sending sensor on.\n");
        mLogger->out(mLogBuffer);
    }
    err = enableSensors();
    if(err) {
        err = COMMAND_FAILED;
    }


    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter] Closing RoR.\n");
        mLogger->out(mLogBuffer);
    }

    err = domeCommand("09tn00200C3\r\n", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    // check returned data to make sure the command was processed
    if(!strstr(resp,"ATC002000D6")) { //
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter] ERROR Closing RoR. res = %s\n", resp);
            mLogger->out(mLogBuffer);
        }
        err = COMMAND_FAILED;
    }

    return err;
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

    if(!bIsConnected)
        return NOT_CONNECTED;

    err = getShutterState(mShutterState);
    if(err)
        return ERR_CMDFAILED;
    if(mShutterState == OPEN){
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

