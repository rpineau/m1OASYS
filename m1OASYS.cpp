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
    bDebugLog = false;

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
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Cm1OASYS] Starting log for version 1.3");
        mLogger->out(mLogBuffer);
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] Trying to connect to %s.", szPort);
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
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] Connected.");
        mLogger->out(mLogBuffer);
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] Getting shutter state.");
        mLogger->out(mLogBuffer);
    }

    // get the current shutter state just to check the connection, we don't care about the state for now.
    err = getShutterState(mShutterState);
    if(err) {
        bIsConnected = false;
        return ERR_COMMNOLINK;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] m1OASYS init done.");
        mLogger->out(mLogBuffer);
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::Connect] bIsConnected = %u.", bIsConnected);
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
                snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::readResponse] readFile error.");
                mLogger->out(mLogBuffer);
            }
            return err;
        }
        if (nBytesRead !=1) {// timeout
            err = RoR_BAD_CMD_RESPONSE;
            if (bDebugLog) {
                snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::readResponse] readFile Timeout while getting response.");
                mLogger->out(mLogBuffer);
            }
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
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::domeCommand] Sending %s",cmd);
        mLogger->out(mLogBuffer);
    }
    err = pSerx->writeFile((void *)cmd, strlen(cmd), nBytesWrite);
    pSerx->flushTx();
    if(err)
        return err;
    err = readResponse(resp, SERIAL_BUFFER_SIZE);
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::domeCommand] response = %s", resp);
        mLogger->out(mLogBuffer);
    }

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
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::enableSensors] Sending sensor on.");
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
    int timeout = 0;
    char resp[SERIAL_BUFFER_SIZE];

    if(!bIsConnected)
        return NOT_CONNECTED;

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::getShutterState]");
        mLogger->out(mLogBuffer);
    }

    err = domeCommand("09xx00100B6\r\n", resp,  SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    // wait for a proper response
    while(!strstr(resp,"open") && !strstr(resp,"closed") && !strstr(resp,"unknown")) {
        if(timeout>50) {
            err = COMMAND_FAILED;
            break;
        }
        err = readResponse(resp, SERIAL_BUFFER_SIZE);
        if(err) {
            err = COMMAND_FAILED;
            break;
        }
        // usleep(100000);  // 100 ms
        mSleeper->sleep(100);
        timeout++;
    }

    if(strstr(resp,"open")) {
        state = OPEN;
        mShutterOpened = true;
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::getShutterState] Shutter is opened");
            mLogger->out(mLogBuffer);
        }
    } else if (strstr(resp,"close")) {
        state = CLOSED;
        mShutterOpened = false;
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::getShutterState] Shutter is closed");
            mLogger->out(mLogBuffer);
        }
    } else {
        state = UNKNOWN;
        mShutterOpened = false;
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::getShutterState] Shutter state is unknown");
            mLogger->out(mLogBuffer);
        }
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
    int timeout = 0;
    
    char resp[SERIAL_BUFFER_SIZE];

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::openShutter]");
        mLogger->out(mLogBuffer);
    }

    if(!bIsConnected) {
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::openShutter] NOT CONNECTED !!!!");
            mLogger->out(mLogBuffer);
        }
        return NOT_CONNECTED;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::openShutter] Sending sensor on.");
        mLogger->out(mLogBuffer);
    }
    err = enableSensors();
    if(err) {
        err = COMMAND_FAILED;
    }

    err = domeCommand("09tn00100C4\r\n", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    
    // check returned data to make sure the command was processed
    while(!strstr(resp,"ATC001000D7")) {
        
        //we're waiting for the answer
        if(timeout>50) {
            err = COMMAND_FAILED;
            break;
        }
        err = readResponse(resp, SERIAL_BUFFER_SIZE);
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::openShutter] response = %s", resp);
            mLogger->out(mLogBuffer);
        }
        // usleep(100000);
        mSleeper->sleep(100);
        timeout++;
    }
    return err;
}

int Cm1OASYS::closeShutter()
{
    int err = RoR_OK;
    int timeout = 0;
    char resp[SERIAL_BUFFER_SIZE];

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter]");
        mLogger->out(mLogBuffer);
    }

    if(!bIsConnected) {
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter] NOT CONNECTED !!!!");
            mLogger->out(mLogBuffer);
        }
        return NOT_CONNECTED;
    }

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter] Sending sensor on.");
        mLogger->out(mLogBuffer);
    }
    err = enableSensors();
    if(err) {
        err = COMMAND_FAILED;
    }

    err = domeCommand("09tn00200C3\r\n", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;

    // check returned data to make sure the command was processed
    while(!strstr(resp,"ATC002000D6")) {
        //we're waiting for the answer
        if(timeout>50) {
            err = COMMAND_FAILED;
            break;
        }
        err = readResponse(resp, SERIAL_BUFFER_SIZE);
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::closeShutter] response = %s", resp);
            mLogger->out(mLogBuffer);
        }
        //usleep(100000);
        mSleeper->sleep(100);
        timeout++;
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

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::isOpenComplete] Checking roof state");
        mLogger->out(mLogBuffer);
    }

    err = getShutterState(mShutterState);
    if(err)
        return ERR_CMDFAILED;

    if(mShutterState == OPEN){
        mShutterOpened = true;
        complete = true;
        mCurrentElPosition = 90.0;
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::isOpenComplete] Roof is opened");
            mLogger->out(mLogBuffer);
        }
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

    if(!bIsConnected)
        return NOT_CONNECTED;

    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::isCloseComplete] Checking roof state");
        mLogger->out(mLogBuffer);
    }

    err = getShutterState(mShutterState);
    if(err)
        return ERR_CMDFAILED;

    if(mShutterState == CLOSED){
        mShutterOpened = false;
        complete = true;
        mCurrentElPosition = 0.0;
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::isOpenComplete] Roof is closed");
            mLogger->out(mLogBuffer);
        }
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

    // 09tn00300C2

    int err = RoR_OK;
    char resp[SERIAL_BUFFER_SIZE];
    
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::abortCurrentCommand]");
        mLogger->out(mLogBuffer);
    }
    
    if(!bIsConnected) {
        if (bDebugLog) {
            snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::abortCurrentCommand] NOT CONNECTED !!!!");
            mLogger->out(mLogBuffer);
        }
        return NOT_CONNECTED;
    }
    
    if (bDebugLog) {
        snprintf(mLogBuffer,ND_LOG_BUFFER_SIZE,"[Cm1OASYS::abortCurrentCommand] Sending abort command.");
        mLogger->out(mLogBuffer);
    }

    err = domeCommand("09tn00300C2\r\n", resp, SERIAL_BUFFER_SIZE);
    if(err)
        return err;
    
    return err;
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

