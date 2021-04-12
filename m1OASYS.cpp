//
//  m1OASYS.cpp
//  Cm1OASYS
//
//  Created by Rodolphe Pineau on 2017-2-23
//  m1OASYS X2 plugin

#include "m1OASYS.h"

Cm1OASYS::Cm1OASYS()
{
    // set some sane values
    m_pSerx = NULL;
    m_bIsConnected = false;


    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;


    m_bShutterOpened = false;
    m_nShutterState = UNKNOWN;

#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\m1OASYSLog.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/m1OASYSLog.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/m1OASYSLog.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::Cm1OASYS] New Constructor Called\n", timestamp);
    fflush(Logfile);
#endif


}

Cm1OASYS::~Cm1OASYS()
{
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::~Cm1OASYS] Destructor Called\n", timestamp );
    fflush(Logfile);
#endif

}

int Cm1OASYS::Connect(const char *szPort)
{
    int nErr = SB_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::Connect] Called with port %s\n", timestamp, szPort);
    fflush(Logfile);
#endif



    // 9600 8N1
    if(m_pSerx->open(szPort, 9600, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1") == 0)
        m_bIsConnected = true;
    else
        m_bIsConnected = false;

    if(!m_bIsConnected)
        return ERR_COMMNOLINK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::Connect] Getting roof state.\n", timestamp);
    fflush(Logfile);
#endif


    // get the current shutter state just to check the connection, we don't care about the state for now.
    nErr = getShutterState(m_nShutterState);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [Cm1OASYS::Connect] Error getting roof state = %d.\n", timestamp, nErr);
        fflush(Logfile);
#endif
        m_bIsConnected = false;
        return ERR_COMMNOLINK;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::Connect] Roof state = %d.\n", timestamp, m_nShutterState);
    fflush(Logfile);
#endif

    syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);

    return SB_OK;
}


void Cm1OASYS::Disconnect()
{
    if(m_bIsConnected) {
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
}

int Cm1OASYS::domeCommand(const char *cmd, char *result, int resultMaxLen)
{
    int nErr = PLUGIN_OK;
    char resp[SERIAL_BUFFER_SIZE];
    unsigned long  nBytesWrite;
    
    m_pSerx->purgeTxRx();
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::domeCommand] Sending %s\n", timestamp, cmd);
    fflush(Logfile);
#endif
    
    nErr = m_pSerx->writeFile((void *)cmd, strlen(cmd), nBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;
    nErr = readResponse(resp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::domeCommand] response  = %s\n", timestamp, resp);
    fflush(Logfile);
#endif
    
    if(result)
        strncpy(result, &resp[2], resultMaxLen);
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::domeCommand] response copied = %s\n", timestamp, result);
    fflush(Logfile);
#endif
    
    return nErr;
    
}


int Cm1OASYS::readResponse(char *szRespBuffer, unsigned int nBufferLen, int nTimeout)
{
    int nErr = PLUGIN_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;
    int nBytesWaiting = 0 ;
    int nbTimeouts = 0;
    
    memset(szRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = szRespBuffer;
    
    do {
        nErr = m_pSerx->bytesWaitingRx(nBytesWaiting);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 3
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CRTIDome::readResponse] nBytesWaiting = %d\n", timestamp, nBytesWaiting);
        fprintf(Logfile, "[%s] [CRTIDome::readResponse] nBytesWaiting nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        if(!nBytesWaiting) {
            if(nbTimeouts++ >= NB_RX_WAIT) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CRTIDome::readResponse] bytesWaitingRx timeout, no data for %d loops\n", timestamp, NB_RX_WAIT);
                fflush(Logfile);
#endif
                nErr = ERR_RXTIMEOUT;
                break;
            }
            m_pSleeper->sleep(MAX_READ_WAIT_TIMEOUT);
            continue;
        }
        nbTimeouts = 0;
        if(ulTotalBytesRead + nBytesWaiting <= nBufferLen)
            nErr = m_pSerx->readFile(pszBufPtr, nBytesWaiting, ulBytesRead, nTimeout);
        else {
            nErr = ERR_RXTIMEOUT;
            break; // buffer is full.. there is a problem !!
        }
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRTIDome::readResponse] readFile error.\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }
        
        if (ulBytesRead != nBytesWaiting) { // timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CRTIDome::readResponse] readFile Timeout Error\n", timestamp);
            fprintf(Logfile, "[%s] [CRTIDome::readResponse] readFile nBytesWaiting = %d\n", timestamp, nBytesWaiting);
            fprintf(Logfile, "[%s] [CRTIDome::readResponse] readFile ulBytesRead = %lu\n", timestamp, ulBytesRead);
            fflush(Logfile);
#endif
        }
        
        ulTotalBytesRead += ulBytesRead;
        pszBufPtr+=ulBytesRead;
    } while (ulTotalBytesRead < nBufferLen  && *(pszBufPtr-1) != 0x0D);
    
    if(!ulTotalBytesRead)
        nErr = COMMAND_TIMEOUT; // we didn't get an answer.. so timeout
    else
        *(pszBufPtr-1) = 0; //remove the \r
    
    return nErr;
}


int Cm1OASYS::enableSensors()
{
    //11xx005sensoron0042
    int nErr = PLUGIN_OK;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::enableSensors] Sending sensor on.\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("11xx005sensoron0042\r\n", resp,  SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [Cm1OASYS::enableSensors] Error enabling sensors = %d.\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::enableSensors] Enabling sensors resp = %s.\n", timestamp, resp);
    fflush(Logfile);
#endif

    m_pSleeper->sleep(3000);    // wait 3 seconds .. enabling sensors takes a long time

	nErr = domeCommand("09xx00200B5\r\n", resp,  SERIAL_BUFFER_SIZE);
	if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::enableSensors] Error enabling sensors = %d.\n", timestamp, nErr);
		fflush(Logfile);
#endif
		return nErr;
	}

	if(!strstr(resp,"NotSecure") && strstr(resp,"Secure") )
		nErr = PLUGIN_OK;
	else
		nErr = ERR_CMDFAILED;
	
    return nErr;
}

int Cm1OASYS::getDomeAz(double &domeAz)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    // convert Az string to double
    domeAz = m_dCurrentAzPosition;
    return nErr;
}

int Cm1OASYS::getDomeEl(double &domeEl)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterOpened)
    {
        domeEl = 0.0;
        return nErr;
    }

    // convert El string to double
    domeEl = m_dCurrentElPosition;

    return nErr;
}


int Cm1OASYS::getShutterState(int &state)
{
    int nErr = PLUGIN_OK;
    int timeout = 0;
    char resp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::getShutterState]\n", timestamp);
    fflush(Logfile);
#endif


    nErr = domeCommand("09xx00100B6\r\n", resp,  SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    // wait for a proper response
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [Cm1OASYS::getShutterState] Waiting for response.\n", timestamp);
    fflush(Logfile);
#endif
    
    while(!strstr(resp,"open") && !strstr(resp,"closed") && !strstr(resp,"unknown")) {
        if(timeout>50) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
			ltime = time(NULL);
			timestamp = asctime(localtime(&ltime));
			timestamp[strlen(timestamp) - 1] = 0;
			fprintf(Logfile, "[%s] [Cm1OASYS::getShutterState] Timeout !\n", timestamp);
			fflush(Logfile);
#endif
            nErr = COMMAND_FAILED;
            break;
        }
        nErr = readResponse(resp, SERIAL_BUFFER_SIZE);
        m_pSleeper->sleep(100);
        timeout++;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [Cm1OASYS::getShutterState] timeout = %d !\n", timestamp, timeout);
        fflush(Logfile);
#endif
    }

    if(strstr(resp,"open")) {
        state = OPEN;
        m_bShutterOpened = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::getShutterState] Roof is opened\n", timestamp);
		fflush(Logfile);
#endif
    } else if (strstr(resp,"close")) {
        state = CLOSED;
        m_bShutterOpened = false;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::getShutterState] Roof is closed\n", timestamp);
		fflush(Logfile);
#endif
    } else {
        state = UNKNOWN;
        m_bShutterOpened = false;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::getShutterState] Roof state is unknown\n", timestamp);
		fflush(Logfile);
#endif
    }

    return nErr;
}


int Cm1OASYS::syncDome(double dAz, double dEl)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_dCurrentAzPosition = dAz;
    return nErr;
}

int Cm1OASYS::parkDome()
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    return nErr;

}

int Cm1OASYS::unparkDome()
{
    syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
    return 0;
}

int Cm1OASYS::gotoAzimuth(double newAz)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_dCurrentAzPosition = newAz;

    return nErr;
}

int Cm1OASYS::openShutter()
{
    int nErr = PLUGIN_OK;
    int timeout = 0;
    
    char resp[SERIAL_BUFFER_SIZE];

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [Cm1OASYS::openShutter]\n", timestamp);
	fflush(Logfile);
#endif

	if(!m_bIsConnected) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::openShutter] Not Connected !\n", timestamp);
		fflush(Logfile);
#endif
    return NOT_CONNECTED;
    }

    nErr = enableSensors();
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [Cm1OASYS::openShutter] Failled to enable sensor, not opening!\n", timestamp);
        fflush(Logfile);
#endif
        return COMMAND_FAILED;
    }

    nErr = domeCommand("09tn00100C4\r\n", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
    // check returned data to make sure the command was processed
    while(!strstr(resp,"TC001000")) {
        
        //we're waiting for the answer
        if(timeout>50) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
			ltime = time(NULL);
			timestamp = asctime(localtime(&ltime));
			timestamp[strlen(timestamp) - 1] = 0;
			fprintf(Logfile, "[%s] [Cm1OASYS::openShutter] Timeout !\n", timestamp);
			fflush(Logfile);
#endif
            nErr = COMMAND_FAILED;
            break;
        }
        nErr = readResponse(resp, SERIAL_BUFFER_SIZE);
        m_pSleeper->sleep(100);
        timeout++;
    }
    return nErr;
}

int Cm1OASYS::closeShutter()
{
    int nErr = PLUGIN_OK;
    int timeout = 0;
    char resp[SERIAL_BUFFER_SIZE];

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [Cm1OASYS::closeShutter]\n", timestamp);
	fflush(Logfile);
#endif

    if(!m_bIsConnected) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::closeShutter] Not Connected !\n", timestamp);
		fflush(Logfile);
#endif
        return NOT_CONNECTED;
    }

    nErr = enableSensors();
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [Cm1OASYS::closeShutter] Failled to enable sensor, not closing!\n", timestamp);
        fflush(Logfile);
#endif
        return COMMAND_FAILED;
    }

    nErr = domeCommand("09tn00200C3\r\n", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;

    // check returned data to make sure the command was processed
    while(!strstr(resp,"TC002000")) {
        //we're waiting for the answer
        if(timeout>50) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
			ltime = time(NULL);
			timestamp = asctime(localtime(&ltime));
			timestamp[strlen(timestamp) - 1] = 0;
			fprintf(Logfile, "[%s] [Cm1OASYS::closeShutter] Timeout !\n", timestamp);
			fflush(Logfile);
#endif
            nErr = COMMAND_FAILED;
            break;
        }
        nErr = readResponse(resp, SERIAL_BUFFER_SIZE);
        m_pSleeper->sleep(100);
        timeout++;
    }

    return nErr;
}


int Cm1OASYS::isGoToComplete(bool &complete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    complete = true;

    return nErr;
}

int Cm1OASYS::isOpenComplete(bool &complete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [Cm1OASYS::isOpenComplete] Checking roof state\n", timestamp);
	fflush(Logfile);
#endif

    nErr = getShutterState(m_nShutterState);
    if(nErr)
        return ERR_CMDFAILED;

    if(m_nShutterState == OPEN){
        m_bShutterOpened = true;
        complete = true;
        m_dCurrentElPosition = 90.0;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::isOpenComplete] Roof is opened\n", timestamp);
		fflush(Logfile);
#endif
    }
    else {
        m_bShutterOpened = false;
        complete = false;
        m_dCurrentElPosition = 0.0;
    }

    return nErr;
}

int Cm1OASYS::isCloseComplete(bool &complete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [Cm1OASYS::isCloseComplete] Checking roof state\n", timestamp);
	fflush(Logfile);
#endif
    nErr = getShutterState(m_nShutterState);
    if(nErr)
        return ERR_CMDFAILED;

    if(m_nShutterState == CLOSED){
        m_bShutterOpened = false;
        complete = true;
        m_dCurrentElPosition = 0.0;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::isCloseComplete] Roof is closed\n", timestamp);
		fflush(Logfile);
#endif
    }
    else {
        m_bShutterOpened = true;
        complete = false;
        m_dCurrentElPosition = 90.0;
    }

    return nErr;
}


int Cm1OASYS::isParkComplete(bool &complete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    complete = true;
    return nErr;
}

int Cm1OASYS::isUnparkComplete(bool &complete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    complete = true;

    return nErr;
}

int Cm1OASYS::isFindHomeComplete(bool &complete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    complete = true;

    return nErr;

}

int Cm1OASYS::abortCurrentCommand()
{

    // 09tn00300C2

    int nErr = PLUGIN_OK;
    char resp[SERIAL_BUFFER_SIZE];
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [Cm1OASYS::abortCurrentCommand]\n", timestamp);
	fflush(Logfile);
#endif
    if(!m_bIsConnected) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [Cm1OASYS::abortCurrentCommand] NOT CONNECTED.\n", timestamp);
		fflush(Logfile);
#endif
        return NOT_CONNECTED;
    }
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [Cm1OASYS::abortCurrentCommand] Sending abort command\n", timestamp);
	fflush(Logfile);
#endif
    nErr = domeCommand("09tn00300C2\r\n", resp, SERIAL_BUFFER_SIZE);
    if(nErr)
        return nErr;
    
    return nErr;
}

#pragma mark - Getter / Setter


double Cm1OASYS::getCurrentAz()
{
    if(m_bIsConnected)
        getDomeAz(m_dCurrentAzPosition);

    return m_dCurrentAzPosition;
}

double Cm1OASYS::getCurrentEl()
{
    if(m_bIsConnected)
        getDomeEl(m_dCurrentElPosition);

    return m_dCurrentElPosition;
}

int Cm1OASYS::getCurrentShutterState()
{
    if(m_bIsConnected)
        getShutterState(m_nShutterState);

    return m_nShutterState;
}

