//Jacobo Gomez
//EEC 172 Final Project

//*****************************************************************************
//
//! \addtogroup ssl
//! @{
//
//*****************************************************************************
// Standard includes
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Simplelink includes
#include "simplelink.h"

//Driverlib includes
#include "stdio.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "spi.h"

//Common interface includes
#include "pin_mux_config.h"
#include "gpio_if.h"
#include "common.h"
#include "uart_if.h"

#include "i2c_if.h"

//Adafruit includes
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1351.h"
#include "glcdfont.h"
#include "test.h"

#define MASTER_MODE      1
#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

#define MAX_URI_SIZE 128
#define URI_SIZE MAX_URI_SIZE + 1

#define APPLICATION_NAME        "SSL"
#define APPLICATION_VERSION     "1.1.1.EEC.Spring2020"
#define SERVER_NAME                "a30zi5zl3zumbx-ats.iot.us-east-1.amazonaws.com"
#define GOOGLE_DST_PORT             8443

#define SL_SSL_CA_CERT "/cert/rootCA.der"
#define SL_SSL_PRIVATE "/cert/private.der"
#define SL_SSL_CLIENT  "/cert/client.der"

//NEED TO UPDATE THIS FOR IT TO WORK!
#define DATE                8    /* Current Date */
#define MONTH               06     /* Month 1-12 */
#define YEAR                2020  /* Current year */
#define HOUR                19    /* Time - hours */
#define MINUTE              12    /* Time - minutes */
#define SECOND              0     /* Time - seconds */

#define POSTHEADER "POST /things/CC3200_Thing/shadow HTTP/1.1\n\r"
#define HOSTHEADER "Host: a30zi5zl3zumbx-ats.iot.us-east-1.amazonaws.com\r\n"
#define CHEADER "Connection: Keep-Alive\r\n"
#define CTHEADER "Content-Type: application/json; charset=utf-8\r\n"
#define CLHEADER1 "Content-Length: "
#define CLHEADER2 "\r\n\r\n"
#define FALL_DATA "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" :  {\"default\": \"     FALL DETECTED\", \"sms\": \"     FALL DETECTED\"} \n\r}}}\n\r\n\r"
#define HELP_DATA "{\"state\": {\n\r\"desired\" : {\n\r\"messageagain\" :  {\"default\": \"     HELP NEEDED\", \"sms\": \"     HELP NEEDED\"} \n\r}}}\n\r\n\r"

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    INTERNET_CONNECTION_FAILED = LAN_CONNECTION_FAILED - 1,
    DEVICE_NOT_IN_STATION_MODE = INTERNET_CONNECTION_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

typedef struct
{
   /* time */
   unsigned long tm_sec;
   unsigned long tm_min;
   unsigned long tm_hour;
   /* date */
   unsigned long tm_day;
   unsigned long tm_mon;
   unsigned long tm_year;
   unsigned long tm_week_day; //not required
   unsigned long tm_year_day; //not required
   unsigned long reserved[3];
}SlDateTime;


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulPingPacketsRecv = 0; //Number of Ping Packets received
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
signed char    *g_Host = SERVER_NAME;
SlDateTime g_time;
#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static long WlanConnect();
static int set_time();
static void BoardInit(void);
static long InitializeAppVariables();
static int tls_connect();
static int connectToAccessPoint();
static int http_post_fall(int);
static int http_post_help(int);
long printErrConvenience(char * msg, long retVal);
void display_fall_detected();
void display_help_requested();
int display_count_down();
void display_sent();
void display_abort();
//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent) {
    if(!pWlanEvent) {
        return;
    }

    switch(pWlanEvent->Event) {
        case SL_WLAN_CONNECT_EVENT: {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'.
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                       g_ucConnectionSSID,g_ucConnectionBSSID[0],
                       g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                       g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                       g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT: {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_USER_INITIATED_DISCONNECTION
            if(SL_USER_INITIATED_DISCONNECTION == pEventData->reason_code) {
                UART_PRINT("[WLAN EVENT]Device disconnected from the AP: %s,"
                    "BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else {
                UART_PRINT("[WLAN ERROR]Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        default: {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
    if(!pNetAppEvent) {
        return;
    }

    switch(pNetAppEvent->Event) {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT: {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            //Gateway IP address
            g_ulGatewayIP = pEventData->gateway;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));
        }
        break;

        default: {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent, SlHttpServerResponse_t *pHttpResponse) {
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent) {
    if(!pDevEvent) {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock) {
    if(!pSock) {
        return;
    }

    switch( pSock->Event ) {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status) {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************


//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    0 on success else error code
//!
//! \return None
//!
//*****************************************************************************
static long InitializeAppVariables() {
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    g_Host = SERVER_NAME;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    return SUCCESS;
}


//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState() {
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode 
    if (ROLE_STA != lMode) {
        if (ROLE_AP == lMode) {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
            }
        }

        // Switch to STA role and restart 
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again 
        if (ROLE_STA != lRetVal) {
            // We don't want to proceed if the device is not coming up in STA-mode 
            return DEVICE_NOT_IN_STATION_MODE;
        }
    }
    
    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt, 
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);
    
    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Auto + SmartConfig 
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, 
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore 
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal) {
        // Wait
        while(IS_CONNECTED(g_ulStatus)) {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, 
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();
    
    return lRetVal; // Success
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void BoardInit(void) {
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  0 on success else error code
//!
//! \warning    If the WLAN connection fails or we don't aquire an IP
//!            address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect() {
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    UART_PRINT("Attempting connection to access point: ");
    UART_PRINT(SSID_NAME);
    UART_PRINT("... ...");
    lRetVal = sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT(" Connected!!!\n\r");


    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus))) {
        // Toggle LEDs to Indicate Connection Progress
        _SlNonOsMainLoopTask();
       // GPIO_IF_LedOff(MCU_IP_ALLOC_IND);
        //MAP_UtilsDelay(800000);
        _SlNonOsMainLoopTask();
        //GPIO_IF_LedOn(MCU_IP_ALLOC_IND);
        //MAP_UtilsDelay(800000);
    }

    return SUCCESS;

}

//*****************************************************************************
//
//! This function updates the date and time of CC3200.
//!
//! \param None
//!
//! \return
//!     0 for success, negative otherwise
//!
//*****************************************************************************

static int set_time() {
    long retVal;

    g_time.tm_day = DATE;
    g_time.tm_mon = MONTH;
    g_time.tm_year = YEAR;
    g_time.tm_sec = HOUR;
    g_time.tm_hour = MINUTE;
    g_time.tm_min = SECOND;

    retVal = sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
                          SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                          sizeof(SlDateTime),(unsigned char *)(&g_time));

    ASSERT_ON_ERROR(retVal);
    return SUCCESS;
}

//*****************************************************************************
//
//! This function demonstrates how certificate can be used with SSL.
//! The procedure includes the following steps:
//! 1) connect to an open AP
//! 2) get the server name via a DNS request
//! 3) define all socket options and point to the CA certificate
//! 4) connect to the server via TCP
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  LED1 is turned solid in case of success
//!    LED2 is turned solid in case of failure
//!
//*****************************************************************************
static int tls_connect() {
    SlSockAddrIn_t    Addr;
    int    iAddrSize;
    unsigned char    ucMethod = SL_SO_SEC_METHOD_TLSV1_2;
    unsigned int uiIP,uiCipher = SL_SEC_MASK_TLS_ECDHE_RSA_WITH_AES_256_CBC_SHA;
    long lRetVal = -1;
    int iSockID;

    lRetVal = sl_NetAppDnsGetHostByName(g_Host, strlen((const char *)g_Host),
                                    (unsigned long*)&uiIP, SL_AF_INET);

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't retrieve the host name \n\r", lRetVal);
    }

    Addr.sin_family = SL_AF_INET;
    Addr.sin_port = sl_Htons(GOOGLE_DST_PORT);
    Addr.sin_addr.s_addr = sl_Htonl(uiIP);
    iAddrSize = sizeof(SlSockAddrIn_t);
    //
    // opens a secure socket 
    //
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, SL_SEC_SOCKET);
    if( iSockID < 0 ) {
        return printErrConvenience("Device unable to create secure socket \n\r", lRetVal);
    }

    //
    // configure the socket as TLS1.2
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECMETHOD, &ucMethod,\
                               sizeof(ucMethod));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }
    //
    //configure the socket as ECDHE RSA WITH AES256 CBC SHA
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_SECURE_MASK, &uiCipher,\
                           sizeof(uiCipher));
    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //
    //configure the socket with CA certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                           SL_SO_SECURE_FILES_CA_FILE_NAME, \
                           SL_SSL_CA_CERT, \
                           strlen(SL_SSL_CA_CERT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Client Certificate - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
                SL_SO_SECURE_FILES_CERTIFICATE_FILE_NAME, \
                                    SL_SSL_CLIENT, \
                           strlen(SL_SSL_CLIENT));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }

    //configure the socket with Private Key - for server verification
    //
    lRetVal = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, \
            SL_SO_SECURE_FILES_PRIVATE_KEY_FILE_NAME, \
            SL_SSL_PRIVATE, \
                           strlen(SL_SSL_PRIVATE));

    if(lRetVal < 0) {
        return printErrConvenience("Device couldn't set socket options \n\r", lRetVal);
    }


    /* connect to the peer device - Google server */
    lRetVal = sl_Connect(iSockID, ( SlSockAddr_t *)&Addr, iAddrSize);

    if(lRetVal < 0) {
        UART_PRINT("Device couldn't connect to server:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
        return printErrConvenience("Device couldn't connect to server \n\r", lRetVal);
    }
    else {
        UART_PRINT("Device has connected to the website:");
        UART_PRINT(SERVER_NAME);
        UART_PRINT("\n\r");
    }

    //GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    //GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
    return iSockID;
}



long printErrConvenience(char * msg, long retVal) {
    UART_PRINT(msg);
   // GPIO_IF_LedOn(MCU_RED_LED_GPIO);
    return retVal;
}



int connectToAccessPoint() {
    long lRetVal = -1;
    //GPIO_IF_LedConfigure(LED1|LED3);

   // GPIO_IF_LedOff(MCU_RED_LED_GPIO);
    //GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);

    lRetVal = InitializeAppVariables();
    ASSERT_ON_ERROR(lRetVal);

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0) {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
          UART_PRINT("Failed to configure the device in its default state \n\r");

      return lRetVal;
    }

    UART_PRINT("Device is configured in default state \n\r");

    CLR_STATUS_BIT_ALL(g_ulStatus);

    ///
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal) {
        UART_PRINT("Failed to start the device \n\r");
        return lRetVal;
    }

    UART_PRINT("Device started as STATION \n\r");

    //
    //Connecting to WLAN AP
    //
    lRetVal = WlanConnect();
    if(lRetVal < 0) {
        UART_PRINT("Failed to establish connection w/ an AP \n\r");
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }

    UART_PRINT("Connection established w/ AP and IP is aquired \n\r");
    return 0;
}

static int http_post_fall(int iTLSSockID){ //the POST for fall detection
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(FALL_DATA);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, FALL_DATA);
    pcBufHeaders += strlen(FALL_DATA);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

static int http_post_help(int iTLSSockID){  //the POST for help request
    char acSendBuff[512];
    char acRecvbuff[1460];
    char cCLLength[200];
    char* pcBufHeaders;
    int lRetVal = 0;

    pcBufHeaders = acSendBuff;
    strcpy(pcBufHeaders, POSTHEADER);
    pcBufHeaders += strlen(POSTHEADER);
    strcpy(pcBufHeaders, HOSTHEADER);
    pcBufHeaders += strlen(HOSTHEADER);
    strcpy(pcBufHeaders, CHEADER);
    pcBufHeaders += strlen(CHEADER);
    strcpy(pcBufHeaders, "\r\n\r\n");

    int dataLength = strlen(HELP_DATA);

    strcpy(pcBufHeaders, CTHEADER);
    pcBufHeaders += strlen(CTHEADER);
    strcpy(pcBufHeaders, CLHEADER1);

    pcBufHeaders += strlen(CLHEADER1);
    sprintf(cCLLength, "%d", dataLength);

    strcpy(pcBufHeaders, cCLLength);
    pcBufHeaders += strlen(cCLLength);
    strcpy(pcBufHeaders, CLHEADER2);
    pcBufHeaders += strlen(CLHEADER2);

    strcpy(pcBufHeaders, HELP_DATA);
    pcBufHeaders += strlen(HELP_DATA);

    int testDataLength = strlen(pcBufHeaders);

    UART_PRINT(acSendBuff);


    //
    // Send the packet to the server */
    //
    lRetVal = sl_Send(iTLSSockID, acSendBuff, strlen(acSendBuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("POST failed. Error Number: %i\n\r",lRetVal);
        sl_Close(iTLSSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        return lRetVal;
    }
    lRetVal = sl_Recv(iTLSSockID, &acRecvbuff[0], sizeof(acRecvbuff), 0);
    if(lRetVal < 0) {
        UART_PRINT("Received failed. Error Number: %i\n\r",lRetVal);
        //sl_Close(iSSLSockID);
        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           return lRetVal;
    }
    else {
        acRecvbuff[lRetVal+1] = '\0';
        UART_PRINT(acRecvbuff);
        UART_PRINT("\n\r\n\r");
    }

    return 0;
}

void display_fall_detected()
{
    /* Fall */
    drawChar(50, 5, 'F', RED, BLACK, 2);
    drawChar(60, 5, 'a', RED, BLACK, 2);
    drawChar(70, 5, 'l', RED, BLACK, 2);
    drawChar(80, 5, 'l', RED, BLACK, 2);

    /* Detected */
    drawChar(15, 30, 'D', RED, BLACK, 2);
    drawChar(27, 30, 'e', RED, BLACK, 2);
    drawChar(40, 30, 't', RED, BLACK, 2);
    drawChar(53, 30, 'e', RED, BLACK, 2);
    drawChar(67, 30, 'c', RED, BLACK, 2);
    drawChar(80, 30, 't', RED, BLACK, 2);
    drawChar(94, 30, 'e', RED, BLACK, 2);
    drawChar(108, 30, 'd', RED, BLACK, 2);

    /* Press */
    drawChar(5, 60, 'P', RED, BLACK, 1);
    drawChar(12, 60, 'r', RED, BLACK, 1);
    drawChar(19, 60, 'e', RED, BLACK, 1);
    drawChar(26, 60, 's', RED, BLACK, 1);
    drawChar(33, 60, 's', RED, BLACK, 1);

    /* SW2*/
    drawChar(43, 60, 'S', RED, BLACK, 1);
    drawChar(49, 60, 'W', RED, BLACK, 1);
    drawChar(55, 60, '2', RED, BLACK, 1);

    /* to */
    drawChar(65, 60, 't', RED, BLACK, 1);
    drawChar(71, 60, 'o', RED, BLACK, 1);


    /* abort */
    drawChar(81, 60, 'a', RED, BLACK, 1);
    drawChar(87, 60, 'b', RED, BLACK, 1);
    drawChar(93, 60, 'o', RED, BLACK, 1);
    drawChar(99, 60, 'r', RED, BLACK, 1);
    drawChar(105, 60, 't', RED, BLACK, 1);
}

void display_help_requested()
{

    /* Help */
    drawChar(45, 5, 'H', RED, BLACK, 2);
    drawChar(60, 5, 'e', RED, BLACK, 2);
    drawChar(70, 5, 'l', RED, BLACK, 2);
    drawChar(80, 5, 'p', RED, BLACK, 2);

    /* Requested */
    drawChar(10, 30, 'R', RED, BLACK, 2);
    drawChar(22, 30, 'e', RED, BLACK, 2);
    drawChar(35, 30, 'q', RED, BLACK, 2);
    drawChar(48, 30, 'u', RED, BLACK, 2);
    drawChar(62, 30, 'e', RED, BLACK, 2);
    drawChar(75, 30, 's', RED, BLACK, 2);
    drawChar(89, 30, 't', RED, BLACK, 2);
    drawChar(103, 30, 'e', RED, BLACK, 2);
    drawChar(117, 30, 'd', RED, BLACK, 2);
}

int display_count_down()
{
    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;} //checks if the SW2 is pressed
    drawChar(44, 90, '1', RED, BLACK, 3);
    drawChar(64, 90, '5', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);  //delay of close to 1 second
    drawChar(44, 90, '1', BLACK, BLACK, 3);
    drawChar(64, 90, '5', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(44, 90, '1', RED, BLACK, 3);
    drawChar(64, 90, '4', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(44, 90, '1', BLACK, BLACK, 3);
    drawChar(64, 90, '4', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(44, 90, '1', RED, BLACK, 3);
    drawChar(64, 90, '3', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(44, 90, '1', BLACK, BLACK, 3);
    drawChar(64, 90, '3', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(44, 90, '1', RED, BLACK, 3);
    drawChar(64, 90, '2', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(44, 90, '1', BLACK, BLACK, 3);
    drawChar(64, 90, '2', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(44, 90, '1', RED, BLACK, 3);
    drawChar(64, 90, '1', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(44, 90, '1', BLACK, BLACK, 3);
    drawChar(64, 90, '1', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(44, 90, '1', RED, BLACK, 3);
    drawChar(64, 90, '0', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(44, 90, '1', BLACK, BLACK, 3);
    drawChar(64, 90, '0', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '9', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '9', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '8', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '8', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '7', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '7', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '6', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '6', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '5', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '5', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '4', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '4', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '3', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '3', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '2', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '2', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '1', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '1', BLACK, BLACK, 3);

    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}
    drawChar(64, 90, '0', RED, BLACK, 3);
    MAP_UtilsDelay(9500900);
    drawChar(64, 90, '0', BLACK, BLACK, 3);
    if (GPIOPinRead(GPIOA2_BASE, 0x40) == 0x40){return 1;}

    return 0;

}

void display_sent()
{
    /* Message */
    drawChar(15, 70, 'M', RED, BLACK, 2);
    drawChar(27, 70, 'e', RED, BLACK, 2);
    drawChar(40, 70, 's', RED, BLACK, 2);
    drawChar(53, 70, 's', RED, BLACK, 2);
    drawChar(67, 70, 'a', RED, BLACK, 2);
    drawChar(80, 70, 'g', RED, BLACK, 2);
    drawChar(94, 70, 'e', RED, BLACK, 2);

    /* Sent */
    drawChar(40, 90, 'S', RED, BLACK, 2);
    drawChar(55, 90, 'e', RED, BLACK, 2);
    drawChar(70, 90, 'n', RED, BLACK, 2);
    drawChar(85, 90, 't', RED, BLACK, 2);

}

void display_abort()
{

    /* Aborted */
    drawChar(15, 60, 'A', RED, BLACK, 2);
    drawChar(27, 60, 'b', RED, BLACK, 2);
    drawChar(40, 60, 'o', RED, BLACK, 2);
    drawChar(53, 60, 'r', RED, BLACK, 2);
    drawChar(67, 60, 't', RED, BLACK, 2);
    drawChar(80, 60, 'e', RED, BLACK, 2);
    drawChar(94, 60, 'd', RED, BLACK, 2);

}

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! \return None
//!
//*****************************************************************************
void main() {
    long lRetVal = -1;
    //
    // Initialize board configuration
    //
    BoardInit();

    PinMuxConfig();

    //
    //Enable clock for SPI
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);


    InitTerm();
    ClearTerm();

    UART_PRINT("Hello world!\n\r");

    //Connect the CC3200 to the local access point
    lRetVal = connectToAccessPoint();

    //Set time so that encryption can be used
    lRetVal = set_time();
    if(lRetVal < 0) {
        UART_PRINT("Unable to set time in the device");
        LOOP_FOREVER();
    }
    //Connect to the website with TLS encryption
    lRetVal = tls_connect();
    if(lRetVal < 0) {
        ERR_PRINT(lRetVal);
    }

    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);


    Adafruit_Init();

    unsigned char x_DataBuf[256], y_DataBuf[256], z_DataBuf[256];

    unsigned char ucDevAddr = 0x18;
    unsigned char ucRdLen = 0x01;
    unsigned char x_ucRegOffset = 0x03;
    unsigned char y_ucRegOffset = 0x05;
    unsigned char z_ucRegOffset = 0x07;

    int retval;
    int x_acc, y_acc, z_acc;  //the x, y, and z values of accelerometer.
    int x2, y2, z2;

    double sum;
    double sfactor;

    fillScreen(BLACK);


    while(1){

        retval = I2C_IF_Write(ucDevAddr, &x_ucRegOffset, 1, 0);
        retval = I2C_IF_Read(ucDevAddr, &x_DataBuf[0], ucRdLen); //read accelerometer x register

        retval = I2C_IF_Write(ucDevAddr, &y_ucRegOffset, 1, 0);
        retval = I2C_IF_Read(ucDevAddr, &y_DataBuf[0], ucRdLen); //read accelerometer y register

        retval = I2C_IF_Write(ucDevAddr, &z_ucRegOffset, 1, 0);
        retval = I2C_IF_Read(ucDevAddr, &z_DataBuf[0], ucRdLen); //read accelerometer y register

        x_acc = x_DataBuf[0]; //convert x data to int
        y_acc = y_DataBuf[0]; //convert y data to int
        z_acc = z_DataBuf[0]; //convert z data to int

        //
        //adjust x, y, and z raw data values by sign extend the char value.
        //
        if (((x_acc >> 7)&1)==1){  //
            x_acc = ( ( (-1)&(~0xFF) ) | x_acc );
        }

        if (((y_acc >> 7)&1)==1){
            y_acc = ( ( (-1)&(~0xFF) ) | y_acc);
        }

        if (((z_acc >> 7)&1)==1){
            z_acc = ( ( (-1)&(~0xFF) ) | z_acc);
        }

        //square the adjusted values
        x2 = pow(x_acc, 2.0);
        y2 = pow(y_acc, 2.0);
        z2 = pow(z_acc, 2.0);

        sum = x2 + y2 + z2; //take the sum of the squared values

        sfactor = sqrt(sum); //take the square root of the sum

        if (sfactor < 3){ //if Sfactor is less 3 then a fall has occured
            display_fall_detected();
            if (display_count_down() == 1){ //if SW2 is pressed then abort
                    fillScreen(BLACK);
                    display_abort();
                    MAP_UtilsDelay(9500900);
                    fillScreen(BLACK);
            }else{ //SW2 was not pressed so send SMS
                    fillScreen(BLACK);
                    display_sent();
                    http_post_fall(lRetVal); //post to AWS. Send the fall message
                    MAP_UtilsDelay(9500900);
                    fillScreen(BLACK);
            }
        }

        if (GPIOPinRead(GPIOA1_BASE, 0x20) == 0x20){ //if the SW3 is pressed then help is needed
            display_help_requested();
            http_post_help(lRetVal); //post to AWS. Send the help message
            display_sent();
            MAP_UtilsDelay(9500900);
            fillScreen(BLACK);
        }


    }


}
