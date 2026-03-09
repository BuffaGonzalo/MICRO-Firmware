/*
 * esp01.c
 *
 *  Created on: Jun 17, 2025
 *      Author: gonza
 */

/*
 * ESP01.c
 * Version: 01b05 - 04/08/2024
 */

#include "ESP01.h"
#include <stddef.h>
#include <string.h>
#include <stdlib.h>


static enum {
	ESP01ATIDLE,
	ESP01ATAT,
	ESP01ATRESPONSE,
	ESP01ATCWMODE,
	ESP01ATCIPMUX,
	ESP01ATCWSAP,        /* Nuevo: configura SoftAP */
	ESP01ATCWDHCP,       /* Nuevo: habilita DHCP del AP */
	ESP01ATCIPSERVER,    /* Nuevo: inicia servidor HTTP puerto 80 */
	ESP01ATCWJAP,
	ESP01CWJAPRESPONSE,
	ESP01ATCIFSR,
	ESP01CIFSRRESPONSE,
	ESP01ATCIPCLOSE,
	ESP01ATCIPSTART,
	ESP01CIPSTARTRESPONSE,
	ESP01ATCONNECTED,
	ESP01ATHARDRST0,
	ESP01ATHARDRST1,
	ESP01ATHARDRSTSTOP,
} esp01ATSate = ESP01ATIDLE;

static union{
	struct{
		uint8_t WAITINGSYMBOL: 1;
		uint8_t WIFICONNECTED: 1;
		uint8_t TXCIPSEND: 1;
		uint8_t SENDINGDATA: 1;
		uint8_t HRDRESETON: 1;
		uint8_t ATRESPONSEOK: 1;
		uint8_t UDPTCPCONNECTED: 1;
		uint8_t WAITINGRESPONSE: 1;
	}bit;
	uint8_t byte;
} esp01Flags;

static void ESP01ATDecode();
static void ESP01DOConnection();
static void ESP01SENDData();
static void ESP01StrToBufTX(const char *str);
static void ESP01ByteToBufTX(uint8_t value);

static uint32_t esp01TimeoutTask = 0;
static uint32_t esp01TimeoutDataRx = 0;
static uint32_t esp01TimeoutTxSymbol = 0;
static void (*ESP01ChangeState)(_eESP01STATUS esp01State);
static void (*ESP01DbgStr)(const char *dbgStr);

static char esp01SSID[64] = {0};
static char esp01PASSWORD[32] = {0};
static char esp01RemoteIP[16] = {0};
static char esp01PROTO[4] = "UDP";
static char esp01RemotePORT[6] = {0};
static char esp01LocalIP[16] = {0};
static char esp01LocalPORT[6] = {0};

/* Variables para el modo SoftAP / Webserver */
static char    esp01AP_SSID[32] = {0};
static char    esp01AP_PASS[32] = {0};
static char    esp01AP_CH[3]    = "5";
static char    esp01AP_ENC[2]   = "3"; /* 3 = WPA2 */
static uint8_t esp01WebServerMode = 0; /* 1 = modo webserver activo */
static uint8_t esp01LastConnID    = 0; /* ID de conexion del ultimo +IPD */

static uint8_t esp01HState = 0;
static uint16_t	esp01nBytes = 0;
static uint8_t esp01RXATBuf[ESP01RXBUFAT];
static uint8_t esp01TXATBuf[ESP01TXBUFAT];
static uint16_t	esp01iwRXAT = 0;
static uint16_t	esp01irRXAT = 0;
static uint16_t esp01irTX = 0;
static uint16_t esp01iwTX = 0;

static uint8_t esp01TriesAT = 0;

//static _sESP01Handle esp01Handle = {.DoCHPD = NULL, .WriteUSARTByte = NULL,
//									.bufRX = NULL, .iwRX = NULL, .sizeBufferRX = 0};
static _sESP01Handle esp01Handle = {.DoCHPD = NULL, .WriteUSARTByte = NULL, .WriteByteToBufRX = NULL};

const char ATAT[] = "AT\r\n";
const char ATCIPMUX[] = "AT+CIPMUX=1\r\n";
const char ATCWQAP[] = "AT+CWQAP\r\n";
const char ATCWMODE[] = "AT+CWMODE=3\r\n";
const char ATCWJAP[] = "AT+CWJAP=";
const char ATCIFSR[] = "AT+CIFSR\r\n";
const char ATCIPSTART[] = "AT+CIPSTART=";
const char ATCIPCLOSE[] = "AT+CIPCLOSE\r\n";
const char ATCIPSEND[]   = "AT+CIPSEND=";
const char ATCWSAP[]     = "AT+CWSAP_CUR=";
const char ATCWDHCP[]    = "AT+CWDHCP_CUR=2,1\r\n";
const char ATCIPSERVER[] = "AT+CIPSERVER=1,80\r\n";

const char respAT[] = "0302AT\r";
const char respATp[] = "0302AT+";
const char respOK[] = "0402OK\r\n";
const char respERROR[] = "0702ERROR\r\n";
const char respWIFIGOTIP[] = "1302WIFI GOT IP\r\n";
const char respWIFICONNECTED[] = "1602WIFI CONNECTED\r\n";
const char respWIFIDISCONNECT[] = "1702WIFI DISCONNECT\r\n";
const char respWIFIDISCONNECTED[] = "1902WIFI DISCONNECTED\r\n";
const char respDISCONNECTED[] = "1402DISCONNECTED\r\n";
const char respSENDOK[] = "0902SEND OK\r\n";
const char respCONNECT[] = "0902CONNECT\r\n";
const char respCLOSED[] = "0802CLOSED\r\n";
const char respCIFSRAPIP[] = "1205+CIFSR:STAIP";
const char respBUSY[] = "0602busy .";
const char respIPD[] = "0410+IPD";
const char respReady[] = "0702ready\r\n";
const char respBUSYP[] = "0602busy p";
const char respBUSYS[] = "0602busy s";
// 	  const char respCIFSRAPIP[] = "1102+CIFSR:APIP";
//    const char respCIFSRAPMAC[] = "1202+CIFSR:APMAC";
//    const char respCIFSRSTAIP[] = "1205+CIFSR:STAIP";
//    const char respCIFSRSTAMAC[] = "1302+CIFSR:STAMAC";

const char *const responses[] = {respAT, respATp, respOK, respERROR, respWIFIGOTIP, respWIFICONNECTED,
								 respWIFIDISCONNECT, respWIFIDISCONNECTED, respDISCONNECTED, respSENDOK, respCONNECT, respCLOSED,
								 respCIFSRAPIP, respBUSY, respIPD, respReady, respBUSYP, respBUSYS, NULL};

static uint8_t indexResponse = 0;
static uint8_t indexResponseChar = 0;

//const char _DNSFAIL[] = "DNS FAIL\r";
//const char _ATCIPDNS[] = "AT+CIPDNS_CUR=1,\"208.67.220.220\",\"8.8.8.8\"\r\n";
//const char CIFSRAPIP[] = "+CIFSR:APIP\r";
//const char CIFSRAPMAC[] = "+CIFSR:APMAC\r";
//const char CIFSRSTAIP[] = "+CIFSR:STAIP\r";
//const char CIFSRSTAMAC[] = "+CIFSR:STAMAC\r";


void ESP01_SetWIFI(const char *ssid, const char *password){
	esp01ATSate = ESP01ATIDLE;
	esp01Flags.byte = 0;
	esp01WebServerMode = 0;   /* Salir del modo webserver al conectar como Station */

	strncpy(esp01SSID, ssid, 64);
	esp01SSID[63] = '\0';
	strncpy(esp01PASSWORD, password, 32);
	esp01PASSWORD[31] = '\0';

	esp01TimeoutTask = 50;
	esp01ATSate = ESP01ATHARDRST0;

	esp01TriesAT = 0;
}

/**
 * @brief Configura el ESP01 en modo SoftAP + inicia servidor HTTP en puerto 80
 *
 * El STM32 podra recibir peticiones HTTP con SSID y contraseña para luego
 * llamar a ESP01_SetWIFI() y conectarse como Station.
 *
 * @param apSSID   Nombre de la red WiFi a crear
 * @param apPass   Contraseña del AP (minimo 8 chars). NULL o "" para red abierta
 * @param ch       Canal WiFi (1-13)
 * @param enc      Encriptacion: 0=abierta, 2=WPA, 3=WPA2, 4=WPA/WPA2
 */
void ESP01_SetWebServer(const char *apSSID, const char *apPass, uint8_t ch, uint8_t enc){
	esp01ATSate = ESP01ATIDLE;
	esp01Flags.byte = 0;
	esp01WebServerMode = 1;

	strncpy(esp01AP_SSID, apSSID, 31);
	esp01AP_SSID[31] = '\0';

	if(apPass && apPass[0] != '\0'){
		strncpy(esp01AP_PASS, apPass, 31);
		esp01AP_PASS[31] = '\0';
	} else {
		esp01AP_PASS[0] = '\0';
		enc = 0; /* Sin contraseña → red abierta */
	}

	itoa(ch,  esp01AP_CH,  10);
	itoa(enc, esp01AP_ENC, 10);

	esp01TimeoutTask = 50;
	esp01ATSate = ESP01ATHARDRST0;
	esp01TriesAT = 0;
}

/**
 * @brief Devuelve el ID de la ultima conexion recibida por +IPD
 *
 * Usar este valor para pasarlo a ESP01_Send() cuando se esta en modo webserver
 */
uint8_t ESP01_GetLastConnID(){
	return esp01LastConnID;
}


_eESP01STATUS ESP01_StartUDP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(LocalPORT == 0)
		LocalPORT = 30000;

	strcpy(esp01PROTO, "UDP");

	strncpy(esp01RemoteIP, RemoteIP, 15);
	esp01RemoteIP[15] = '\0';

	itoa(RemotePORT, esp01RemotePORT, 10);
	itoa(LocalPORT, esp01LocalPORT, 10);

	if(esp01SSID[0] == '\0')
		return ESP01_WIFI_NOT_SETED;

	if(esp01Flags.bit.WIFICONNECTED == 0)
		return ESP01_WIFI_DISCONNECTED;

	esp01ATSate = ESP01ATCIPCLOSE;

	return ESP01_UDPTCP_CONNECTING;
}

_eESP01STATUS ESP01_StartTCP(const char *RemoteIP, uint16_t RemotePORT, uint16_t LocalPORT){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(LocalPORT == 0)
		LocalPORT = 30000;

	strcpy(esp01PROTO, "TCP");

	strncpy(esp01RemoteIP, RemoteIP, 15);
	esp01RemoteIP[15] = '\0';

	itoa(RemotePORT, esp01RemotePORT, 10);
	itoa(LocalPORT, esp01LocalPORT, 10);

	if(esp01SSID[0] == '\0')
		return ESP01_WIFI_NOT_SETED;

	if(esp01Flags.bit.WIFICONNECTED == 0)
		return ESP01_WIFI_DISCONNECTED;

	esp01ATSate = ESP01ATCIPCLOSE;

	return ESP01_UDPTCP_CONNECTING;
}


void ESP01_CloseUDPTCP(){
	if(esp01Handle.WriteUSARTByte == NULL)
		return;

	esp01ATSate = ESP01ATCIPCLOSE;
}

_eESP01STATUS ESP01_StateWIFI(){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.WIFICONNECTED)
		return ESP01_WIFI_CONNECTED;
	else
		return ESP01_WIFI_DISCONNECTED;
}

char *ESP01_GetLocalIP(){
	if(esp01Flags.bit.WIFICONNECTED &&  esp01LocalIP[0]!='\0')
		return esp01LocalIP;

	return NULL;
}


_eESP01STATUS ESP01_StateUDPTCP(){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.UDPTCPCONNECTED)
		return ESP01_UDPTCP_CONNECTED;
	else
		return ESP01_UDPTCP_DISCONNECTED;
}


void ESP01_WriteRX(uint8_t value){
//	if(esp01Handle.bufRX == NULL)
//		return;
	esp01RXATBuf[esp01iwRXAT++] = value;
	if(esp01iwRXAT == ESP01RXBUFAT)
		esp01iwRXAT = 0;
}

_eESP01STATUS ESP01_Send(uint8_t connID, uint8_t *buf, uint16_t irRingBuf, uint16_t length, uint16_t sizeRingBuf){
	if(esp01Handle.WriteUSARTByte == NULL)
		return ESP01_NOT_INIT;

	if(esp01Flags.bit.UDPTCPCONNECTED == 0 && !esp01WebServerMode)
		return ESP01_UDPTCP_DISCONNECTED;

	if(esp01Flags.bit.SENDINGDATA == 0){
		char strInt[10];
		uint8_t l = 0;

		itoa(length, strInt, 10);
		l = strlen(strInt);
		if(l>4 || l==0)
			return ESP01_SEND_ERROR;

		ESP01StrToBufTX(ATCIPSEND);

		/* Con CIPMUX=1 (webserver) el formato es AT+CIPSEND=connID,length */
		/* Con CIPMUX=0 (UDP/TCP)   el formato es AT+CIPSEND=length        */
		if(esp01WebServerMode){
			char connStr[4];
			itoa(connID, connStr, 10);
			ESP01StrToBufTX(connStr);
			ESP01ByteToBufTX(',');
		}

		ESP01StrToBufTX(strInt);
		ESP01StrToBufTX("\r>");

		for(uint16_t i=0; i<length; i++){
			esp01TXATBuf[esp01iwTX++] = buf[irRingBuf++];
			if(esp01iwTX == ESP01TXBUFAT)
				esp01iwTX = 0;
			if(irRingBuf == sizeRingBuf)
				irRingBuf = 0;
		}

		esp01Flags.bit.TXCIPSEND = 1;
		esp01Flags.bit.SENDINGDATA = 1;

		if(ESP01DbgStr != NULL){
			ESP01DbgStr("+&DBGSENDING DATA ");
			ESP01DbgStr(strInt);
			ESP01DbgStr("\n");
		}

		return ESP01_SEND_READY;
	}

	if(ESP01DbgStr != NULL)
		ESP01DbgStr("+&DBGSENDING DATA BUSY\n");

	return ESP01_SEND_BUSY;
}


void ESP01_Init(_sESP01Handle *hESP01){

	memcpy(&esp01Handle, hESP01, sizeof(_sESP01Handle));

	esp01ATSate = ESP01ATIDLE;
	esp01HState = 0;
	esp01irTX = 0;
	esp01iwTX = 0;
	esp01irRXAT = 0;
	esp01iwRXAT = 0;
	esp01Flags.byte = 0;
	esp01WebServerMode = 0;
	esp01LastConnID    = 0;
	ESP01ChangeState = NULL;
	ESP01DbgStr = NULL;
}


void ESP01_Timeout10ms(){
	if(esp01TimeoutTask)
		esp01TimeoutTask--;

	if(esp01TimeoutDataRx){
		esp01TimeoutDataRx--;
		if(!esp01TimeoutDataRx)
			esp01HState = 0;
	}

	if(esp01TimeoutTxSymbol)
		esp01TimeoutTxSymbol--;
}

void ESP01_Task(){

	if(esp01irRXAT != esp01iwRXAT)
		ESP01ATDecode();

	if(!esp01TimeoutTask)
		ESP01DOConnection();

	ESP01SENDData();
}

void ESP01_AttachChangeState(void (*aESP01ChangeState)(_eESP01STATUS esp01State)){
	ESP01ChangeState = aESP01ChangeState;
}

void ESP01_AttachDebugStr(void (*aESP01DbgStr)(const char *dbgStr)){
	ESP01DbgStr = aESP01DbgStr;
}

int ESP01_IsHDRRST(){
	if(esp01ATSate==ESP01ATHARDRST0 || esp01ATSate==ESP01ATHARDRST1 || esp01ATSate==ESP01ATHARDRSTSTOP)
		return 1;
	return 0;
}




/* Private Functions */
static void ESP01ATDecode(){
	uint16_t i;
	uint8_t value;

	if(esp01ATSate==ESP01ATHARDRST0 || esp01ATSate==ESP01ATHARDRST1 ||
	   esp01ATSate==ESP01ATHARDRSTSTOP){
		esp01irRXAT = esp01iwRXAT;
		return;
	}


	i = esp01iwRXAT;
	esp01TimeoutDataRx = 2;
	while(esp01irRXAT != i){
		value = esp01RXATBuf[esp01irRXAT];
		switch(esp01HState){
		case 0:
            indexResponse = 0;
            indexResponseChar = 4;
            while(responses[indexResponse] != NULL){
                if(value == responses[indexResponse][indexResponseChar]){
                    esp01nBytes = (responses[indexResponse][0] - '0');
                    esp01nBytes *= 10;
                    esp01nBytes += (responses[indexResponse][1] - '0');
                    esp01nBytes--;
                    break;
                }
                indexResponse++;
            }
            if(responses[indexResponse] != NULL){
                esp01HState = 1;
                indexResponseChar++;
            }
			else{
				esp01TimeoutDataRx = 0;
				if(esp01Flags.bit.WAITINGSYMBOL){
					if(value == '>'){
						esp01Flags.bit.WAITINGSYMBOL = 0;
						esp01TimeoutTxSymbol = 0;
					}
				}
			}
			break;
		case 1:
            if(value == responses[indexResponse][indexResponseChar]){
                esp01nBytes--;
                if(!esp01nBytes || value=='\r'){
                    esp01HState = (responses[indexResponse][2] - '0');
                    esp01HState *= 10;
                    esp01HState += (responses[indexResponse][3] - '0');
                    break;
                }
            }
            else{
                indexResponse = 0;
                while(responses[indexResponse] != NULL){
                    esp01nBytes = (responses[indexResponse][0] - '0');
                    esp01nBytes *= 10;
                    esp01nBytes += (responses[indexResponse][1] - '0');
                    esp01nBytes -= (indexResponseChar-3);
                    if(esp01nBytes<128 && value==responses[indexResponse][indexResponseChar]){
                        if(esp01nBytes == 0){
                            esp01HState = (responses[indexResponse][2] - '0');
                            esp01HState *= 10;
                            esp01HState += (responses[indexResponse][3] - '0');
                        }
                        break;
                    }
                    indexResponse++;
                }
                if(responses[indexResponse] == NULL){
                    esp01HState = 0;
                    esp01irRXAT--;
                    break;
                }
            }
			indexResponseChar++;
			break;
		case 2:
			if(value == '\n'){
				esp01HState = 0;
				switch(indexResponse){
				case 0://AT
				case 1:
					break;
				case 2://OK
					if(esp01ATSate == ESP01ATRESPONSE){
						esp01TimeoutTask = 0;
						esp01Flags.bit.ATRESPONSEOK = 1;
					}
					break;
				case 3://ERROR
					if(esp01Flags.bit.SENDINGDATA){
						esp01Flags.bit.SENDINGDATA = 0;
						esp01Flags.bit.UDPTCPCONNECTED = 0;
						esp01irTX = esp01iwTX;
					}
					break;
				case 4://WIFI GOT IP
					esp01TimeoutTask = 0;
					if(esp01ATSate == ESP01CWJAPRESPONSE)
						esp01Flags.bit.ATRESPONSEOK = 1;
					esp01Flags.bit.WIFICONNECTED = 1;
					if(ESP01ChangeState != NULL)
						ESP01ChangeState(ESP01_WIFI_CONNECTED);
					break;
				case 5://WIFI CONNECTED
					break;
				case 6://WIFI DISCONNECT
				case 7://WIFI DISCONNECTED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					if(ESP01ChangeState != NULL)
						ESP01ChangeState(ESP01_WIFI_DISCONNECTED);
					/* En modo webserver el WIFI DISCONNECT viene del cliente que se
					 * desconecta del SoftAP, o de CWMODE=3 al arrancar.
					 * NO debemos resetear: el servidor HTTP debe seguir activo. */
					if(esp01WebServerMode)
						break;
					esp01Flags.bit.WIFICONNECTED = 0;
					if(esp01ATSate == ESP01CWJAPRESPONSE)
						break;
					esp01ATSate = ESP01ATHARDRSTSTOP;
					break;
				case 8://DISCONNECTED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					break;
				case 9://SEND OK
					esp01Flags.bit.SENDINGDATA = 0;
					if(ESP01ChangeState != NULL)
						ESP01ChangeState(ESP01_SEND_OK);
					break;
				case 10://CONNECT
					/* En CIPMUX=0: "CONNECT\r\n" indica conexion UDP/TCP establecida.
					 * En CIPMUX=1: "X,CONNECT\r\n" (con ID) → no matchea aqui.
					 * Solo actuamos si NO estamos en webserver para no interferir. */
					if(!esp01WebServerMode){
						esp01TimeoutTask = 0;
						esp01Flags.bit.ATRESPONSEOK = 1;
						esp01Flags.bit.UDPTCPCONNECTED = 1;
						if(ESP01ChangeState != NULL)
							ESP01ChangeState(ESP01_UDPTCP_CONNECTED);
					}
					break;
				case 11://CLOSED
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					break;
				case 13://busy
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					esp01Flags.bit.WIFICONNECTED = 0;
					break;
				case 15://ready
					esp01Flags.bit.UDPTCPCONNECTED = 0;
					esp01Flags.bit.WIFICONNECTED = 0;
					esp01ATSate = ESP01ATHARDRSTSTOP;
					break;
				case 16://busy p
					break;
				case 17://busy s
					break;
				}
			}
			break;
		case 5://CIFR,STAIP
			if(value == ','){
				esp01HState = 6;
				if(ESP01DbgStr != NULL)
					ESP01DbgStr("+&DBGRESPONSE CIFSR\n");
			}
			else{
				esp01HState = 0;
				esp01irRXAT--;
				if(ESP01DbgStr != NULL)
					ESP01DbgStr("+&DBGERROR CIFSR 5\n");
			}
			break;
		case 6:
			if(value == '\"'){
				esp01HState = 7;
				esp01nBytes = 0;
			}
			break;
		case 7:
			if(value == '\"' || esp01nBytes==16)
				esp01HState = 8;
			else
				esp01LocalIP[esp01nBytes++] = value;
			break;
		case 8:
			if(value == '\n'){
				esp01HState = 0;
				if(esp01nBytes < 16){
					esp01LocalIP[esp01nBytes] = '\0';
					esp01Flags.bit.ATRESPONSEOK = 1;
					esp01TimeoutTask = 0;
				}
				else
					esp01LocalIP[0] = '\0';
				if(ESP01ChangeState != NULL)
					ESP01ChangeState(ESP01_WIFI_NEW_IP);
			}
			break;
		case 10://IPD - primera coma
			if(value == ','){
				if(esp01WebServerMode){
					/* Con CIPMUX=1: +IPD,connID,N:data  →  leer connID primero */
					esp01HState = 13;
					esp01LastConnID = 0;
				} else {
					/* Con CIPMUX=0: +IPD,N:data  →  leer longitud directamente */
					esp01HState = 11;
					esp01nBytes = 0;
				}
			}
			else{
				esp01HState = 0;
				esp01irRXAT--;
			}
			break;
		case 13://IPD connID (solo CIPMUX=1): espera la segunda coma
			if(value == ','){
				esp01HState = 11;
				esp01nBytes = 0;
			}
			else if(value >= '0' && value <= '9'){
				esp01LastConnID = (uint8_t)(esp01LastConnID * 10 + (value - '0'));
			}
			else{
				esp01HState = 0;
				esp01irRXAT--;
			}
			break;
		case 11:
			if(value == ':')
				esp01HState = 12;
			else{
				if(value<'0' || value>'9'){
					esp01HState = 0;
					esp01irRXAT--;
				}
				else{
					esp01nBytes *= 10;
					esp01nBytes += (value - '0');
				}
			}
			break;
		case 12:
			if(esp01Handle.WriteByteToBufRX != NULL)
				esp01Handle.WriteByteToBufRX(value);
			esp01nBytes--;
			if(!esp01nBytes){
				esp01HState = 0;
				if(ESP01DbgStr != NULL)
					ESP01DbgStr("+&DBGRESPONSE IPD\n");
			}
			break;
		default:
			esp01HState = 0;
			esp01TimeoutDataRx = 0;
		}

		esp01irRXAT++;
		if(esp01irRXAT == ESP01RXBUFAT)
			esp01irRXAT = 0;
	}

}

static void ESP01DOConnection(){

	esp01TimeoutTask = 100;
	switch(esp01ATSate){
	case ESP01ATIDLE:
		esp01TimeoutTask = 0;
		break;
	case ESP01ATHARDRST0:
		esp01Handle.DoCHPD(0);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01HARDRESET0\n");
		esp01ATSate = ESP01ATHARDRST1;
		break;
	case ESP01ATHARDRST1:
		esp01Handle.DoCHPD(1);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01HARDRESET1\n");
		esp01ATSate = ESP01ATHARDRSTSTOP;
		esp01TimeoutTask = 500;
		break;
	case ESP01ATHARDRSTSTOP:
		esp01ATSate = ESP01ATAT;
		esp01TriesAT = 0;
		break;
	case ESP01ATAT:
		if(esp01TriesAT){
			esp01TriesAT--;
			if(!esp01TriesAT){
				esp01ATSate = ESP01ATHARDRST0;
				break;
			}
		}
		else
			esp01TriesAT = 4;

		esp01Flags.bit.ATRESPONSEOK = 0;
		ESP01StrToBufTX(ATAT);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01AT\n");
		esp01ATSate = ESP01ATRESPONSE;
		break;
	case ESP01ATRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCWMODE;
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCWMODE:
		ESP01StrToBufTX(ATCWMODE);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCWMODE\n");
		esp01ATSate = ESP01ATCIPMUX;
		break;
	case ESP01ATCIPMUX:
		ESP01StrToBufTX(ATCIPMUX);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCIPMUX\n");
		/* Bifurcar: webserver (AP+CIPSERVER) vs. station (CWJAP) */
		esp01ATSate = (esp01WebServerMode) ? ESP01ATCWSAP : ESP01CWJAPRESPONSE;
		break;

	/* ---- NUEVOS ESTADOS: modo SoftAP + Webserver ---- */
	case ESP01ATCWSAP:
		/* AT+CWSAP_CUR="ssid","pass",canal,enc */
		ESP01StrToBufTX(ATCWSAP);
		ESP01ByteToBufTX('"');
		ESP01StrToBufTX(esp01AP_SSID);
		ESP01ByteToBufTX('"');
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('"');
		ESP01StrToBufTX(esp01AP_PASS);
		ESP01ByteToBufTX('"');
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01AP_CH);
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01AP_ENC);
		ESP01ByteToBufTX('\r');
		ESP01ByteToBufTX('\n');
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCWSAP\n");
		esp01ATSate = ESP01ATCWDHCP;
		break;
	case ESP01ATCWDHCP:
		/* AT+CWDHCP_CUR=2,1  →  habilita DHCP del SoftAP */
		ESP01StrToBufTX(ATCWDHCP);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCWDHCP\n");
		esp01ATSate = ESP01ATCIPSERVER;
		break;
	case ESP01ATCIPSERVER:
		/* AT+CIPSERVER=1,80  →  inicia servidor TCP en puerto 80 */
		ESP01StrToBufTX(ATCIPSERVER);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCIPSERVER\n");
		esp01ATSate = ESP01ATCONNECTED;
		break;
	/* ---- FIN NUEVOS ESTADOS ---- */
	case ESP01ATCWJAP:
		if(esp01Flags.bit.WIFICONNECTED){
			esp01ATSate = ESP01ATCIFSR;
			break;
		}
		if(esp01SSID[0] == '\0')
			break;
		ESP01StrToBufTX(ATCWJAP);
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01SSID);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01PASSWORD);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX('\r');
		ESP01ByteToBufTX('\n');
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCWJAP\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CWJAPRESPONSE;
		esp01TimeoutTask = 1500;
		break;
	case ESP01CWJAPRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK){
			esp01ATSate = ESP01ATCIFSR;
			esp01TriesAT = 4;
		}
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCIFSR:
		esp01LocalIP[0] = '\0';
		ESP01StrToBufTX(ATCIFSR);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01CIFSR\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01ATSate = ESP01CIFSRRESPONSE;
		break;
	case ESP01CIFSRRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCIPCLOSE;
		else{
			esp01TriesAT--;
			if(esp01TriesAT == 0){
				esp01ATSate = ESP01ATAT;
				break;
			}
			esp01ATSate = ESP01ATCIFSR;
		}
		break;
	case ESP01ATCIPCLOSE:
		if(esp01RemoteIP[0] == '\0')
			break;
		ESP01StrToBufTX(ATCIPCLOSE);
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCIPCLOSE\n");
		esp01ATSate = ESP01ATCIPSTART;
		break;
	case ESP01ATCIPSTART:
		ESP01StrToBufTX(ATCIPSTART);
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01PROTO);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('\"');
		ESP01StrToBufTX(esp01RemoteIP);
		ESP01ByteToBufTX('\"');
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01RemotePORT);
		ESP01ByteToBufTX(',');
		ESP01StrToBufTX(esp01LocalPORT);
		ESP01ByteToBufTX(',');
		ESP01ByteToBufTX('0');
		ESP01ByteToBufTX('\r');
		ESP01ByteToBufTX('\n');
		if(ESP01DbgStr != NULL)
			ESP01DbgStr("+&DBGESP01ATCIPSTART\n");
		esp01Flags.bit.ATRESPONSEOK = 0;
		esp01Flags.bit.UDPTCPCONNECTED = 0;
		esp01ATSate = ESP01CIPSTARTRESPONSE;
		esp01TimeoutTask = 200;
		break;
	case ESP01CIPSTARTRESPONSE:
		if(esp01Flags.bit.ATRESPONSEOK)
			esp01ATSate = ESP01ATCONNECTED;
		else
			esp01ATSate = ESP01ATAT;
		break;
	case ESP01ATCONNECTED:
		/* En modo webserver solo esperamos conexiones entrantes */
		if(esp01WebServerMode){
			esp01TimeoutTask = 0;
			break;
		}
		if(esp01Flags.bit.WIFICONNECTED == 0){
			esp01ATSate = ESP01ATAT;
			break;
		}
		if(esp01Flags.bit.UDPTCPCONNECTED == 0){
			esp01ATSate = ESP01ATCIPCLOSE;
			break;
		}
		esp01TimeoutTask = 0;
		break;
	}
}

static void ESP01SENDData(){
	uint8_t value;

	if(esp01Flags.bit.WAITINGSYMBOL){
		if(!esp01TimeoutTxSymbol){
			esp01irTX = esp01iwTX;
			esp01Flags.bit.WAITINGSYMBOL = 0;
			esp01Flags.bit.SENDINGDATA   = 0;
			/* En modo webserver NO reiniciamos el AT: el servidor sigue activo */
			if(!esp01WebServerMode){
				esp01ATSate = ESP01ATAT;
				esp01TimeoutTask = 10;
			}
		}
		return;
	}
	if(esp01irTX != esp01iwTX){
		value = esp01TXATBuf[esp01irTX];
		if(esp01Flags.bit.TXCIPSEND){
			if(value == '>')
				value = '\n';
		}
		if(esp01Handle.WriteUSARTByte(value)){
			if(esp01Flags.bit.TXCIPSEND){
				if(esp01TXATBuf[esp01irTX] == '>'){
					esp01Flags.bit.TXCIPSEND = 0;
					esp01Flags.bit.WAITINGSYMBOL = 1;
					esp01TimeoutTxSymbol = 5;
				}
			}
			esp01irTX++;
			if(esp01irTX == ESP01TXBUFAT)
				esp01irTX = 0;
		}
	}
}

static void ESP01StrToBufTX(const char *str){
	for(int i=0; str[i]; i++){
		esp01TXATBuf[esp01iwTX++] = str[i];
		if(esp01iwTX == ESP01TXBUFAT)
			esp01iwTX = 0;
	}
}

static void ESP01ByteToBufTX(uint8_t value){
	esp01TXATBuf[esp01iwTX++] = value;
	if(esp01iwTX == ESP01TXBUFAT)
		esp01iwTX = 0;
}


/* END Private Functions*/
