/**
 * @file   protocol.h
 * @author Gonzalo M. buffa
 * @date   24/05/2025
 *
 * protocol.c
 * File part of the protocol library, based un the UNER protocol. It's designed to simplify the
 * the development of code for the user by reducing the amount of code lines in the main.c
 *
 * User Attention!!!
 * The library automates the configuration/decryption of headers and some data manipulation
 * The user still needs to handle the received data (RxData) and also what to do with the received data.
 */

#include "util.h"
#include <stdint.h>
#include <unerPrtcl.h>

#include "main.h"

uint8_t isCommand = FALSE;
uint8_t chk = 0;


//Function definitions

uint8_t unerPrtcl_PutHeaderOnTx(_sComm  *dataTx, uint8_t ID, uint8_t frameLength)
{
	frameLength++;
    dataTx->chk = 0;
    dataTx->indexData = dataTx->indexW;

    dataTx->buff[dataTx->indexW++]='U';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='N';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='E';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]='R';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=frameLength;
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=':';
    dataTx->indexW &= dataTx->mask;
    dataTx->buff[dataTx->indexW++]=ID;
    dataTx->indexW &= dataTx->mask;

    dataTx->bytes = TXBYTES;
    dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^frameLength ^':'^ID) ;

    return  dataTx->chk;
}

uint8_t unerPrtcl_PutByteOnTx(_sComm *dataTx, uint8_t byte)
{
	dataTx->bytes++;
    dataTx->buff[dataTx->indexW++]=byte;
    dataTx->indexW &= dataTx->mask;
    dataTx->chk ^= byte;
    return dataTx->chk;
}

uint8_t unerPrtcl_PutStrOntx(_sComm *dataTx, const char *str)
{
    volatile uint8_t globalIndex=0;
    while(str[globalIndex]){
    	dataTx->bytes++;
        dataTx->buff[dataTx->indexW++]=str[globalIndex];
        dataTx->indexW &= dataTx->mask;
        dataTx->chk ^= str[globalIndex++];
    }
    //dataTx->bytes += ++globalIndex;
    return dataTx->chk;
}

uint8_t unerPrtcl_GetByteFromRx(_sComm *dataRx, uint8_t start, uint8_t end) {
	uint8_t getByte;
	dataRx->indexData += start;
	dataRx->indexData &= dataRx->mask;
	getByte = dataRx->buff[dataRx->indexData];
	dataRx->indexData += end;
	dataRx->indexData &= dataRx->mask;
	return getByte;
}


uint8_t unerPrtcl_DecodeHeader(_sComm *dataRx)
{
	uint8_t nBytes = 0;
	static uint8_t header = HEADER_U;
    uint8_t auxIndex=dataRx->indexW;
    while(dataRx->indexR != auxIndex){
        switch(header)
        {
            case HEADER_U:
                if(dataRx->buff[dataRx->indexR] == 'U'){
                   header = HEADER_N;
                }
            break;
            case HEADER_N:
                if(dataRx->buff[dataRx->indexR] == 'N'){
                    header = HEADER_E;
                }else{
                    if(dataRx->buff[dataRx->indexR] != 'U'){
                    	header = HEADER_U;
                        dataRx->indexR--;
                    }
                }
            break;
            case HEADER_E:
                if(dataRx->buff[dataRx->indexR] == 'E'){
                    header = HEADER_R;
                }else{
                    header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case HEADER_R:
                if(dataRx->buff[dataRx->indexR] == 'R'){
                    header = NBYTES;
                }else{
                    header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case NBYTES:
                nBytes=dataRx->buff[dataRx->indexR];
                header = TOKEN;
            break;
            case TOKEN:
                if(dataRx->buff[dataRx->indexR] == ':'){
                    header = PAYLOAD;
                    dataRx->indexData = dataRx->indexR+1;
                    dataRx->indexData &= dataRx->mask;
                    dataRx->chk = 0;
                    dataRx->chk ^= ('U' ^'N' ^'E' ^'R' ^nBytes ^':') ;
                }else{
                    header = HEADER_U;
                    dataRx->indexR--;
                }
            break;
            case PAYLOAD:
                nBytes--;
                if(nBytes>0){
                   dataRx->chk ^= dataRx->buff[dataRx->indexR];
                }else{
                    header = HEADER_U;
                    if(dataRx->buff[dataRx->indexR] == dataRx->chk)
                        return TRUE;
                }
            break;
            default:
                header = HEADER_U;
            break;
        }
        dataRx->indexR++;
        dataRx->indexR &= dataRx->mask;
    }
    return FALSE;
}

void unerPrtcl_Init(_sComm *Rx, _sComm *Tx, volatile uint8_t *buffRx, volatile uint8_t *buffTx){
	Rx->buff = (uint8_t *)buffRx;
    Rx->indexR = 0;
    Rx->indexW = 0;
    Rx->indexData = 0;
    Rx->bytes = 0;
    Rx->mask = RXBUFSIZE - 1; //Control de buffer 2n-1
    Rx->chk = 0;

    Tx->buff = (uint8_t *)buffTx;
    Tx->indexR = 0;
    Tx->indexW = 0;
    Tx->indexData = 0;
    Tx->bytes = 0;
    Tx->mask = TXBUFSIZE - 1;
    Tx->chk = 0;

}


