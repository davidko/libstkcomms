/*
   This file is part of BaroboLink.

   Foobar is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   Foobar is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _LIBSTKCOMMS_H_
#define _LIBSTKCOMMS_H_
#ifndef _WIN32
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#ifndef __MACH__
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#else
#include <stdint.h>
#endif
#else
#include <winsock2.h>
#ifdef ENABLE_BLUETOOTH
#include <basetyps.h>
#include <ws2bth.h>
#endif
typedef unsigned char uint8_t;
#define uint16_t UINT16
/*
typedef struct bdaddr_s {
  UINT8 b[6];
} bdaddr_t;
*/
void baswap(bdaddr_t *dst, const bdaddr_t *src);
int str2ba(const char *str, bdaddr_t *ba);
#endif
#include "thread_macros.h"

typedef enum robot_type_e
{
  MOBOT_NONE,
  MOBOT_A,
  MOBOT_IL
} robot_type_t;

typedef enum connection_type_e
{
  CONNECT_NONE,
  CONNECT_SOCKET,
  CONNECT_TTY
} connection_type_t;

#ifdef BUILD_CSTKCOMMS
typedef struct stkComms_s
{
  int socket;
  int isConnected;
  int programComplete;
  uint8_t signature[3];
#if defined (_WIN32) or defined (_MSYS)
  HANDLE commHandle;
  OVERLAPPED ov;
#endif

  MUTEX_T* progressLock;
  COND_T* progressCond;
  double progress;
  char* lockfileName;

#if !defined (_MSYS)
#ifdef _WIN32
#ifdef ENABLE_BLUETOOTH
  SOCKADDR_BTH addr;
#endif
#elif defined __MACH__
  void* addr;
#else
  struct sockaddr_rc addr;
#endif
#endif
  robot_type_t formFactor;
  connection_type_t connectionType;
} stkComms_t;
#else
struct stkComms_s;
typedef struct stkComms_s stkComms_t;
#endif

#ifdef BUILD_CSTKCOMMS
typedef struct hexFile_s
{
    uint8_t *data;
    int dataAllocSize;
    int len;
} hexFile_t;
#else
struct hexFile_s;
typedef struct hexFile_s hexFile_t;
#endif
class CHexFile;
extern "C" {
stkComms_t* stkComms_new();
int stkComms_init(stkComms_t* comms);
int stkComms_destroy(stkComms_t* comms);
int stkComms_connect(stkComms_t* comms, const char addr[]);
int stkComms_connectWithAddressTTY(stkComms_t* comms, const char* address);
int stkComms_connectWithTTY(stkComms_t* comms, const char* ttyfilename);
int stkComms_disconnect(stkComms_t* comms);
int stkComms_setSocket(stkComms_t* comms, int socket);
double stkComms_getProgress(stkComms_t* comms);
void stkComms_setProgress(stkComms_t* comms, double progress);
int stkComms_isProgramComplete(stkComms_t* comms); 
void stkComms_setProgramComplete(stkComms_t* comms, int complete);
int stkComms_handshake(stkComms_t* comms);
int stkComms_setDevice(
      stkComms_t* comms,
      uint8_t     DeviceCode,
      uint8_t     Revision,
      uint8_t     progtype,
      uint8_t     parmode,
      uint8_t     polling,
      uint8_t     selftimed,
      uint8_t     lockbytes,
      uint8_t     fusebytes,
      uint8_t     flashpollval1,
      uint8_t     flashpollval2,
      uint8_t     eeprompollval1,
      uint8_t     eeprompollval2,
      uint8_t     pagesizehigh,
      uint8_t     pagesizelow,
      uint8_t     eepromsizehigh,
      uint8_t     eepromsizelow,
      uint8_t     flashsize4,
      uint8_t     flashsize3,
      uint8_t     flashsize2,
      uint8_t     flashsize1);
int stkComms_setDeviceExt(
      stkComms_t* comms,
      uint8_t    commandsize,
      uint8_t    eeprompagesize,
      uint8_t    signalpagel,
      uint8_t    signalbs2,
      uint8_t    resetdisable);
int stkComms_enterProgMode(stkComms_t* comms);
int stkComms_leaveProgMode(stkComms_t* comms);
int stkComms_checkSignature(stkComms_t* comms);
int stkComms_loadAddress(stkComms_t* comms, uint16_t address);
int stkComms_progHexFile(stkComms_t* comms, const char* filename);
int stkComms_checkFlash(stkComms_t* comms, const char* filename);
int stkComms_checkPage(stkComms_t* comms, hexFile_t* hexfile, uint16_t address, uint16_t size);
int stkComms_progPage(stkComms_t* comms, uint8_t* data, uint16_t size);
int stkComms_progFuses(stkComms_t* comms);
int stkComms_readData(stkComms_t* comms, uint16_t address, uint8_t *byte);
int stkComms_writeData(stkComms_t* comms, uint16_t address, uint8_t byte);
int stkComms_universal(stkComms_t* comms, uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);
int stkComms_setdtr (stkComms_t* comms, int on);

int stkComms_sendBytes(stkComms_t* comms, void* buf, size_t len);
int stkComms_recvBytes(stkComms_t* comms, uint8_t* buf, size_t expectedBytes, size_t size);
int stkComms_recvBytes2(stkComms_t* comms, uint8_t* buf, size_t size);

void libstkcomms_is_present(void);


hexFile_t* hexFile_new();
int hexFile_init(hexFile_t* hf);
int hexFile_init2(hexFile_t* hf, const char* filename);
int hexFile_destroy(hexFile_t* hf);
uint8_t hexFile_getByte(hexFile_t* hf, int index);
int hexFile_loadFile(hexFile_t* hf, const char* filename);
int hexFile_len(hexFile_t* hf); 
void hexFile_realloc(hexFile_t* hf);
void hexFile_parseLine(hexFile_t* hf, const char* line);
}

#endif
