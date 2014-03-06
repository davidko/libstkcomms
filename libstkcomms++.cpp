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

#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#if defined (_WIN32)
#include <winsock2.h>
#ifdef ENABLE_BLUETOOTH
#include <Ws2bth.h>
#endif
#elif defined (_MSYS)
#include <winsock2.h>
#else
#include <sys/ioctl.h>
#endif
#include "libstkcomms.hpp"
#include "libstkcomms.h"
#include "command.h"
#include "thread_macros.h"

//#define DEBUG

#ifdef DEBUG
#define THROW throw
#else
#define THROW
#endif

CStkComms::CStkComms()
{
  _comms = stkComms_new();
  stkComms_init(_comms);
}

CStkComms::~CStkComms()
{
  stkComms_destroy(_comms);
}

int CStkComms::connect(const char addr[])
{
#ifndef __MACH__
  return stkComms_connect(_comms, addr);
#else
  return stkComms_connectWithAddressTTY(_comms, addr);
#endif
}

int CStkComms::connectWithTTY(const char* ttyfilename)
{
  return stkComms_connectWithTTY(_comms, ttyfilename);
}

int CStkComms::disconnect()
{
  return stkComms_disconnect(_comms);
}

int CStkComms::setSocket(int socket)
{
  return stkComms_setSocket(_comms, socket);
}

int CStkComms::programAll(const char* hexFileName)
{
  if(handshake()) {
    THROW;
    return -1;
  }
  if(setDevice()) {
    THROW;
    return -1;
  }
  if(setDeviceExt()) {
    THROW;
    return -1;
  }
  if(enterProgMode()) {
    THROW;
    return -1;
  }
  if(checkSignature()) {
    THROW;
    return -1;
  }
  /*
  if(progFuses()) {
    THROW;
    return -1;
  }
  */
  if(progHexFile(hexFileName)) {
    THROW;
    return -1;
  }
  if(checkFlash(hexFileName)) {
    THROW;
    return -1;
  }
  if(leaveProgMode()) {
    THROW;
    return -1;
  }
	stkComms_setProgress(_comms, 1.0);
	stkComms_setProgramComplete(_comms, 1);
  return 0;  
}

int CStkComms::programAll(const char* hexFileName, int hwRev)
{
  if(handshake()) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  if(setDevice()) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  if(setDeviceExt()) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  if(enterProgMode()) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  if(checkSignature()) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  /*
  if(writeData(0x11, hwRev)) {
    THROW;
    return -1;
  }
  */
  if(progFuses()) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  if(progHexFile(hexFileName)) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  if(checkFlash(hexFileName)) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
  if(leaveProgMode()) {
    THROW;
    printf("programming failed %s:%d\n", __FILE__, __LINE__);
    return -1;
  }
	stkComms_setProgress(_comms, 1.1);
	stkComms_setProgramComplete(_comms, 1);
  return 0;  
}

struct programAllThreadArg{
  const char* hexFileName;
  CStkComms* stkComms;
  int hwRev;
};

void* programAllThread(void* arg)
{
  programAllThreadArg *a = (programAllThreadArg*)arg;
  if(a->hwRev == 0) {
    a->stkComms->programAll(a->hexFileName);
  } else {
    a->stkComms->programAll(a->hexFileName, a->hwRev);
  }
  return NULL;
}

int CStkComms::programAllAsync(const char* hexFileName)
{
  THREAD_T thread;
  struct programAllThreadArg *a;
  a = (struct programAllThreadArg*)malloc(sizeof(struct programAllThreadArg));
  a->hexFileName = hexFileName;
  a->stkComms = this;
  a->hwRev = 0;
	stkComms_setProgress(_comms, 0.01);
  THREAD_CREATE(&thread, programAllThread, a);
  return 0;
}

int CStkComms::programAllAsync(const char* hexFileName, int hwRev)
{
  THREAD_T thread;
  struct programAllThreadArg *a;
  a = (struct programAllThreadArg*)malloc(sizeof(struct programAllThreadArg));
  a->hexFileName = hexFileName;
  a->stkComms = this;
  a->hwRev = hwRev;
	stkComms_setProgress(_comms, 0.01);
  THREAD_CREATE(&thread, programAllThread, a);
  return 0;
}

double CStkComms::getProgress()
{
  return stkComms_getProgress(_comms);
}

int CStkComms::isProgramComplete() {
  return stkComms_isProgramComplete(_comms);
}

int CStkComms::handshake()
{
  return stkComms_handshake(_comms);
}

int CStkComms::setDevice(
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
      uint8_t     flashsize1)
{
  return stkComms_setDevice(_comms,
      DeviceCode,
      Revision,
      progtype,
      parmode,
      polling,
      selftimed,
      lockbytes,
      fusebytes,
      flashpollval1,
      flashpollval2,
      eeprompollval1,
      eeprompollval2,
      pagesizehigh,
      pagesizelow,
      eepromsizehigh,
      eepromsizelow,
      flashsize4,
      flashsize3,
      flashsize2,
      flashsize1);
}

int CStkComms::setDeviceExt(
      uint8_t    commandsize,
      uint8_t    eeprompagesize,
      uint8_t    signalpagel,
      uint8_t    signalbs2,
      uint8_t    resetdisable)
{
  return stkComms_setDeviceExt( _comms,
      commandsize,
      eeprompagesize,
      signalpagel,
      signalbs2,
      resetdisable);
}

int CStkComms::enterProgMode()
{
  return stkComms_enterProgMode(_comms);
}

int CStkComms::leaveProgMode()
{
  return stkComms_leaveProgMode(_comms);
}

int CStkComms::checkSignature()
{
  return stkComms_checkSignature(_comms);
}

int CStkComms::loadAddress(uint16_t address)
{
  return stkComms_loadAddress(_comms, address);
}

int CStkComms::progHexFile(const char* filename)
{
  return stkComms_progHexFile(_comms, filename);
}

int CStkComms::checkFlash(const char* filename)
{
  return stkComms_checkFlash(_comms, filename);
}

int CStkComms::checkPage(CHexFile* hexfile, uint16_t address, uint16_t size)
{
  return stkComms_checkPage(_comms, hexfile->_hf, address, size);
}

int CStkComms::progPage(uint8_t* data, uint16_t size)
{
  return stkComms_progPage(_comms, data, size);
}

int CStkComms::progFuses()
{
  return stkComms_progFuses(_comms);
}

int CStkComms::readData(uint16_t address, uint8_t *byte)
{
  return stkComms_readData(_comms, address, byte);
}

int CStkComms::writeData(uint16_t address, uint8_t byte)
{
  return stkComms_writeData(_comms, address, byte);
}

int CStkComms::universal(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4)
{
  return stkComms_universal(_comms,
      byte1,
      byte2,
      byte3,
      byte4);
}

int CStkComms::sendBytes(void* buf, size_t len)
{
  return stkComms_sendBytes(_comms, buf, len);
}

int CStkComms::recvBytes(uint8_t* buf, size_t expectedBytes, size_t size)
{
  return stkComms_recvBytes(_comms, buf, expectedBytes, size);
}

int CStkComms::recvBytes(uint8_t* buf, size_t size)
{
  return stkComms_recvBytes2(_comms, buf, size);
}

int CStkComms::setdtr (int on)
{
  return stkComms_setdtr(_comms, on);
} 

CHexFile::CHexFile()
{
  _hf = hexFile_new();
  hexFile_init(_hf);
}

CHexFile::CHexFile(const char* filename)
{
  _hf = hexFile_new();
  hexFile_init2(_hf, filename);
}

CHexFile::~CHexFile()
{
  hexFile_destroy(_hf);
}

uint8_t CHexFile::getByte(int index)
{
  return hexFile_getByte(_hf, index);
}

int CHexFile::loadFile(const char* filename)
{
  return hexFile_loadFile(_hf, filename);
}

int CHexFile::len()
{
  return hexFile_len(_hf);
}

void CHexFile::realloc()
{
  return hexFile_realloc(_hf);
}

void CHexFile::parseLine(const char* line)
{
  hexFile_parseLine(_hf, line);
}

void libstkcomms_is_present(void)
{
}

#ifdef _WIN32
void baswap(bdaddr_t *dst, const bdaddr_t *src)
{
	register unsigned char *d = (unsigned char *) dst;
	register const unsigned char *s = (const unsigned char *) src;
	register int i;

	for (i = 0; i < 6; i++)
		d[i] = s[5-i];
}

int str2ba(const char *str, bdaddr_t *ba)
{
	UINT8 b[6];
	const char *ptr = str;
	int i;

	for (i = 0; i < 6; i++) {
		b[i] = (UINT8) strtol(ptr, NULL, 16);
		if (i != 5 && !(ptr = strchr(ptr, ':')))
			ptr = ":00:00:00:00:00";
		ptr++;
	}

	baswap(ba, (bdaddr_t *) b);

	return 0;
}
#endif
