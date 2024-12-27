#define  _CRT_SECURE_NO_WARNINGS
#define  _CRT_SECURE_NO_DEPRECATE
#pragma  warning(disable : 4115)

#include <stdio.h>
#include <winsock2.h>
#include "phidget21.h"
#include "def.h"
#include "temp.h"
#include "ExternDef.h"
#include "Dome.h"
#include "SocketHirez.h"
#include <windows.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <conio.h>
#include <process.h>
#include <tchar.h>

#pragma comment(lib,"ws2_32.lib") //Winsock Library

//char *IPSrvHirez    = "192.168.2.161";  //WatchLT1

#ifdef C14
 char *IPDevHirez[2] = { STRGX(CONTX(192.168.2.,OCT4HIREZ0)),    // RA
                         STRGX(CONTX(192.168.2.,OCT4HIREZ1)) };  // Dec
 unsigned short portHirez[2] = { PORTHIREZ0, PORTHIREZ1 };
 char *IPSrvHirez = "192.168.2.155";
#endif
#ifdef RC16
 char *IPDevHirez[2] = { STRGX(CONTX(192.168.2.,OCT4HIREZ2)),    // RA
                         STRGX(CONTX(192.168.2.,OCT4HIREZ3)) };  // Dec
 unsigned short portHirez[2] = { PORTHIREZ2, PORTHIREZ3 };
 char *IPSrvHirez = "192.168.2.154";
#endif 


SOCKET sockHirez[2];
WSADATA wsaHirez[2];
struct sockaddr_in SrvHirezSock[2], DevHirezSock[2];
int serverlenHirez[2] = { sizeof SrvHirezSock[0], sizeof SrvHirezSock[0]};
int clientlenHirez[2] = { sizeof DevHirezSock[1], sizeof DevHirezSock[1]};
HANDLE SocketThreadHirez[2]    = {0};      // Thread handle
int    SocketThreadNrHirez[2]  = {1, 1};   // Number of threads started
int    SocketConnectedHirez[2] = {-1, -1}; // Socket connected
extern int   ExitRequest;

extern t_Motor Drive[]; //, FocusM;

t_SocketSrvHirez    SrvHirez[2];
t_SocketDevHirez    DevHirez[2];
t_SrvDevHirezStatus SrvHirezStatus[2] = {0, 0};
t_SrvDevHirezStatus DevHirezStatus[2] = {0, 0};
int              SrvHirezLatency[2];
int              DevHirezEthCommTimeout[2] = { ETH_TIMEOUT, ETH_TIMEOUT };

void SocketInitHirez(int HIREZ)
{
  static int Initialized[2] = { 0, 0 };
  ULONG NonBlock = 1;
  int Status = 0;
  
  if (HIREZ == HIREZ_RA) 
    Log("SocketInitHirez RA:", LOG);
  else
    Log("SocketInitHirez Dec:", LOG);

  if (!Initialized[HIREZ])
  {
    Log(" Initialising Winsock...", LOG);
    if (WSAStartup(MAKEWORD(2,2),&wsaHirez[HIREZ]) != 0)
    {
      Log(" Failed", LOG); //. Error Code : %d",WSAGetLastError());
      return;
    }
  
//    Log(" Initialised", LOG);  Quiet it success
  
    //Create a socket
    if ((sockHirez[HIREZ] = socket(AF_INET , SOCK_DGRAM, 0)) == INVALID_SOCKET)  // SOCK_DGRAM SOCK_STREAM IPPROTO_UDP
    {
      Log(" Could not create socket", LOG);// : %d" , WSAGetLastError());
      return;
    }

//    Log(" Socket created", LOG); Quiet it success

    memset((char *) &SrvHirezSock[HIREZ], 0, sizeof(SrvHirezSock[HIREZ]));
    SrvHirezSock[HIREZ].sin_addr.s_addr = inet_addr(IPSrvHirez);
    SrvHirezSock[HIREZ].sin_family      = AF_INET;
    SrvHirezSock[HIREZ].sin_port        = htons(portHirez[HIREZ]);
    memset((char *) &DevHirezSock[HIREZ], 0, sizeof(DevHirezSock[HIREZ]));
    DevHirezSock[HIREZ].sin_addr.s_addr = inet_addr(IPDevHirez[HIREZ]);
    DevHirezSock[HIREZ].sin_family      = AF_INET;
    DevHirezSock[HIREZ].sin_port        = htons(portHirez[HIREZ]);
    if (bind(sockHirez[HIREZ], (struct sockaddr *)&SrvHirezSock[HIREZ], sizeof(SrvHirezSock[HIREZ])) == SOCKET_ERROR)
    {
      //int Error = GetLastError();
      Log(" Could not bind socket", LOG);
      return;
    }
//  Log(" Socket bind success", LOG);  Quiet it success

    // Set to non blocking
    Status = ioctlsocket(sockHirez[HIREZ], FIONBIO, &NonBlock);

    SocketConnectedHirez[HIREZ] = 0;

    if (HIREZ == 0)
      SocketThreadHirez[HIREZ] = (HANDLE)_beginthread(SocketProcHirez0, 0, &SocketThreadNrHirez[HIREZ]);
    else
      SocketThreadHirez[HIREZ] = (HANDLE)_beginthread(SocketProcHirez1, 0, &SocketThreadNrHirez[HIREZ]);
    Initialized[HIREZ]  = 1;
  }
  else
  {
    Initialized[0] = 0;
    closesocket(sockHirez[HIREZ]);
    WSACleanup();
  }
}


void SocketExitHirez(int HIREZ)
{
//SocketThreadNrHirez[HIREZ] = 0;
  if (SocketThreadHirez[HIREZ])
  {
    WaitForSingleObject(SocketThreadHirez[HIREZ], INFINITE);
//  CloseHandle(SocketThreadHirez[HIREZ]);
  }
}


void SocketProcHirez0(void *ThreadNr)
{
  static int Status;
  int HIREZ = HIREZ_RA;

  while (ThreadNr && *(int *)ThreadNr )
  {
    if (DevHirezEthCommTimeout[HIREZ] > 0) 
    {
      DevHirezEthCommTimeout[HIREZ]--;
      SrvHirez[HIREZ].Status.EthCommFail = 1;
      *(unsigned int *)&DevHirez[HIREZ].Status = UINT_MAX;
    }
    else
    {
      DevHirezEthCommTimeout[HIREZ] = 0;
      SrvHirez[HIREZ].Status.EthCommFail = 0;
    }

    if (ExitRequest)
      SocketExitHirez(HIREZ);
    else
    // Process DevHirezSock[HIREZ] connections
    {
      t_SocketDevHirez EthRcvBuf;

      Status = recvfrom(sockHirez[HIREZ], (char *)&EthRcvBuf, sizeof EthRcvBuf, 0, (struct sockaddr *) &DevHirezSock[HIREZ], &clientlenHirez[HIREZ]);
      if (Status == SOCKET_ERROR)
        Status = WSAGetLastError();
      else
      {
        SocketConnectedHirez[HIREZ] = 1;
        
        // Chksum of data from device
        unsigned int DevChkSum = 0;
        unsigned char *ucptr = (unsigned char *)&EthRcvBuf;
        for (unsigned int n=0; n<(sizeof (t_SocketDevHirez) - sizeof (DevHirez[HIREZ].ChkSum)); n++)
        {
          DevChkSum += *ucptr;
          ucptr++;
        }
        SrvHirezStatus[HIREZ].ChkSumFail = (DevChkSum != EthRcvBuf.ChkSum);
        DevHirezStatus[HIREZ] = DevHirez[HIREZ].Status;

        if (SrvHirezStatus[HIREZ].ChkSumFail == 0)
        {
          memcpy((char *)&DevHirez[HIREZ], &EthRcvBuf, sizeof (t_SocketDevHirez));
          DevHirezEthCommTimeout[HIREZ] = 0;
        }
        else
          DevHirezEthCommTimeout[HIREZ] = ETH_TIMEOUT;
      }

      static int subTx = 0;  // Don't inundate slower client
      if (--subTx < 0)
      {
        subTx = 2;

        SrvHirez[HIREZ].HB++;
        SrvHirez[HIREZ].HB &= 0xff;
        SrvHirezLatency[HIREZ] = SrvHirez[HIREZ].HB - DevHirez[HIREZ].HBSrv;  // Latency
        if (SrvHirezLatency[HIREZ] < 0) SrvHirezLatency[HIREZ] += 256;
        if (SrvHirezLatency[HIREZ] > 5) DevHirezEthCommTimeout[HIREZ] = ETH_TIMEOUT;

        // Chksum for data to device
        SrvHirez[HIREZ].ChkSum = 0;
        unsigned char *ucptr = (unsigned char *)&SrvHirez[HIREZ];
        for (unsigned int n=0; n<(sizeof (t_SocketSrvHirez) - sizeof (SrvHirez[HIREZ].ChkSum)); n++)
        {
          SrvHirez[HIREZ].ChkSum += *ucptr;
          ucptr++;
        }
        Status = sendto(sockHirez[HIREZ] , (char *)&SrvHirez[HIREZ], sizeof SrvHirez[HIREZ], 0, (struct sockaddr *) &DevHirezSock[HIREZ], sizeof(DevHirezSock[HIREZ]));
        if (Status == SOCKET_ERROR)
          Status = WSAGetLastError();
      }

//DSpare1 = (float)SrvHirezLatency[HIREZ];
DSpare1 = (float)DevHirez[HIREZ].dt[0];
//DSpare2 = (float)DevHirez[HIREZ].dx[0];
DSpare3 = (float)DevHirez[HIREZ].dy[0];

DSpare5 = (float)DevHirez[HIREZ].dt[1];
//DSpare6 = (float)DevHirez[HIREZ].dx[1];
DSpare7 = (float)DevHirez[HIREZ].dy[1];

#define PIX2ARCSEC 1.f/(4.75f*2.f*3.1415f*25.4f*1000.f/360.f/60.f/60.f*1.17f)
DSpare4 = ((DevHirez[HIREZ].dt[0] != 0.f) ? (DevHirez[HIREZ].dy[0]/DevHirez[HIREZ].dt[0]) : 0.f) * PIX2ARCSEC;
DSpare8 = ((DevHirez[HIREZ].dt[1] != 0.f) ? (DevHirez[HIREZ].dy[1]/DevHirez[HIREZ].dt[1]) : 0.f) * PIX2ARCSEC;

//DSpare4 = (float)DevHirez[HIREZ].xi;
//DSpare4 = (float)DevHirez[HIREZ].yi;
#define ADJK (15.f/12.7f/.75f/1.07f)
extern double Ha;
if (Drive[0].State == DRIVE_OFF)
{
  DSpare2 = (float)Ha - DSpare4 * PIX2ARCSEC / 3600.f * ADJK;
}
DSpare6 = DSpare2 + -DSpare4  * PIX2ARCSEC / 3600.f * ADJK;
//DSpare5 = (float)Ha - DSpare6;
    }
    Sleep(SRV_PERIOD);
  }
  _endthread();
}

// TODO THIS IS THE SAME AS FOR RA SENSOR, COMBINE! since this one was not updated
void SocketProcHirez1(void *ThreadNr)
{
  static int Status;
  int IO = HIREZ_DEC;

  while (ThreadNr && *(int *)ThreadNr )
  {
    if (DevHirezEthCommTimeout[IO] > 0) 
    {
      DevHirezEthCommTimeout[IO]--;
      SrvHirez[IO].Status.EthCommFail = 1;
      *(unsigned int *)&DevHirez[IO].Status = UINT_MAX;
    }
    else
    {
      DevHirezEthCommTimeout[IO] = 0;
      SrvHirez[IO].Status.EthCommFail = 0;
    }

    if (ExitRequest)
    {
      SocketExitHirez(IO);
    }
    else
    {
      t_SocketDevHirez EthRcvBuf;

      Status = recvfrom(sockHirez[IO], (char *)&EthRcvBuf, sizeof EthRcvBuf, 0, (struct sockaddr *) &DevHirezSock[IO], &clientlenHirez[IO]);
      if (Status == SOCKET_ERROR)
        Status = WSAGetLastError();
      else
      {
        SocketConnectedHirez[IO] = 1;
        
        // Chksum of data from device
        unsigned int DevChkSum = 0;
        unsigned char *ucptr = (unsigned char *)&EthRcvBuf;
        for (unsigned int n=0; n<(sizeof (t_SocketDevHirez) - sizeof (DevHirez[IO].ChkSum)); n++)
        {
          DevChkSum += *ucptr;
          ucptr++;
        }
        SrvHirezStatus[IO].ChkSumFail = (DevChkSum != EthRcvBuf.ChkSum);
        DevHirezStatus[IO] = DevHirez[IO].Status;

        if (SrvHirezStatus[IO].ChkSumFail == 0)
        {
          memcpy((char *)&DevHirez[IO], &EthRcvBuf, sizeof (t_SocketDevHirez));
          DevHirezEthCommTimeout[IO] = 0;
        }
        else
          DevHirezEthCommTimeout[IO] = ETH_TIMEOUT;
      }

      SrvHirez[IO].HB++;
      SrvHirez[IO].HB &= 0xff;
      SrvHirezLatency[IO] = SrvHirez[IO].HB - DevHirez[IO].HBSrv;  // Latency
      if (SrvHirezLatency[IO] < 0) SrvHirezLatency[IO] += 256;
      if (SrvHirezLatency[IO] > 5) DevHirezEthCommTimeout[IO] = ETH_TIMEOUT;

      // Mount motor commands
//    SrvHirez[IO].MotCmd[MOT_RA]    = Drive[0].Rate;
//    SrvHirez[IO].MotCmd[MOT_DEC]   = Drive[1].Rate;

      // Chksum for data to device
      SrvHirez[IO].ChkSum = 0;
      unsigned char *ucptr = (unsigned char *)&SrvHirez[IO];
      for (unsigned int n=0; n<(sizeof (t_SocketSrvHirez) - sizeof (SrvHirez[IO].ChkSum)); n++)
      {
        SrvHirez[IO].ChkSum += *ucptr;
        ucptr++;
      }

      static int subTx = 0;  // Don't inundate slower client
      if (--subTx < 0)
      {
        subTx = 5;
        Status = sendto(sockHirez[IO] , (char *)&SrvHirez[IO], sizeof SrvHirez[IO], 0, (struct sockaddr *) &DevHirezSock[IO], sizeof(DevHirezSock[IO]));
        if (Status == SOCKET_ERROR)
          Status = WSAGetLastError();
      }

//  DSpare6 = (float)((DevHirez[IO].TamEnc[0][4]<<16) + (DevHirez[IO].TamEnc[0][3]<<8) + (DevHirez[IO].TamEnc[0][2]<<0));
//  DSpare7 = (float)((DevHirez[IO].TamEnc[1][4]<<16) + (DevHirez[IO].TamEnc[1][3]<<8) + (DevHirez[IO].TamEnc[1][2]<<0));
    }
    Sleep(SRV_PERIOD);
  }
  _endthread();
}

