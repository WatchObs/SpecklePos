// Hirez position sensor over UDP

#ifndef SRV_PERIOD
#define SRV_PERIOD  100                // Server period in millisecond
#define ETH_TIMEOUT (1000/SRV_PERIOD)  // Cycles to time out (ostensibly a second)
#endif

// Hirez sensor controller indexing
#define HIREZ_RA  0
#define HIREZ_DEC 1

#define C14_HIREZ  0
#define RC16_HIREZ 1

typedef struct t_SrvDevHirezStatusStruct
{
  int ChkSumFail  : 1;
  int EthCommFail : 1;
  int pad         :30;
} t_SrvDevHirezStatus;

typedef struct t_SocketSrvHirezstruct
{
  int                 HB;
  t_SrvDevHirezStatus Status;
  int                 DriveRate;
  unsigned int        ChkSum;
} t_SocketSrvHirez;

typedef struct t_SocketDevHirezstruct
{
  int                 HB;
  int                 HBSrv;
  t_SrvDevHirezStatus Status;
  float               TimeStamp;
  float               dt;
  float               dx;
  float               dy;
  float               xi;
  float               yi;
  unsigned int        ChkSum;
} t_SocketDevHirez;

#define STRG(A) #A
#define STRGX(A) STRG(A)
#define CONT(A,B) A ## B
#define CONTX(A,B) CONT(A,B)

#define  PORTHIREZ0 50020 // C14  RA  Hirez port
#define  PORTHIREZ1 50021 // C14  DEC Hirez port
#define  PORTHIREZ2 50022 // RC16 RA  Hirez port
#define  PORTHIREZ3 50023 // RC16 DEC Hirez port
#define  OCT4HIREZ0 204   // Last octet of C14  RA  Hirez
#define  OCT4HIREZ1 205   // Last octet of C14  DEC Hirez
//#define  OCT4HIREZ2 206   // Last octet of RC16 RA  Hirez
#define  OCT4HIREZ2 117   // Last octet of RC16 RA  Hirez
#define  OCT4HIREZ3 207   // Last octet of RC16 DEC Hirez
