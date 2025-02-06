/*------------------------------------------------------------------------------
* dgs.c : Qzss correction receiver QZS-6C dependent functions
*
*          Written by DATAGNSS, 2024/12/05
*          www.datagnss.com dev@datagnss.com
*
* reference :
*     QZS-6C receiver wiki, https://docs.datagnss.com/gnss/qzs6c_l6_receiver
*
* version : $Revision: 0.5 $ $Date: 2024/11/08 00:05:05 $
* history : 2024/11/08 1.0  new
*-----------------------------------------------------------------------------*/
#include "rtklib.h"

#define DGS_SYNC1   0xF1        /* dgs message sync code 1 */
#define DGS_SYNC2   0xD9        /* dgs message sync code 2 */

#define ID_QZSSL6   0x0210      /* dgs message id: qzss l6 message */

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((uint8_t *)(p)))
#define S1(p) (*((int8_t  *)(p)))
static uint16_t U2(uint8_t *p) {uint16_t u; memcpy(&u,p,2); return u;}

enum {
    QZSSL6_ALL_GOOD = 0x0,
    QZSSL6_RS_FAILED = 0x1,
    QZSSL6_WEEK_NOT_CONFIRM = 0x2,
    QZSSL6_TOW_NOT_CONFIRM = 0x4
};

/* checksum ------------------------------------------------------------------*/
static int checksum(uint8_t *buff, int len)
{
    uint8_t cka=0,ckb=0;
    int i;
    
    for (i=2;i<len-2;i++) {
        cka+=buff[i]; ckb+=cka;
    }
    return cka==buff[len-2]&&ckb==buff[len-1];
}


/* decode dgs qzss l6 message ----------------------------------------------------------------------------------------------+
+--------------------------------------------------------------------------------------------------------------------------------+
+ sync code ---- type ---- payload length - prn -- freqid - len_data(N+2) - gpsw ---- gpst --- snr ---- flag --- msgs -- ck1_2 --+
+ 0xF1 0xD9 - 0x02 0x73 ---- 2 bytes ---- 1 byte - 1 byte -- 2 bytes ---- 2 bytes - 4 bytes - 1 byte - 1 byte -- N*4 -- 2 bytes -+
+--------------------------------------------------------------------------------------------------------------------------------+
*/
static int decode_qzssl6(raw_t *raw, rtcm_t *rtcm)
{    
    uint8_t *p=raw->buff+6; /* skip 6 bytes, sync code (2), type(2), length(2) */
    int ret=0;
    uint16_t payload_len = U2(raw->buff+4); /* payload length, little-endian */

    /* prn(2), freqid(1), len_data(1), gpsw(2), gpst(4), snr(1), flag(1) */
    uint8_t freqid,len_data_N,snr,flag;
    uint16_t prn;
    uint16_t gpsw; /* GPS Week Number, big-endian */
    uint32_t gpst; /* GPS Time of Week, big-endian */

    trace(2,"decode_qzssl6: raw_len=%d payload_len=%d\n",raw->len,payload_len);
    
    if (raw->len<272) {
        trace(2,"dgs qzssl6 length error: len=%d\n",raw->len);
        return -1;
    }
    
    prn = U2(p) - 700;  /* SVN = PRN - 700 */
    freqid = U1(p+2);
    len_data_N = U1(p+3) - 2;
    gpsw = (p[4] << 8) | p[5];  /* GPS Week Number (big-endian) */
    gpst = (p[6] << 24) | (p[7] << 16) | (p[8] << 8) | p[9];  /* GPS Time of Week (big-endian) */
    snr = p[10];  /* SNR (big-endian) */
    flag = p[11]; /* Flag (big-endian) */

    trace(2,"decode_qzssl6: prn=%2d freqid=%d len_data(N)=%d gpsw=%d gpst=%d snr=%d flag=%d\n",
          prn, freqid, len_data_N, gpsw, gpst, snr, flag);
#if 0
    printf("prn=%2d freqid=%d len_data(N)=%d gpsw=%d gpst=%d snr=%d flag=%d(%s)\n",
          prn, freqid, len_data_N, gpsw, gpst, snr, flag, flag==0?"GOOD":flag==1?"RS FAILED":flag==2?"WEEK NOT CONFIRM":flag==4?"TOW NOT CONFIRM":"UNKNOWN");
#endif

    if (raw->outtype) {
        sprintf(raw->msgtype,"DGS QZSSL6  (%4d): prn=%2d freqid=%d len_data(N)=%d gpsw=%d gpst=%d snr=%d flag=%d",
            raw->len,prn,freqid,len_data_N,gpsw,gpst,snr,flag);
    }
    
    /* only process data with no errors */
    ret=0;
    if (flag==0) {
        memcpy(rtcm->buff,p+12, len_data_N *4);
                
        /* call mdccssr module to decode QZSS L6 data */
        ret=decode_qzss_l6emsg(rtcm);
        
        if (ret<0) {
            trace(2,"decode qzss l6e message error: prn=%2d\n",prn);
        }
    } else {
        trace(2,"decode qzss l6e message error: flag=%d\n",flag);
    }
    return ret;
}

/* decode dgs raw message ----------------------------------------------*/
static int decode_dgs(raw_t *raw, rtcm_t *rtcm)
{
    int type=(U1(raw->buff+2)<<8)+U1(raw->buff+3);
    int payload_len;
    
    trace(3,"decode_dgs: type=%04x len=%d\n",type,raw->len);
    
    /* checksum */
    if (!checksum(raw->buff,raw->len)) {
        trace(2,"dgs checksum error: type=%04x len=%d\n",type,raw->len);
        return -1;
    }

    payload_len = U2(raw->buff+4); /* payload length, little-endian */

#if 0
    printf("dgs type=%04x raw_len=%d payload_len=%d\n",type,raw->len,payload_len);
#endif

    switch (type) {
        case ID_QZSSL6: return decode_qzssl6(raw,rtcm);
    }
    
    if (raw->outtype) {
        sprintf(raw->msgtype,"DGS 0x%02X 0x%02X (%4d)",type>>8,type&0xFF,
                raw->len);
    }
    return 0;
}

/* sync code -----------------------------------------------------------------*/
static int sync_dgs(uint8_t *buff, uint8_t data)
{
    buff[0]=buff[1]; buff[1]=data;
    return buff[0]==DGS_SYNC1&&buff[1]==DGS_SYNC2;
}

/* input dgs raw message from stream -------------------------------------*/
extern int input_dgs(raw_t *raw, rtcm_t *rtcm, uint8_t data)
{
    trace(5,"input_dgs: data=%02x\n",data);
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        if (!sync_dgs(raw->buff,data)) return 0;
        raw->nbyte=2;
        return 0;
    }
    raw->buff[raw->nbyte++]=data;
    
    if (raw->nbyte==6) {
        if ((raw->len=U2(raw->buff+4)+8)>MAXRAWLEN) {
            trace(2,"dgs length error: len=%d\n",raw->len);
            raw->nbyte=0;
            return -1;
        }
    }
    if (raw->nbyte<6||raw->nbyte<raw->len) return 0;
    raw->nbyte=0;
    
    /* decode dgs raw message */
    return decode_dgs(raw,rtcm);
}

/* input dgs raw message from file --------------------------------------*/
extern int input_dgs_f(raw_t *raw, rtcm_t *rtcm, FILE *fp)
{
    int i,data;
    
    trace(4,"input_dgs_f:\n");
    
    /* synchronize frame */
    if (raw->nbyte==0) {
        for (i=0;;i++) {
            if ((data=fgetc(fp))==EOF) return -2;
            if (sync_dgs(raw->buff,(uint8_t)data)) break;
            if (i>=4096) return 0;
        }
    }
    if (fread(raw->buff+2,1,4,fp)<4) return -2;
    raw->nbyte=6;
    
    if ((raw->len=U2(raw->buff+4)+8)>MAXRAWLEN) {
        trace(2,"dgs length error: len=%d\n",raw->len);
        raw->nbyte=0;
        return -1;
    }
    if (fread(raw->buff+6,1,raw->len-6,fp)<(size_t)(raw->len-6)) return -2;
    raw->nbyte=0;
    
    /* decode dgs raw message */
    return decode_dgs(raw,rtcm);
}

