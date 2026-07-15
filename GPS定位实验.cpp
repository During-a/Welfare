
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"

/* ========================== 调试串口 UART2 ================================ */
void UART2_Init(uint32_t bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART2, ENABLE);
}

int fputc(int ch, FILE *f)
{
    USART_SendData(USART2, (uint8_t)ch);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    return ch;
}

void USART2_IRQHandler(void)
{
    if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) {
        uint8_t ch = USART_ReceiveData(USART2);
        USART_SendData(USART2, ch);
    }
}

/* ========================== GPS 模块 (UART4) ============================= */
#define GPS_USART  UART4    // 关键修正：使用 UART4

#define GPS_BUF_SIZE  500
uint8_t g_GPS_RxBuf[GPS_BUF_SIZE];
volatile uint16_t g_GPS_RxBufLen = 0;
volatile uint8_t  g_GPS_RxDataOK = 0;

void GPS_Init(uint32_t baud)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);   // 修正：AHB1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = baud;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(GPS_USART, &USART_InitStructure);

    USART_ITConfig(GPS_USART, USART_IT_RXNE, ENABLE);
    USART_ITConfig(GPS_USART, USART_IT_IDLE, ENABLE);
    USART_Cmd(GPS_USART, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(GPS_USART, USART_IT_RXNE) != RESET) {
        if (g_GPS_RxDataOK == 0) {
            if (g_GPS_RxBufLen < GPS_BUF_SIZE - 1) {
                g_GPS_RxBuf[g_GPS_RxBufLen++] = USART_ReceiveData(GPS_USART);
            }
        } else {
            (void)USART_ReceiveData(GPS_USART);
        }
        USART_ClearITPendingBit(GPS_USART, USART_IT_RXNE);
    }

    if (USART_GetITStatus(GPS_USART, USART_IT_IDLE) == SET) {
        volatile uint32_t tmp = UART4->SR;   // 使用 UART4
        tmp = UART4->DR;
        (void)tmp;
        g_GPS_RxDataOK = 1;
    }
}

/* ========================== NMEA 解析器 ================================== */
#define ID_GGA   "GNGGA"
#define ID_RMC   "GNRMC"
#define ID_GLL   "GNGLL"
#define ID_GSA   "GNGSA"
#define ID_GSV   "GPGSV"
#define ID_VTG   "GNVTG"

#define ENABLE_GGA  1
#define ENABLE_RMC  1
#define ENABLE_GLL  1
#define ENABLE_GSA  1
#define ENABLE_GSV  1
#define ENABLE_VTG  1

#if ENABLE_GGA
typedef struct {
    char utc[11];
    double lat;
    char lat_dir;
    double lon;
    char lon_dir;
    unsigned char quality;
    unsigned char sats;
    double hdop;
    double alt;
    double undulation;
    unsigned char age;
    unsigned short stn_ID;
} GGA;
#endif

#if ENABLE_RMC
typedef struct {
    char utc[11];
    unsigned char status;
    double lat;
    char lat_dir;
    double lon;
    char lon_dir;
    double speed_Kn;
    double track_true;
    char date[7];
    double mag_var;
    char mag_var_dir;
    char mode;
    char nav_status;
} RMC;
#endif

#if ENABLE_GLL
typedef struct {
    double lat;
    char lat_dir;
    double lon;
    char lon_dir;
    char utc[11];
    char status;
} GLL;
#endif

#if ENABLE_GSA
typedef struct {
    char mode_MA;
    char mode_123;
    unsigned short prn[12];
    double pdop;
    double hdop;
    double vdop;
    unsigned char sysid;
} GSA;
#endif

#if ENABLE_GSV
#pragma pack(1)
typedef struct {
    unsigned char prn;
    unsigned char elev;
    unsigned short azimuth;
    unsigned char SNR;
} SAT_INFO;
#pragma pack()

typedef struct {
    unsigned char msgs;
    unsigned char msg;
    unsigned char sats;
    unsigned char sysid;
    SAT_INFO sat_info[36];
} GSV;
#endif

#if ENABLE_VTG
typedef struct {
    double track_true;
    double track_mag;
    double speed_Kn;
    double speed_Km;
    char mode;
} VTG;
#endif

// 全局变量
#if ENABLE_GGA
GGA gga;
#endif
#if ENABLE_RMC
RMC rmc;
#endif
#if ENABLE_GLL
GLL gll;
#endif
#if ENABLE_GSA
GSA gsa;
#endif
#if ENABLE_GSV
GSV gsv;
#endif
#if ENABLE_VTG
VTG vtg;
#endif

// ------------------------- 解析函数实现 -------------------------
#if ENABLE_GGA
void gga_parse(GGA *gga, char *gga_data)
{
    char *p = NULL, *next = NULL;
    unsigned char times = 0;
    if (!gga || !gga_data) return;
    p = gga_data;
    memset(gga, 0, sizeof(GGA));
    while (p) {
        next = strpbrk(p, ",");
        if (next == p) { p = next + 1; times++; continue; }
        if (next == NULL) {
            next = strpbrk(p, "*");
            if (next == NULL) break;
        }
        switch (times) {
            case 1: memcpy(gga->utc, p, (next - p) > 10 ? 10 : (next - p)); gga->utc[next-p] = '\0'; break;
            case 2: gga->lat = strtod(p, NULL); break;
            case 3: gga->lat_dir = p[0]; break;
            case 4: gga->lon = strtod(p, NULL); break;
            case 5: gga->lon_dir = p[0]; break;
            case 6: gga->quality = (unsigned char)strtol(p, NULL, 10); break;
            case 7: gga->sats = (unsigned char)strtol(p, NULL, 10); break;
            case 8: gga->hdop = strtod(p, NULL); break;
            case 9: gga->alt = strtod(p, NULL); break;
            case 10: break;
            case 11: gga->undulation = strtod(p, NULL); break;
            case 12: break;
            case 14: gga->stn_ID = (unsigned short)strtol(p, NULL, 10); break;
            default: break;
        }
        p = next + 1;
        times++;
    }
}

void gga_show(GGA *gga)
{
    printf("=========== GGA DATA:\r\n");
    printf("utc:%s\r\n", gga->utc);
    printf("lat:%lf\r\n", gga->lat);
    printf("lat_dir:%c\r\n", gga->lat_dir);
    printf("lon:%lf\r\n", gga->lon);
    printf("lon_dir:%c\r\n", gga->lon_dir);
    printf("quality:%u\r\n", gga->quality);
    printf("sats:%u\r\n", gga->sats);
    printf("hdop:%lf\r\n", gga->hdop);
    printf("alt:%lf\r\n", gga->alt);
    printf("undulation:%lf\r\n", gga->undulation);
    printf("age:%u\r\n", gga->age);
    printf("stn_ID:%u\r\n", gga->stn_ID);
}
#endif

#if ENABLE_RMC
void rmc_parse(RMC *rmc, char *rmc_data)
{
    char *p = NULL, *next = NULL;
    unsigned char times = 0;
    if (!rmc || !rmc_data) return;
    p = rmc_data;
    memset(rmc, 0, sizeof(RMC));
    while (p) {
        next = strpbrk(p, ",");
        if (next == p) { p = next + 1; times++; continue; }
        if (next == NULL) {
            next = strpbrk(p, "*");
            if (next == NULL) break;
        }
        switch (times) {
            case 1: memcpy(rmc->utc, p, (next-p)>10?10:(next-p)); rmc->utc[next-p]='\0'; break;
            case 2: rmc->status = p[0]; break;
            case 3: rmc->lat = strtod(p, NULL); break;
            case 4: rmc->lat_dir = p[0]; break;
            case 5: rmc->lon = strtod(p, NULL); break;
            case 6: rmc->lon_dir = p[0]; break;
            case 7: rmc->speed_Kn = strtod(p, NULL); break;
            case 8: rmc->track_true = strtod(p, NULL); break;
            case 9: memcpy(rmc->date, p, (next-p)>6?6:(next-p)); rmc->date[next-p]='\0'; break;
            case 10: rmc->mag_var = strtod(p, NULL); break;
            case 11: rmc->mag_var_dir = p[0]; break;
            case 12: rmc->mode = p[0]; break;
            case 13: rmc->nav_status = p[0]; break;
            default: break;
        }
        p = next + 1;
        times++;
    }
}

void rmc_show(RMC *rmc)
{
    printf("=========== RMC DATA:\r\n");
    printf("utc:%s\r\n", rmc->utc);
    printf("status:%c\r\n", rmc->status);
    printf("lat:%lf\r\n", rmc->lat);
    printf("lat_dir:%c\r\n", rmc->lat_dir);
    printf("lon:%lf\r\n", rmc->lon);
    printf("lon_dir:%c\r\n", rmc->lon_dir);
    printf("speed_Kn:%lf\r\n", rmc->speed_Kn);
    printf("track_true:%lf\r\n", rmc->track_true);
    printf("date:%s\r\n", rmc->date);
    printf("mag_var:%lf\r\n", rmc->mag_var);
    printf("mag_var_dir:%c\r\n", rmc->mag_var_dir);
    printf("mode:%c\r\n", rmc->mode);
    printf("nav_status:%c\r\n", rmc->nav_status);
}
#endif

#if ENABLE_GLL
void gll_parse(GLL *gll, char *gll_data)
{
    char *p = NULL, *next = NULL;
    unsigned char times = 0;
    if (!gll || !gll_data) return;
    p = gll_data;
    memset(gll, 0, sizeof(GLL));
    while (p) {
        next = strpbrk(p, ",");
        if (next == p) { p = next + 1; times++; continue; }
        if (next == NULL) {
            next = strpbrk(p, "*");
            if (next == NULL) break;
        }
        switch (times) {
            case 1: gll->lat = strtod(p, NULL); break;
            case 2: gll->lat_dir = p[0]; break;
            case 3: gll->lon = strtod(p, NULL); break;
            case 4: gll->lon_dir = p[0]; break;
            case 5: memcpy(gll->utc, p, (next-p)>10?10:(next-p)); gll->utc[next-p]='\0'; break;
            case 6: gll->status = p[0]; break;
            default: break;
        }
        p = next + 1;
        times++;
    }
}

void gll_show(GLL *gll)
{
    printf("=========== GLL DATA:\r\n");
    printf("lat:%lf\r\n", gll->lat);
    printf("lat_dir:%c\r\n", gll->lat_dir);
    printf("lon:%lf\r\n", gll->lon);
    printf("lon_dir:%c\r\n", gll->lon_dir);
    printf("utc:%s\r\n", gll->utc);
    printf("status:%c\r\n", gll->status);
}
#endif

#if ENABLE_GSA
void gsa_parse(GSA *gsa, char *gsa_data)
{
    char *p = NULL, *next = NULL;
    unsigned char times = 0;
    if (!gsa || !gsa_data) return;
    p = gsa_data;
    memset(gsa, 0, sizeof(GSA));
    while (p) {
        next = strpbrk(p, ",");
        if (next == p) { p = next + 1; times++; continue; }
        if (next == NULL) {
            next = strpbrk(p, "*");
            if (next == NULL) break;
        }
        if (times >= 3 && times <= 14) {
            gsa->prn[times - 3] = (unsigned short)strtol(p, NULL, 10);
        }
        switch (times) {
            case 1: gsa->mode_MA = p[0]; break;
            case 2: gsa->mode_123 = p[0]; break;
            case 15: gsa->pdop = strtod(p, NULL); break;
            case 16: gsa->hdop = strtod(p, NULL); break;
            case 17: gsa->vdop = strtod(p, NULL); break;
            case 18: gsa->sysid = (unsigned char)strtol(p, NULL, 10); break;
            default: break;
        }
        p = next + 1;
        times++;
    }
}

void gsa_show(GSA *gsa)
{
    int i;
    printf("=========== GSA DATA:\r\n");
    printf("mode_MA:%c\r\n", gsa->mode_MA);
    printf("mode_123:%c\r\n", gsa->mode_123);
    for (i = 0; i < 12; i++) {
        printf("prn[%d]:%u\r\n", i, gsa->prn[i]);
    }
    printf("pdop:%lf\r\n", gsa->pdop);
    printf("hdop:%lf\r\n", gsa->hdop);
    printf("vdop:%lf\r\n", gsa->vdop);
    printf("sysid:%u\r\n", gsa->sysid);
}
#endif

#if ENABLE_GSV
void gsv_parse(GSV *gsv, char *gsv_data)
{
    char *p = NULL, *next = NULL;
    unsigned char times = 0;
    unsigned char sat_count = 0;
    unsigned char sat_index = 0;
    if (!gsv || !gsv_data) return;
    p = gsv_data;
    memset(gsv, 0, sizeof(GSV));
    while (p) {
        next = strpbrk(p, ",");
        if (next == p) { p = next + 1; times++; continue; }
        if (next == NULL) {
            next = strpbrk(p, "*");
            if (next == NULL) break;
            gsv->sysid = (unsigned char)strtol(p, NULL, 10);
            break;
        }
        switch (times) {
            case 1: gsv->msgs = (unsigned char)strtol(p, NULL, 10); break;
            case 2: gsv->msg = (unsigned char)strtol(p, NULL, 10); break;
            case 3: gsv->sats = (unsigned char)strtol(p, NULL, 10); break;
            case 4: case 5: case 6: case 7:
                if (gsv->msg > 0) {
                    sat_index = (gsv->msg - 1) * 4 + sat_count;
                    if (sat_index >= gsv->sats) sat_index = gsv->sats - 1;
                    if (sat_index < 36) {
                        switch (times) {
                            case 4: gsv->sat_info[sat_index].prn = (unsigned char)strtol(p, NULL, 10); break;
                            case 5: gsv->sat_info[sat_index].elev = (unsigned char)strtol(p, NULL, 10); break;
                            case 6: gsv->sat_info[sat_index].azimuth = (unsigned short)strtol(p, NULL, 10); break;
                            case 7: gsv->sat_info[sat_index].SNR = (unsigned char)strtol(p, NULL, 10); break;
                            default: break;
                        }
                    }
                    if (times == 7) sat_count++;
                }
                break;
            default: break;
        }
        p = next + 1;
        times++;
    }
}

void gsv_show(GSV *gsv)
{
    int i;
    printf("=========== GSV DATA:\r\n");
    printf("msgs:%u\r\n", gsv->msgs);
    printf("msg:%u\r\n", gsv->msg);
    printf("sats:%u\r\n", gsv->sats);
    for (i = 0; i < gsv->sats && i < 36; i++) {
        printf("--- sat[%d] ---\r\n", i);
        printf("prn:%u\r\n", gsv->sat_info[i].prn);
        printf("elev:%u\r\n", gsv->sat_info[i].elev);
        printf("azimuth:%u\r\n", gsv->sat_info[i].azimuth);
        printf("SNR:%u\r\n", gsv->sat_info[i].SNR);
    }
    printf("sysid:%u\r\n", gsv->sysid);
}
#endif

#if ENABLE_VTG
void vtg_parse(VTG *vtg, char *vtg_data)
{
    char *p = NULL, *next = NULL;
    unsigned char times = 0;
    if (!vtg || !vtg_data) return;
    p = vtg_data;
    memset(vtg, 0, sizeof(VTG));
    while (p) {
        next = strpbrk(p, ",");
        if (next == p) { p = next + 1; times++; continue; }
        if (next == NULL) {
            next = strpbrk(p, "*");
            if (next == NULL) break;
        }
        switch (times) {
            case 1: vtg->track_true = strtod(p, NULL); break;
            case 3: vtg->track_mag = strtod(p, NULL); break;
            case 5: vtg->speed_Kn = strtod(p, NULL); break;
            case 7: vtg->speed_Km = strtod(p, NULL); break;
            case 9: vtg->mode = p[0]; break;
            default: break;
        }
        p = next + 1;
        times++;
    }
}

void vtg_show(VTG *vtg)
{
    printf("=========== VTG DATA:\r\n");
    printf("track_true:%lf\r\n", vtg->track_true);
    printf("track_mag:%lf\r\n", vtg->track_mag);
    printf("speed_Kn:%lf\r\n", vtg->speed_Kn);
    printf("speed_Km:%lf\r\n", vtg->speed_Km);
    printf("mode:%c\r\n", vtg->mode);
}
#endif

// ------------------------- 报文区分与解析入口 -------------------------
void gps_parse(char *gps_data)
{
    if (!gps_data) return;

#if ENABLE_GGA
    if (strncmp(gps_data, ID_GGA, strlen(ID_GGA)) == 0) {
        gga_parse(&gga, gps_data);
        gga_show(&gga);
        return;
    }
#endif
#if ENABLE_RMC
    if (strncmp(gps_data, ID_RMC, strlen(ID_RMC)) == 0) {
        rmc_parse(&rmc, gps_data);
        rmc_show(&rmc);
        return;
    }
#endif
#if ENABLE_GLL
    if (strncmp(gps_data, ID_GLL, strlen(ID_GLL)) == 0) {
        gll_parse(&gll, gps_data);
        gll_show(&gll);
        return;
    }
#endif
#if ENABLE_GSA
    if (strncmp(gps_data, ID_GSA, strlen(ID_GSA)) == 0) {
        gsa_parse(&gsa, gps_data);
        gsa_show(&gsa);
        return;
    }
#endif
#if ENABLE_GSV
    if (strncmp(gps_data, ID_GSV, strlen(ID_GSV)) == 0) {
        gsv_parse(&gsv, gps_data);
        gsv_show(&gsv);
        return;
    }
#endif
#if ENABLE_VTG
    if (strncmp(gps_data, ID_VTG, strlen(ID_VTG)) == 0) {
        vtg_parse(&vtg, gps_data);
        vtg_show(&vtg);
        return;
    }
#endif
}

// ------------------------- 读取并解析 GPS 数据 -------------------------
void GPS_ReadAndParse(void)
{
    char *pStart, *pEnd;

    while (g_GPS_RxDataOK == 0);

    printf("========== GPS RAW DATA:\r\n");
    printf("%s", g_GPS_RxBuf);

    pStart = (char*)g_GPS_RxBuf;
    while (1) {
        pStart = strpbrk(pStart, "$");
        if (pStart == NULL) break;
        pEnd = strpbrk(pStart, "\n");
        if (pEnd == NULL) break;
        *pEnd = '\0';
        gps_parse(pStart);
        *pEnd = '\n';
        pStart = pEnd + 1;
    }

    memset(g_GPS_RxBuf, 0, sizeof(g_GPS_RxBuf));
    g_GPS_RxBufLen = 0;
    g_GPS_RxDataOK = 0;
}

/* ========================== 主程序 ======================================== */
int main(void)
{
    UART2_Init(115200);
    printf("GPS Test Start...\r\n");

    GPS_Init(9600);

    while (1) {
        GPS_ReadAndParse();
    }
}
