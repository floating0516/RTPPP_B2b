#ifndef PPPB2B_DECODER_H
#define PPPB2B_DECODER_H

#include <QtCore>
#include <vector>
#include <cstdint>
#include <cstdio>
#include "rtklib.h"
#include "satObs.h"

extern "C" {
# include "clock_orbit_rtcm.h"
}

#define IF_MAXSAT 255
#define IF_MAXSSR 120
#define IF_MAXMASK 16

// Define types to match GNSS_PPP.h but within C++ context or using std types
typedef signed char INT8S;
typedef unsigned char INT8U;
typedef short INT16S;
typedef unsigned short INT16U;
typedef int INT32S;
typedef unsigned int INT32U;
typedef long long INT64S;
typedef unsigned long long INT64U;

typedef struct {
    char CK_FILE[255][255];
    char Site_number[20];
    char Machine_number[20];
    FILE* fp_output;
    char DATA_PPP_FILENAME[250];
} config_t;

typedef struct {
    short satslot;
    unsigned char IODN;
    unsigned char IODCorr;
    double radial;
    double Tangentialdir;
    double Normaldir;
    unsigned char URAclass;
    unsigned char URAvalue;
    double ura;
} Trackcp;

typedef struct {
    unsigned char IODP;
    char prn_make[IF_MAXSAT];
    char sub1NULL[200];
} ppp_sub1;

typedef struct {
    int sub1NULL;
    Trackcp trasub[6];
} ppp_sub2;

typedef struct {
    short satslot;
    unsigned char num4;
    unsigned char pattern[16];
    double deciation[16];
} Intersymbol_dev;

typedef struct {
    unsigned char num;
    Intersymbol_dev intersub3[31];
} ppp_sub3;

typedef struct {
    unsigned char IODP;
    unsigned char subtupe1;
    unsigned char IDO_corr[23];
    double c[23];
    int REV;
} ppp_sub4;

typedef struct {
    unsigned char IODP;
    unsigned char subtupe2;
    unsigned char URAclass[70];
    unsigned char URAvalue[70];
    int REV;
} ppp_sub5;

typedef struct {
    unsigned char IOD_corr;
    int Co;
} clock_NUMC_sub6;

typedef struct {
    unsigned int toc;
    short DEV;
    unsigned char IOD_SSR;
    unsigned char IOPD;
    unsigned char Slot_S;
    clock_NUMC_sub6 clock_N_sub6[22];
} clock_su6;

typedef struct {
    unsigned int tot;
    short DEV;
    unsigned char IOD_SSR;
    Trackcp numO[6];
} track_su6;

typedef struct {
    unsigned char NUMC;
    unsigned char NUMO;
    track_su6 sub6_tarck;
    clock_su6 sub6_clock;
} ppp_sub6;

typedef struct {
    short satslot;
    unsigned char IOD_corr;
    int Co;
} clock_NUMC_sub7;

typedef struct {
    unsigned int toc;
    short DEV;
    unsigned char IOD_SSR;
    unsigned char IOPD;
    unsigned char Slot_S;
    clock_NUMC_sub7 clock_sub7[22];
} clock_su7;

typedef struct {
    unsigned char NUMC;
    unsigned char NUMO;
    track_su6 sub7_tarck;
    clock_su7 sub7_clock;
} ppp_sub7;

union ppp_sub_union {
    ppp_sub1 type1;
    ppp_sub2 type2;
    ppp_sub3 type3;
    ppp_sub4 type4;
    ppp_sub5 type5;
    ppp_sub6 type6;
    ppp_sub7 type7;
};

typedef struct {
    short mestype;
    int CRC;
    short nudata;
    unsigned char SSR;
    short prn;
    int BDSweek;
    double BDSsod;
    double BDSsow;
    union ppp_sub_union type;
    unsigned char sta[10240];
} pppdata;

struct week_second {
    unsigned short BDSweek;
    INT32U BDSsecond;
};

struct Message_header {
    struct week_second current_week_second;
    unsigned short current_PRN;
    unsigned char current_sin_s;
    char current_mess_sys;
    unsigned char data[1024];
};

typedef struct {
    int SSR;
    int bweek;
    double bsow;
    double RAC[IF_MAXSAT][3];
    double ura[IF_MAXSAT];
    int iode[IF_MAXSAT];
    int iodcorr[IF_MAXSAT];
} ppp_ssr_orbit;

typedef struct {
    int SSR;
    int iodp;
    int bweek;
    double bsow;
    double C0[IF_MAXSAT];
    int iode[IF_MAXSAT];
    int iodcorr[IF_MAXSAT];
} ppp_ssr_clock;

typedef struct {
    int SSR;
    int iodp;
    char cmake[IF_MAXSAT];
} ppp_ssr_mask;

class PPPB2bDecoder : public QObject
{
    Q_OBJECT
 public:
    PPPB2bDecoder();
    ~PPPB2bDecoder();

    int input(const uint8_t* sbf_block, int len);
    void setStaID(const QString& staID);
    void setVerboseSatPrint(bool enabled);

private:
    uint16_t U2(const uint8_t* p) const;
    uint32_t U4(const uint8_t* p) const;
    QString svid2prn(quint16 svid) const;
    int decode_b2b_payload(const uint8_t* payload, int payload_len);

    // Adapted from b2b-decoder.c
    bool gnssinit(const char* ssrfile, const char* outfile);
    bool b2b_parse_ppp(unsigned char* datapackage, pppdata* PPPB2B);
    int b2b_updateiode(int SSR, int prn, int iodcrc);
    void b2b_fillmem(pppdata* p_sbas);
    bool b2b_parsecorr(struct Message_header* mh);
    // Helper functions from GNSS_PPP.h/b2b-decoder.c logic
    unsigned int getbitu(const unsigned char *buff, int pos, int len);
    int getbits(const unsigned char *buff, int pos, int len);
    int subtype_prn(char* make, int subtype, int ix);
    void m_outorbit(ppp_ssr_orbit* orbit);
    void m_outclock(ppp_ssr_clock* clock);
    void wksow2mjd(int week, double sow, int* mjd, double* sod);
    void mjd2wksow(int mjd, double sod, int *week, double *sow);
    
    // Member variables for state
    config_t ssr_config;
    int ssr_clock_count;
    int ssr_orbit_count;
    int ssr_mask_count;
    ppp_ssr_orbit ssr_orbits[IF_MAXSSR];
    ppp_ssr_clock ssr_clocks[IF_MAXSSR];
    ppp_ssr_mask ssr_masks[IF_MAXMASK];

    // Integration with BNC Core - similar to RTCM3coDecoder
    ClockOrbit                            _clkOrb;
    bncTime                               _lastTime;
    QString                               _staID;
    QMap<t_prn, unsigned int>             _IODs;
    QMap<bncTime, QList<t_orbCorr> >      _orbCorrections;
    QMap<bncTime, QList<t_clkCorr> >      _clkCorrections;
    QMap<t_prn, t_clkCorr>                _lastClkCorrections;

    void sendResults();
    void emitCorrections(const pppdata* p_sbas);
    void mapOrbitToRTCM3(const ppp_ssr_orbit* b2b_orbit, int satIdx);
    void mapClockToRTCM3(const ppp_ssr_clock* b2b_clock, int satIdx);
    void processBufferedCorrections();

    // Buffer for unifying corrections
    QList<t_orbCorr> _orbBuffer;
    QList<t_clkCorr> _clkBuffer;
    bncTime          _lastEmitTime;
    uint16_t         _epochWeek;
    uint32_t         _epochTow;
    bool             _epochC59Avail;
    bool             _epochC60Avail;
    bool             _epochC61Avail;

 signals:
    void newOrbCorrections(QList<t_orbCorr>);
    void newClkCorrections(QList<t_clkCorr>);
};

extern bool g_b2bDebugSatPrint;

#endif
