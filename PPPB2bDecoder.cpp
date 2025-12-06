#include <QDebug>
#include "PPPB2bDecoder.h"
#include "SBFcoDecoder.h"
#include "SBFDecoder.h"
#include "bnccore.h"
#include <iostream>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <cstdlib>

// Helper macros and functions from utils.c (or adapted)
static int md_julday(int iyear, int imonth, int iday) {
    int iyr, result;
    int doy_of_month[12] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
    if (iyear < 0 || imonth < 0 || iday < 0 || imonth > 12 || iday > 366 || (imonth != 0 && iday > 31)) {
        return 0; // Error handling simplified
    }
    iyr = iyear;
    if (imonth <= 2) iyr -= 1;
    result = 365 * iyear - 678941 + iyr / 4 - iyr / 100 + iyr / 400 + iday;
    if (imonth != 0) result = result + doy_of_month[imonth - 1];
    return result;
}

static void mjd2doy(int jd, int* iyear, int* idoy) {
    *iyear = (jd + 678940) / 365;
    *idoy = jd - md_julday(*iyear, 1, 1);
    while (*idoy <= 0) {
        (*iyear)--;
        *idoy = jd - md_julday(*iyear, 1, 1) + 1;
    }
}

static void yeardoy2monthday(int iyear, int idoy, int* imonth, int* iday) {
    int days_in_month[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    int id, i;
    if ((iyear % 4 == 0 && iyear % 100 != 0) || iyear % 400 == 0)
        days_in_month[1] = 29;
    id = idoy;
    for (i = 0; i < 12; i++) {
        id = id - days_in_month[i];
        if (id > 0) continue;
        *iday = id + days_in_month[i];
        *imonth = i + 1;
        break;
    }
}

static void mjd2date(int jd, double sod, int* iyear, int* imonth, int* iday, int* ih, int* imin, double* sec) {
    int doy = 0;
    mjd2doy(jd, iyear, &doy);
    yeardoy2monthday(*iyear, doy, imonth, iday);
    *ih = (int)(sod / 3600.0);
    *imin = (int)((sod - (*ih) * 3600.0) / 60.0);
    *sec = sod - (*ih) * 3600.0 - (*imin) * 60.0;
}

static int satslot_prn(int prn) {
    int a = -1;
    if (1 <= prn && prn <= 63) a = prn;
    else if (64 <= prn && prn <= 100) a = prn - 63;
    else if (101 <= prn && prn <= 137) a = prn - 100;
    else if (138 <= prn && prn <= 174) a = prn - 137;
    return a;
}

static int syssig_prn(int prn) {
    int a = -1;
    if (1 <= prn && prn <= 63) a = 0; // C
    else if (64 <= prn && prn < 100) a = 1; // G
    else if (101 <= prn && prn < 137) a = 2; // E
    else if (138 <= prn && prn < 174) a = 3; // R
    return a;
}

PPPB2bDecoder::PPPB2bDecoder() {
    // Initialize state variables
    ssr_clock_count = 0;
    ssr_orbit_count = 0;
    ssr_mask_count = 0;
    memset(ssr_orbits, 0, sizeof(ssr_orbits));
    memset(ssr_clocks, 0, sizeof(ssr_clocks));
    memset(ssr_masks, 0, sizeof(ssr_masks));
    memset(&ssr_config, 0, sizeof(ssr_config));

    // Initialize config (simulated gnssinit call without file ops for now, or default values)
    // We don't open files here as per original b2b-decoder which took file paths.
    // If specific initialization is needed, it can be added here.
    // strcpy(ssr_config.Machine_number, "sz001");
    // strcpy(ssr_config.Site_number, "BJ03");
    // strcpy(ssr_config.DATA_PPP_FILENAME, "b2b_outfile");
    gnssinit(nullptr, nullptr);

    // Initialize RTCM3 structures
    memset(&_clkOrb, 0, sizeof(_clkOrb));
    _staID = "B2b_SSR";
}

PPPB2bDecoder::~PPPB2bDecoder() {
    if (ssr_config.fp_output) {
        fclose(ssr_config.fp_output);
        ssr_config.fp_output = nullptr;
    }
    _IODs.clear();
    _orbCorrections.clear();
    _clkCorrections.clear();
    _lastClkCorrections.clear();
}

uint16_t PPPB2bDecoder::U2(const uint8_t* p) const { uint16_t u; memcpy(&u,p,2); return u; }
uint32_t PPPB2bDecoder::U4(const uint8_t* p) const { uint32_t u; memcpy(&u,p,4); return u; }

QString PPPB2bDecoder::svid2prn(quint16 svid) const{
  // GPS: 1–37 -> Gnn (nn = SVID)
  if (svid >= 1 && svid <= 37) {
    return QString("G%1").arg(svid, 2, 10, QLatin1Char('0'));
  }
  // GLONASS: 38–61 -> Rnn (nn = SVID-37)
  if (svid >= 38 && svid <= 61) {
    int prn = svid - 37;
    return QString("R%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // GLONASS unknown slot: 62
  if (svid == 62) {
    return QString("R??");
  }
  // GLONASS: 63–68 -> Rnn (nn = SVID-38)
  if (svid >= 63 && svid <= 68) {
    int prn = svid - 38;
    return QString("R%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // Galileo: 71–106 -> Enn (nn = SVID-70)
  if (svid >= 71 && svid <= 106) {
    int prn = svid - 70;
    return QString("E%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // SBAS: 120–140 -> Snn (nn = SVID-100)
  if (svid >= 120 && svid <= 140) {
    int prn = svid - 100;
    return QString("S%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // BeiDou: 141–180 -> Cnn (nn = SVID-140)
  if (svid >= 141 && svid <= 180) {
    int prn = svid - 140;
    return QString("C%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // QZSS: 181–190 -> Jnn (nn = SVID-180)
  if (svid >= 181 && svid <= 190) {
    int prn = svid - 180;
    return QString("J%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // NavIC/IRNSS: 191–197 -> Inn (nn = SVID-190)
  if (svid >= 191 && svid <= 197) {
    int prn = svid - 190;
    return QString("I%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // SBAS: 198–215 -> Snn (nn = SVID-157)
  if (svid >= 198 && svid <= 215) {
    int prn = svid - 157;
    return QString("S%1").arg(prn, 3, 10, QLatin1Char('0'));
  }
  // NavIC/IRNSS: 216–222 -> Inn (nn = SVID-208)
  if (svid >= 216 && svid <= 222) {
    int prn = svid - 208;
    return QString("I%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // BeiDou: 223–245 -> Cnn (nn = SVID-182)
  if (svid >= 223 && svid <= 245) {
    int prn = svid - 182;
    return QString("C%1").arg(prn, 2, 10, QLatin1Char('0'));
  }
  // 未知/不在范围
  return QString("UNK_%1").arg(svid);
}

static QByteArray parseBytesLiteral(const QString& s) {
  QString t = s;
  if (t.size() >= 3 && t.startsWith("b'") && t.endsWith("'")) {
    t = t.mid(2, t.size() - 3);
  } else if (t.size() >= 3 && t.startsWith("b\"") && t.endsWith("\"")) {
    t = t.mid(2, t.size() - 3);
  }
  QByteArray out;
  for (int i = 0; i < t.size();) {
    QChar c = t[i];
    if (c == '\\' && i + 1 < t.size() && t[i + 1] == 'x') {
      int j = i + 2;
      int consumed = 0;
      int val = 0;
      while (consumed < 2 && j < t.size()) {
        QChar hc = t[j];
        bool hex = (hc >= '0' && hc <= '9') || (hc >= 'a' && hc <= 'f') || (hc >= 'A' && hc <= 'F');
        if (!hex) break;
        int hv = QString(hc).toInt(nullptr, 16);
        val = (val << 4) | hv;
        ++consumed;
        ++j;
      }
      if (consumed > 0) {
        out.append(char(val));
        i = j;
      } else {
        ++i;
      }
    } else {
      out.append(char(c.unicode() & 0xFF));
      ++i;
    }
  }
  return out;
}

static QString toRawArray(const QByteArray& buf) {
  QString s;
  s.reserve(buf.size() * 6 + 32);
  s += "const uint8_t raw_msg[] = { ";
  for (int i = 0; i < buf.size(); ++i) {
    s += "0x";
    s += QString("%1").arg((unsigned char)buf.at(i), 2, 16, QLatin1Char('0')).toUpper();
    if (i + 1 < buf.size()) s += ", ";
  }
  s += " };";
  return s;
}

int PPPB2bDecoder::input(const uint8_t* sbf_block, int len) {
  if (!sbf_block || len < 8) return -1;
  uint16_t id_rev = U2(sbf_block + 4);
  uint16_t blen   = U2(sbf_block + 6);
  uint16_t type   = (uint16_t)(id_rev & 0x1FFF);
  if (blen != len) return -2;
  const uint8_t* payload = sbf_block + 8;
  int payload_len = len - 8;
  if (type == 4242) {
    return decode_b2b_payload(payload, payload_len);
  }
  return 0;
}

int PPPB2bDecoder::decode_b2b_payload(const uint8_t* payload, int payload_len) {
  if (payload_len < 12) return -1;
  uint32_t TOW = U4(payload + 0) / 1000;
  uint16_t WNc = U2(payload + 4);
  uint8_t  SVIDb = *(payload + 6);
  uint8_t  CRCp  = *(payload + 7);
  uint8_t  Src   = *(payload + 9);
  uint8_t  RxCh  = *(payload + 11);
  QString prnMask = svid2prn(SVIDb);

  if (prnMask=="C59")
  {
    QString head = QString("PPPB2b: TOW=%1 WNc=%2 PRN=%3 CRCPassed=%4 Src=%5 RxCh=%6")
                   .arg(TOW).arg(WNc).arg(svid2prn(SVIDb)).arg((int)CRCp).arg((int)Src).arg((int)RxCh);
    BNC_CORE->slotMessage(head.toUtf8(), false);

    const int NAV_WORDS = 31;
    if (payload_len >= 12 + NAV_WORDS * 4) {
      QString navHex;
      navHex.reserve(124 * 2);
      for (int w = 0; w < 31; ++w) {
        quint32 NBw = U4(payload + 12 + w * 4);
        navHex += QString("%1").arg(NBw, 8, 16, QLatin1Char('0')).toUpper();
      }
      
      // Check for EC0FC prefix to skip invalid frames
      if (navHex.startsWith("EC0FC", Qt::CaseInsensitive)) {
          BNC_CORE->slotMessage("Skipping frame starting with EC0FC", false);
          return 1; // Return 1 to continue processing next inputs
      }

      QByteArray decoded = SBFcoDecoder::decode_LDPC_navbitsRaw(navHex.toUtf8());
      // BNC_CORE->slotMessage(navHex.toUtf8(), false); // Optional: print raw hex
      
      // Prepare preview log
      int previewN = qMin(decoded.size(), 124);
      QByteArray preview = decoded.left(previewN);
      QString previewEsc;
      previewEsc.reserve(preview.size() * 4 + 2);
      previewEsc += "b'";
      for (int i = 0; i < preview.size(); ++i) {
        previewEsc += "\\x";
        previewEsc += QString("%1").arg((unsigned char)preview.at(i), 2, 16, QLatin1Char('0')).toUpper();
      }
      previewEsc += "'";
      QString out = QString("C59 NAVBits decoded preview (%1 bytes): %2")
                      .arg(previewN)
                      .arg(toRawArray(parseBytesLiteral(previewEsc)));
      BNC_CORE->slotMessage(out.toUtf8(), false);

      // Use the decoded data with the new C-based logic
      if (decoded.size() > 0) {
          // Construct Message_header for b2b_parsecorr
          struct Message_header mh;
          memset(&mh, 0, sizeof(mh));
          mh.current_PRN = 59; // As per b2b_ifsystem logic for PPP
          mh.current_week_second.BDSweek = WNc;
          // Note: TOW from SBF is in seconds, but b2b_parsecorr expects BDSsecond/sow logic.
          // Here we pass the TOW as received.
          mh.current_week_second.BDSsecond = TOW; 
          mh.current_sin_s = 0; // Placeholder
          mh.current_mess_sys = 'C';

          // Set epoch time for integration
          bncTime epoTime;
          epoTime.set(WNc, TOW);
          _lastTime = epoTime;

          unsigned int year, month, day, hour, min;
          double sec;
          _lastTime.civil_date(year, month, day);
          _lastTime.civil_time(hour, min, sec);
          int isec = (int)sec;
          int msec = (int)((sec - isec) * 1000);
          QDateTime qdt(QDate(year, month, day), QTime(hour, min, isec, msec), Qt::UTC);
          // BNC_CORE->setDateAndTimeGPS(qdt); // Do not force update system time with SBF time

          // Copy decoded data to mh.data. 
          // b2b_parsecorr expects raw bytes in mh.data.
          // Ensure we don't overflow.
          int copyLen = qMin((int)sizeof(mh.data), decoded.size());
          memcpy(mh.data, decoded.constData(), copyLen);

          // Call b2b_parsecorr
          bool res = b2b_parsecorr(&mh);
          if (res) {
             // Debug Output: Check Time Sync
             QDateTime sysTime = BNC_CORE->dateAndTimeGPS();
             QString msg = QString("B2b Time: %1, Sys Time: %2, Diff: %3 s")
                           .arg(qdt.toString("yyyy-MM-dd HH:mm:ss"))
                           .arg(sysTime.toString("yyyy-MM-dd HH:mm:ss"))
                           .arg(sysTime.isValid() ? QString::number(qdt.secsTo(sysTime)) : "N/A");
             BNC_CORE->slotMessage(msg.toUtf8(), false);

             // BNC_CORE->slotMessage(QString("b2b_parsecorr success").toUtf8(), false);
             // After parsing, data is in ssr_orbits and ssr_clocks. 
             // Now trigger integration to BNC core.
             sendResults();
          } else {
             BNC_CORE->slotMessage(QString("b2b_parsecorr failed").toUtf8(), false);
          }
      }
    }
  }
  return 1;
}

// --- Helper implementations adapted from b2b-decoder.c ---

unsigned int PPPB2bDecoder::getbitu(const unsigned char *buff, int pos, int len) {
    unsigned int bits = 0;
    int i;
    for (i = 0; i < len; i++) bits = (bits << 1) + ((buff[(pos + i) / 8] >> (7 - (pos + i) % 8)) & 1u);
    return bits;
}

int PPPB2bDecoder::getbits(const unsigned char *buff, int pos, int len) {
    unsigned int bits = getbitu(buff, pos, len);
    if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) return (int)bits;
    return (int)(bits | (~0u << len)); /* extend sign */
}

bool PPPB2bDecoder::gnssinit(const char* ssrfile, const char* outfile) {
    // Logic already in constructor, but if needed to reset or re-init
    memset(ssr_orbits, 0, sizeof(ssr_orbits));
    memset(ssr_clocks, 0, sizeof(ssr_clocks));
    memset(ssr_masks, 0, sizeof(ssr_masks));
    memset(&ssr_config, 0, sizeof(ssr_config));
    ssr_orbit_count = 0;
    ssr_clock_count = 0;
    ssr_mask_count = 0;

    // For real-time, these files might not be used directly, but we keep the structure.
    if (ssrfile) strcpy(ssr_config.CK_FILE[0], ssrfile);
    strcpy(ssr_config.Machine_number, "sz001");
    strcpy(ssr_config.Site_number, "BJ03");
    strcpy(ssr_config.DATA_PPP_FILENAME, "b2b_outfile");

    // Not opening file here to avoid modifying filesystem unless needed
    return true;
}

bool PPPB2bDecoder::b2b_parse_ppp(unsigned char* datapackage, pppdata* PPPB2B) {
    int b;
    unsigned int type = getbitu(datapackage, 0, 6);
    PPPB2B->mestype = type;
    PPPB2B->BDSsod = getbitu(datapackage, 6, 17);
    PPPB2B->nudata = getbitu(datapackage, 23, 4);
    PPPB2B->SSR = getbitu(datapackage, 27, 2);
    PPPB2B->CRC = getbitu(datapackage, 462, 24);
    int r = 29, a = 0;
    if (8 <= PPPB2B->mestype && PPPB2B->mestype <= 63) {
        return true;
    }
    switch (type) {
    case 1:
        PPPB2B->type.type1.IODP = getbitu(datapackage, 29, 4);
        for (a = 0; a < IF_MAXSAT; a++) {
            PPPB2B->type.type1.prn_make[a] = getbitu(datapackage, 33 + a, 1);
        }
        for (a = 0; a < 174; a++) {
            PPPB2B->type.type1.sub1NULL[a] = getbitu(datapackage, 288 + a, 1);
        }
        break;
    case 2:
        for (a = 0; a < 6; a++) {
            PPPB2B->type.type2.trasub[a].satslot = getbitu(datapackage, r, 9); r += 9;
            PPPB2B->type.type2.trasub[a].IODN = getbitu(datapackage, r, 10); r += 10;
            PPPB2B->type.type2.trasub[a].IODCorr = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type2.trasub[a].radial = getbits(datapackage, r, 15) * 0.0016; r += 15;
            PPPB2B->type.type2.trasub[a].Tangentialdir = getbits(datapackage, r, 13) * 0.0064; r += 13;
            PPPB2B->type.type2.trasub[a].Normaldir = getbits(datapackage, r, 13) * 0.0064; r += 13;
            PPPB2B->type.type2.trasub[a].URAclass = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type2.trasub[a].URAvalue = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type2.trasub[a].ura = pow(3, PPPB2B->type.type2.trasub[a].URAclass) * (1 + 0.25 * PPPB2B->type.type2.trasub[a].URAvalue) - 1;
        }
        PPPB2B->type.type2.sub1NULL = getbitu(datapackage, r, 19);
        break;
    case 3:
        b = 0;
        PPPB2B->type.type3.num = getbitu(datapackage, r, 5); r += 5;
        for (a = 0; a < PPPB2B->type.type3.num; a++) {
            PPPB2B->type.type3.intersub3[a].satslot = getbitu(datapackage, r, 9); r += 9;
            PPPB2B->type.type3.intersub3[a].num4 = getbitu(datapackage, r, 4); r += 4;
            for (b = 0; b < PPPB2B->type.type3.intersub3[a].num4; b++) {
                PPPB2B->type.type3.intersub3[a].pattern[b] = getbitu(datapackage, r, 4); r += 4;
                PPPB2B->type.type3.intersub3[a].deciation[b] = getbits(datapackage, r, 12) * 0.017; r += 12;
            }
        }
        break;
    case 4:
        PPPB2B->type.type4.IODP = getbitu(datapackage, r, 4); r += 4;
        PPPB2B->type.type4.subtupe1 = getbitu(datapackage, r, 5); r += 5;
        for (a = 0; a < 23; a++) {
            PPPB2B->type.type4.IDO_corr[a] = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type4.c[a] = getbits(datapackage, r, 15) * 0.0016; r += 15;
        }
        PPPB2B->type.type4.REV = getbitu(datapackage, r, 10);
        break;
    case 5:
        PPPB2B->type.type5.IODP = getbitu(datapackage, r, 4); r += 4;
        PPPB2B->type.type5.subtupe2 = getbitu(datapackage, r, 3); r += 3;
        for (a = 0; a < 70; a++) {
            PPPB2B->type.type5.URAclass[a] = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type5.URAvalue[a] = getbitu(datapackage, r, 3); r += 3;
        }
        break;
    case 6:
        PPPB2B->type.type6.NUMC = getbitu(datapackage, r, 5); r += 5;
        PPPB2B->type.type6.NUMO = getbitu(datapackage, r, 3); r += 3;

        PPPB2B->type.type6.sub6_clock.toc = getbitu(datapackage, r, 17); r += 17;
        PPPB2B->type.type6.sub6_clock.DEV = getbitu(datapackage, r, 4); r += 4;
        PPPB2B->type.type6.sub6_clock.IOD_SSR = getbitu(datapackage, r, 2); r += 2;
        PPPB2B->type.type6.sub6_clock.IOPD = getbitu(datapackage, r, 4); r += 4;
        PPPB2B->type.type6.sub6_clock.Slot_S = getbitu(datapackage, r, 9); r += 9;
        for (a = 0; a < PPPB2B->type.type6.NUMC; a++) {
            PPPB2B->type.type6.sub6_clock.clock_N_sub6[a].IOD_corr = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type6.sub6_clock.clock_N_sub6[a].Co = getbitu(datapackage, r, 15); r += 15;
        }
        PPPB2B->type.type6.sub6_tarck.tot = getbitu(datapackage, r, 17); r += 17;
        PPPB2B->type.type6.sub6_tarck.DEV = getbitu(datapackage, r, 4); r += 4;
        PPPB2B->type.type6.sub6_clock.IOD_SSR = getbitu(datapackage, r, 2); r += 2;
        for (a = 0; a < PPPB2B->type.type6.NUMO; a++) {
            PPPB2B->type.type6.sub6_tarck.numO[a].satslot = getbitu(datapackage, r, 9); r += 9;
            PPPB2B->type.type6.sub6_tarck.numO[a].IODN = getbitu(datapackage, r, 10); r += 10;
            PPPB2B->type.type6.sub6_tarck.numO[a].IODCorr = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type6.sub6_tarck.numO[a].radial = getbits(datapackage, r, 15) * 0.0016; r += 15;
            PPPB2B->type.type6.sub6_tarck.numO[a].Tangentialdir = getbits(datapackage, r, 13) * 0.0064; r += 13;
            PPPB2B->type.type6.sub6_tarck.numO[a].Normaldir = getbits(datapackage, r, 13) * 0.0064; r += 13;
            PPPB2B->type.type6.sub6_tarck.numO[a].URAclass = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type6.sub6_tarck.numO[a].URAvalue = getbitu(datapackage, r, 3); r += 3;
        }
        break;
    case 7:
        PPPB2B->type.type7.NUMC = getbitu(datapackage, r, 5); r += 5;
        PPPB2B->type.type7.NUMO = getbitu(datapackage, r, 3); r += 3;
        PPPB2B->type.type7.sub7_clock.toc = getbitu(datapackage, r, 17); r += 17;
        PPPB2B->type.type7.sub7_clock.DEV = getbitu(datapackage, r, 4); r += 4;
        PPPB2B->type.type7.sub7_clock.IOD_SSR = getbitu(datapackage, r, 2); r += 2;
        for (a = 0; a < PPPB2B->type.type6.NUMC; a++) {
            PPPB2B->type.type7.sub7_clock.clock_sub7[a].satslot = getbitu(datapackage, r, 9); r += 9;
            PPPB2B->type.type7.sub7_clock.clock_sub7[a].IOD_corr = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type7.sub7_clock.clock_sub7[a].Co = getbitu(datapackage, r, 15); r += 15;
        }
        PPPB2B->type.type7.sub7_tarck.tot = getbitu(datapackage, r, 17); r += 17;
        PPPB2B->type.type7.sub7_tarck.DEV = getbitu(datapackage, r, 4); r += 4;
        PPPB2B->type.type7.sub7_tarck.IOD_SSR = getbitu(datapackage, r, 2); r += 2;
        for (a = 0; a < PPPB2B->type.type7.NUMO; a++) {
            PPPB2B->type.type7.sub7_tarck.numO[a].satslot = getbitu(datapackage, r, 9); r += 9;
            PPPB2B->type.type7.sub7_tarck.numO[a].IODN = getbitu(datapackage, r, 10); r += 10;
            PPPB2B->type.type7.sub7_tarck.numO[a].IODCorr = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type7.sub7_tarck.numO[a].radial = getbits(datapackage, r, 15) * 0.0016; r += 15;
            PPPB2B->type.type7.sub7_tarck.numO[a].Tangentialdir = getbits(datapackage, r, 13) * 0.0064; r += 13;
            PPPB2B->type.type7.sub7_tarck.numO[a].Normaldir = getbits(datapackage, r, 13) * 0.0064; r += 13;
            PPPB2B->type.type7.sub7_tarck.numO[a].URAclass = getbitu(datapackage, r, 3); r += 3;
            PPPB2B->type.type7.sub7_tarck.numO[a].URAvalue = getbitu(datapackage, r, 3); r += 3;
        }
        break;
    default:
        // printf("parse_PPPB2B  error\n");
        return false;
    }
    return true;
}

int PPPB2bDecoder::b2b_updateiode(int SSR, int prn, int iodcrc) {
    for(int i = ssr_orbit_count - 1; i >= (ssr_orbit_count - 7 >= 0 ? ssr_orbit_count - 7 : 0); i--) {
        if(ssr_orbits[i].SSR != SSR) continue;
        if(ssr_orbits[i].iodcorr[prn] == iodcrc)
            return ssr_orbits[i].iode[prn];
    }
    return -1;
}

void PPPB2bDecoder::b2b_fillmem(pppdata* p_sbas) {
    int i, prn, ipos, nsum, bgot;
    ppp_ssr_orbit *ptr_fill = NULL;
    ppp_ssr_mask* ptr_mask = NULL;
    ppp_ssr_clock* ptr_clk = NULL;

    switch(p_sbas->mestype) {
    case 1:
        for(i = 0, bgot = false; i < ssr_mask_count; i++) {
            if(ssr_masks[i].SSR != p_sbas->SSR) continue;
            if(ssr_masks[i].iodp == p_sbas->type.type1.IODP) {
                bgot = true;
            }
        }
        if(bgot == false) {
            if (ssr_mask_count >= IF_MAXMASK) {
                for (i = 0; i < ssr_mask_count - 1; i++) {
                    ssr_masks[i] = ssr_masks[i + 1];
                }
                ssr_mask_count--;
            }
            ptr_mask = ssr_masks + ssr_mask_count++;

            ptr_mask->SSR = p_sbas->SSR;
            ptr_mask->iodp = p_sbas->type.type1.IODP;
            memcpy(ptr_mask->cmake, p_sbas->type.type1.prn_make, sizeof(char) * IF_MAXSAT);
            
            BNC_CORE->slotMessage(QString("MT1 MASK: SSR=%1 IODP=%2").arg(ptr_mask->SSR).arg(ptr_mask->iodp).toUtf8(), false);
            QString mask_str = "BDS MASK content: ";
            for(int k=0; k<IF_MAXSAT; k++) {
                if(ptr_mask->cmake[k]) {
                     int prn = satslot_prn(k + 1); // Assuming index k corresponds to slot k+1
                     if(syssig_prn(k+1) == 0) { // 0 for 'C' in SYS array
                         mask_str += QString::number(prn) + " ";
                     }
                }
            }
            BNC_CORE->slotMessage(mask_str.toUtf8(), false);
        }
        break;
    case 2:
        ipos = -1;
        for(i = 0; i < ssr_orbit_count; i++) {
            if(ssr_orbits[i].SSR != p_sbas->SSR || ssr_orbits[i].bweek != p_sbas->BDSweek || (int)ssr_orbits[i].bsow != p_sbas->BDSsow) continue;
            ipos = i;
            break;
        }
        ptr_fill = ssr_orbits + ipos;
        if(-1 == ipos) {
            if(ssr_orbit_count >= IF_MAXSSR) {
                for(i = 0; i < ssr_orbit_count - 1; i++) {
                    ssr_orbits[i] = ssr_orbits[i + 1];
                }
                ssr_orbit_count--;
            }
            ptr_fill = ssr_orbits + ssr_orbit_count++;
            ptr_fill->bweek = p_sbas->BDSweek;
            ptr_fill->bsow = p_sbas->BDSsow;
            ptr_fill->SSR = p_sbas->SSR;
            for(i = 0; i < IF_MAXSAT; i++) ptr_fill->iodcorr[i] = -1;
            for(i = 0; i < IF_MAXSAT; i++) ptr_fill->iode[i] = -1;
            for(i = ssr_orbit_count - 1, nsum = 0; i >= 0; i--) {
                if(ssr_orbits[i].SSR == p_sbas->SSR)
                    if(++nsum == 2)
                        ipos = i;
            }
            if(nsum >= 2) m_outorbit(&ssr_orbits[ipos]);
        }
        for(i = 0; i < 6; i++) {
            if(p_sbas->type.type2.trasub[i].ura == 0)
                continue;
            prn = p_sbas->type.type2.trasub[i].satslot - 1;
            ptr_fill->RAC[prn][0]= p_sbas->type.type2.trasub[i].radial;
            ptr_fill->RAC[prn][1]= p_sbas->type.type2.trasub[i].Tangentialdir;
            ptr_fill->RAC[prn][2]= p_sbas->type.type2.trasub[i].Normaldir;
            ptr_fill->iodcorr[prn] = p_sbas->type.type2.trasub[i].IODCorr;
            ptr_fill->iode[prn] = p_sbas->type.type2.trasub[i].IODN;
            ptr_fill->ura[prn] = p_sbas->type.type2.trasub[i].ura;
        }
        BNC_CORE->slotMessage(QString("MT2 ORBIT processed for SSR=%1").arg(p_sbas->SSR).toUtf8(), false);
        
        // Trigger immediate signal emission for this message
        emitCorrections(p_sbas);
        break;
    case 3:
        break;
    case 4:
        ipos = -1;
        for(i = 0; i < ssr_clock_count; i++) {
            if(ssr_clocks[i].SSR != p_sbas->SSR || ssr_clocks[i].bweek != p_sbas->BDSweek || (int)ssr_clocks[i].bsow != p_sbas->BDSsow) continue;
            ipos = i;
            break;
        }
        ptr_clk = ssr_clocks + ipos;
        if(ipos == -1) {
            if(ssr_clock_count >= IF_MAXSSR) {
                for(i = 0; i < ssr_clock_count - 1; i++) {
                    ssr_clocks[i] = ssr_clocks[i + 1];
                }
                ssr_clock_count--;
            }
            ptr_clk = ssr_clocks + ssr_clock_count++;
            ptr_clk->SSR = p_sbas->SSR;
            ptr_clk->bweek = p_sbas->BDSweek;
            ptr_clk->bsow = p_sbas->BDSsow;
            ptr_clk->iodp = p_sbas->type.type4.IODP;
            for(i = 0; i < IF_MAXSAT; i++) ptr_clk->iode[i] = -1;
            for(i = 0; i < IF_MAXSAT; i++) ptr_clk->iodcorr[i] = -1;
            /*for(i = ssr_clock_count - 1, nsum = 0; i >= 0; i--) {
                if(ssr_clocks[i].SSR == p_sbas->SSR)
                    if(++nsum == 2)
                        ipos = i;
            }
            if(nsum >= 2) m_outclock(&ssr_clocks[ipos]);*/
        }
        for(i = 0, ptr_mask = NULL; i < ssr_mask_count; i++) {
            if(ssr_masks[i].iodp == ptr_clk->iodp) {
                ptr_mask = ssr_masks + i;
                break;
            }
        }
        for(i = 0; i < 23 && ptr_mask != NULL; i++) {
            prn = subtype_prn(ptr_mask->cmake, p_sbas->type.type4.subtupe1, i+1);
            if(prn == -1) continue;
            ptr_clk->C0[prn] = p_sbas->type.type4.c[i];
            if(fabs(fabs(ptr_clk->C0[prn]) - 26.2128) < 0.01) continue;
            ptr_clk->iodcorr[prn] = p_sbas->type.type4.IDO_corr[i];
            ptr_clk->iode[prn] = b2b_updateiode(ptr_clk->SSR, prn, ptr_clk->iodcorr[prn]);
        }
        m_outclock(ptr_clk);
        BNC_CORE->slotMessage(QString("MT4 CLOCK processed for SSR=%1").arg(p_sbas->SSR).toUtf8(), false);
        
        // Trigger immediate signal emission for this message
        emitCorrections(p_sbas);
        break;
    case 5:
        break;
    case 6:
        break;
    case 7:
        break;
    default:
        break;
    }
}

bool PPPB2bDecoder::b2b_parsecorr(struct Message_header* mh) {
    int mjd, week, i;
    double sow;
    pppdata p_sbas = {0};
    // In real-time stream, TOW/Week from SBF header is usually reliable.
    // Original code used a search loop around mjd because file data might lack context or have jumps.
    // For real-time, we trust the SBF time but should still be careful about week rollovers if needed.
    // However, the loop search logic relies on checking if 'ddif' is small.
    // We will keep the logic but optimize for the fact we already have current week/sow.
    
    p_sbas.BDSsow = mh->current_week_second.BDSsecond;
    p_sbas.BDSweek = mh->current_week_second.BDSweek;
    wksow2mjd(p_sbas.BDSweek, p_sbas.BDSsow, &mjd, &p_sbas.BDSsod);
    p_sbas.prn = mh->current_PRN;

    // Parse the payload bits first
    if(b2b_parse_ppp(mh->data, &p_sbas)) {
        // Time consistency check (optional for real-time if we trust SBF timestamp)
        // The original loop checks if the decoded time matches the header time within half a week.
        // Since SBF provides explicit WNc (Week Number) and TOW, we might not need to search +/- 1 day/week unless there's ambiguity.
        // We'll keep the check but prioritize the provided week.
        
        for(i = -1; i < 1; i++) {
            mjd2wksow(mjd + i, p_sbas.BDSsod, &week, &sow);
            double ddif = (week - p_sbas.BDSweek) * 86400 * 7 + sow - p_sbas.BDSsow;
            if(fabs(ddif) < 43200) {
                break;
            }
        }
        mjd2wksow(mjd + i, p_sbas.BDSsod, &p_sbas.BDSweek, &p_sbas.BDSsow);
        
        b2b_fillmem(&p_sbas);
        return true;
    }
    return false;
}

int PPPB2bDecoder::subtype_prn(char* make, int subtype, int ix) {
    int i, ipos = subtype * 23 + ix, b = 0;
    for(i = 0; i < IF_MAXSAT; i++) {
        if(make[i] == 1) b++;
        if(b == ipos) return i;
    }
    return -1;
}

void PPPB2bDecoder::m_outorbit(ppp_ssr_orbit* orbit) {
    // Placeholder for outputting orbit to BNC console instead of file
    int nsum = 0, week, iyear, imonth, iday, ih, im, mjd, nsat = 0;
    double dsec, sod;
    char SYS[4] = {'C','G','E','R'};
    week = orbit->bweek + 1356.0; // Adjust to GPS week if needed? Original code had this.
    wksow2mjd(week, orbit->bsow, &mjd, &sod);
    mjd2date(mjd, sod, &iyear, &imonth, &iday, &ih, &im, &dsec);
    for(int isat = 0; isat < IF_MAXSAT; isat++) {
        if(orbit->iode[isat] == -1) continue;
        nsat++;
    }
    
    QString header = QString("> ORBIT %1 %2 %3 %4 %5 %6 %7 %8 CLK01")
        .arg(iyear, 4, 10, QLatin1Char('0'))
        .arg(imonth, 2, 10, QLatin1Char('0'))
        .arg(iday, 2, 10, QLatin1Char('0'))
        .arg(ih, 2, 10, QLatin1Char('0'))
        .arg(im, 2, 10, QLatin1Char('0'))
        .arg(dsec, 4, 'f', 1)
        .arg(2)
        .arg(nsat);
    BNC_CORE->slotMessage(header.toUtf8(), false);

    for(int isat = 0; isat < IF_MAXSAT; isat++) {
        if(orbit->iode[isat] == -1) continue;
        int sysIdx = syssig_prn(isat + 1);
        if (sysIdx < 0) continue;
        if (SYS[sysIdx] == 'C') {
             BNC_CORE->slotMessage(QString("BDS ORB: PRN=%1 IODN=%2 IODCorr=%3").arg(satslot_prn(isat + 1)).arg(orbit->iode[isat]).arg(orbit->iodcorr[isat]).toUtf8(), false);
        }
        QString line = QString("%1%2 %3 %4 %5 %6 %7 %8 %9")
            .arg(SYS[sysIdx])
            .arg(satslot_prn(isat + 1), 2, 10, QLatin1Char('0'))
            .arg(orbit->iode[isat], 10)
            .arg(orbit->RAC[isat][0], 11, 'f', 4)
            .arg(orbit->RAC[isat][1], 11, 'f', 4)
            .arg(orbit->RAC[isat][2], 11, 'f', 4)
            .arg(0.0, 11, 'f', 4)
            .arg(0.0, 11, 'f', 4)
            .arg(0.0, 11, 'f', 4);
        BNC_CORE->slotMessage(line.toUtf8(), false);
    }
}

void PPPB2bDecoder::m_outclock(ppp_ssr_clock* clock) {
    int week, iyear, imonth, iday, ih, im, nsat = 0, mjd;
    double dsec, sod;
    char SYS[4] = {'C','G','E','R'};
    week = clock->bweek + 1356.0;
    wksow2mjd(week, clock->bsow, &mjd, &sod);
    mjd2date(mjd, sod, &iyear, &imonth, &iday, &ih, &im, &dsec);
    for(int isat = 0; isat < IF_MAXSAT; isat++) {
        if(clock->iode[isat] == -1) continue;
        nsat++;
    }
    
    QString header = QString("> CLOCK %1 %2 %3 %4 %5 %6 %7 %8 CLK01")
        .arg(iyear, 4, 10, QLatin1Char('0'))
        .arg(imonth, 2, 10, QLatin1Char('0'))
        .arg(iday, 2, 10, QLatin1Char('0'))
        .arg(ih, 2, 10, QLatin1Char('0'))
        .arg(im, 2, 10, QLatin1Char('0'))
        .arg(dsec, 4, 'f', 1)
        .arg(2)
        .arg(nsat);
    BNC_CORE->slotMessage(header.toUtf8(), false);

    for(int isat = 0; isat < IF_MAXSAT; isat++) {
        if(clock->iode[isat] == -1) continue;
        int sysIdx = syssig_prn(isat + 1);
        if (sysIdx < 0) continue;
        if (SYS[sysIdx] == 'C') {
             BNC_CORE->slotMessage(QString("BDS CLK: PRN=%1 IODP=%2 IODCorr=%3").arg(satslot_prn(isat + 1)).arg(clock->iodp).arg(clock->iodcorr[isat]).toUtf8(), false);
        }
        QString line = QString("%1%2 %3 %4 %5 %6")
            .arg(SYS[sysIdx])
            .arg(satslot_prn(isat + 1), 2, 10, QLatin1Char('0'))
            .arg(clock->iode[isat], 10)
            .arg(clock->C0[isat], 11, 'f', 4)
            .arg(0.0, 11, 'f', 4)
            .arg(0.0, 11, 'f', 4);
        BNC_CORE->slotMessage(line.toUtf8(), false);
    }
}

void PPPB2bDecoder::wksow2mjd(int week, double sow, int* mjd, double* sod) {
    *mjd = (int)((sow + week * 604800.0) / 86400.0) + 44244;
    *sod = fmod(sow + week * 604800.0, 86400.0);
}

void PPPB2bDecoder::mjd2wksow(int mjd, double sod, int *week, double *sow) {
    double total_sec = (mjd - 44244) * 86400.0 + sod;
    *week = (int)(total_sec / 604800.0);
    *sow = total_sec - *week * 604800.0;
}

void PPPB2bDecoder::mapOrbitToRTCM3(const ppp_ssr_orbit* b2b_orbit, int satIdx) {
}

void PPPB2bDecoder::mapClockToRTCM3(const ppp_ssr_clock* b2b_clock, int satIdx) {
}

void PPPB2bDecoder::emitCorrections(const pppdata* p_sbas) {
    // Emit corrections based on the message type processed in b2b_fillmem
    
    // Determine which structures to use based on p_sbas->SSR
    ppp_ssr_orbit* current_orbits = NULL;
    ppp_ssr_clock* current_clocks = NULL;
    
    // Find the orbit/clock sets that match the current SSR ID
    for (int i=0; i<ssr_orbit_count; i++) {
        if (ssr_orbits[i].SSR == p_sbas->SSR) {
             current_orbits = &ssr_orbits[i];
             break;
        }
    }
    for (int i=0; i<ssr_clock_count; i++) {
        if (ssr_clocks[i].SSR == p_sbas->SSR) {
             current_clocks = &ssr_clocks[i];
             break;
        }
    }
    
    // If message type is 2 (Orbit), buffer orbit corrections
    if (p_sbas->mestype == 2 && current_orbits) {
        for(int isat = 0; isat < IF_MAXSAT; isat++) {
             if(current_orbits->iode[isat] == -1) continue;
             
             int sysIdx = syssig_prn(isat + 1);
             if (sysIdx < 0) continue;
             
             char sysCh = ' ';
             if (sysIdx == 0) sysCh = 'C';
             else if (sysIdx == 1) sysCh = 'G';
             else if (sysIdx == 2) sysCh = 'E';
             else if (sysIdx == 3) sysCh = 'R';
             
             t_orbCorr orbCorr;
             int prn = satslot_prn(isat + 1);
             
             orbCorr._prn.set(sysCh, prn);
             orbCorr._staID = _staID.toStdString();
             orbCorr._iod = current_orbits->iode[isat];
             orbCorr._time = _lastTime; 
             orbCorr._updateInt = 0; 
             orbCorr._system = sysCh;
             
             orbCorr._xr[0] = current_orbits->RAC[isat][0]; 
             orbCorr._xr[1] = current_orbits->RAC[isat][1]; 
             orbCorr._xr[2] = current_orbits->RAC[isat][2]; 
             
             orbCorr._dotXr[0] = 0.0;
             orbCorr._dotXr[1] = 0.0;
             orbCorr._dotXr[2] = 0.0;
             
             _orbBuffer.append(orbCorr);
        }
    }
    
    // If message type is 4 (Clock), buffer clock corrections
    if (p_sbas->mestype == 4 && current_clocks) {
        for(int isat = 0; isat < IF_MAXSAT; isat++) {
            if(current_clocks->iode[isat] == -1) continue;
            
            int sysIdx = syssig_prn(isat + 1);
            if (sysIdx < 0) continue;
            
            char sysCh = ' ';
            if (sysIdx == 0) sysCh = 'C';
            else if (sysIdx == 1) sysCh = 'G';
            else if (sysIdx == 2) sysCh = 'E';
            else if (sysIdx == 3) sysCh = 'R';
            
            t_clkCorr clkCorr;
            int prn = satslot_prn(isat + 1);
            
            clkCorr._prn.set(sysCh, prn);
            clkCorr._staID = _staID.toStdString();
            clkCorr._time = _lastTime;
            clkCorr._updateInt = 0;
            
            clkCorr._dClk = current_clocks->C0[isat] / t_CST::c; 
            clkCorr._dotDClk = 0.0;
            clkCorr._dotDotDClk = 0.0;
            
            clkCorr._iod = current_clocks->iode[isat]; 
            
            _clkBuffer.append(clkCorr);
        }
    }

    processBufferedCorrections();
}

void PPPB2bDecoder::processBufferedCorrections() {
    if (_lastEmitTime.undef()) {
        _lastEmitTime = _lastTime;
    }

    // Check if 30 seconds have passed since last emit
    if (std::abs(_lastTime - _lastEmitTime) >= 5.0) {
        if (!_orbBuffer.isEmpty()) {
            // Update time for all buffered orbit corrections to current time
            for (int i = 0; i < _orbBuffer.size(); ++i) {
                _orbBuffer[i]._time = _lastTime;
            }
            emit newOrbCorrections(_orbBuffer);
            // t_orbCorr::writeEpochSimple(&std::cout, _orbBuffer); // Optional debug
            // std::cout << std::endl;
            _orbBuffer.clear();
        }

        if (!_clkBuffer.isEmpty()) {
             // Update time for all buffered clock corrections to current time
            for (int i = 0; i < _clkBuffer.size(); ++i) {
                _clkBuffer[i]._time = _lastTime;
            }
            emit newClkCorrections(_clkBuffer);
            // t_clkCorr::writeEpochSimple(&std::cout, _clkBuffer); // Optional debug
            // std::cout << std::endl;
            _clkBuffer.clear();
        }
        
        _lastEmitTime = _lastTime;
    }
}

void PPPB2bDecoder::sendResults() {
    // Deprecated or used for full flush?
    // Currently we emit incrementally in emitCorrections called from b2b_fillmem
}
