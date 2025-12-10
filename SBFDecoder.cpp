// Part of BNC, a utility for retrieving decoding and
// converting GNSS data streams from NTRIP broadcasters.
//
// Minimal SBF decoder implementation: frame sync and type probing

#include "SBFDecoder.h"
#include "bnccore.h"
#include "PPPB2bDecoder.h"

// SBF sync bytes
static const unsigned char SBF_SYNC1 = 0x24; // '$'
static const unsigned char SBF_SYNC2 = 0x40; // '@'

SBFDecoder::SBFDecoder(const QByteArray &staID) : _staID(staID) {
  _b2bDec = new PPPB2bDecoder();
  if (_b2bDec) _b2bDec->setStaID(_staID);
  if (_b2bDec) _b2bDec->setVerboseSatPrint(false);
}
SBFDecoder::~SBFDecoder() {
  delete _b2bDec;
  _b2bDec = nullptr;
}

quint16 SBFDecoder::U2(const unsigned char *p) {
  quint16 u;
  memcpy(&u, p, 2);
  return u;
}

quint32 SBFDecoder::U4(const unsigned char *p) {
  quint32 u;
  memcpy(&u, p, 4);
  return u;
}
static unsigned getbitu_be(const uint8_t* buff, int pos, int len) {
  unsigned bits = 0;
  for (int i = 0; i < len; ++i) {
    int byte = (pos + i) >> 3;
    int bit  = 7 - ((pos + i) & 7);
    bits = (bits << 1) | ((buff[byte] >> bit) & 1U);
  }
  return bits;
}
static uint32_t crc24q_bits_be(const uint8_t* p, int pos, int len_bits) {
  uint32_t crc = 0;
  for (int i = 0; i < len_bits; ++i) {
    int byte = (pos + i) >> 3;
    int bit  = 7 - ((pos + i) & 7);
    uint32_t inb = (p[byte] >> bit) & 1u;
    crc = ((crc << 1) & 0xFFFFFFu) | inb;
    if (crc & 0x1000000u) crc ^= 0x1864CFBu;
  }
  return crc & 0xFFFFFFu;
}
static quint32 U4BE(const unsigned char* p) {
  return (quint32(p[0]) << 24) | (quint32(p[1]) << 16) | (quint32(p[2]) << 8) | quint32(p[3]);
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
// 将 SVID 映射为各星座的 PRN 掩码。
// 按用户要求：北斗采用 PRN = SVID - 140，掩码为 Cxx。
QString SBFDecoder::svid2prn(quint16 svid) {
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

static const unsigned short CRC_16CCIT_LookUp[256] = {
  0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
  0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
  0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
  0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
  0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
  0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
  0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
  0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
  0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
  0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
  0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
  0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
  0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
  0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
  0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
  0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

unsigned short SBFDecoder::sbf_checksum(const unsigned char *buff, int len) {
  unsigned short crc = 0;
  for (int i = 0; i < len; ++i) {
    crc = (crc << 8) ^ CRC_16CCIT_LookUp[(crc >> 8) ^ buff[i]];
  }
  return crc;
}

bool SBFDecoder::trySync() {
  // ensure the first two bytes are sync sequence
  while (_acc.size() >= 2) {
    const unsigned char *b = reinterpret_cast<const unsigned char *>(_acc.constData());
    if (b[0] == SBF_SYNC1 && b[1] == SBF_SYNC2) return true;
    // drop one byte and retry
    _acc.remove(0, 1);
  }
  return false;
}

bool SBFDecoder::hasWholeFrame(quint16 &len) const {
  if (_acc.size() < 8) return false;
  const unsigned char *b = reinterpret_cast<const unsigned char *>(_acc.constData());
  len = U2(b + 6);
  if (len == 0) return false;
  return _acc.size() >= static_cast<int>(len);
}

bool SBFDecoder::takeOneFrame(QByteArray &frame) {
  quint16 len = 0;
  if (!hasWholeFrame(len)) return false;
  frame = _acc.left(len);
  _acc.remove(0, len);
  return true;
}

t_irc SBFDecoder::Decode(char *buffer, int bufLen, std::vector<std::string> &errmsg) {
  if (!buffer || bufLen <= 0) return failure;



  _acc.append(QByteArray::fromRawData(buffer, bufLen));

  // Align to sync
  if (!trySync()) {
    // Not yet synchronized; keep buffering
    return failure;
  }

  // Process available frames; minimal probing of type
  int frames = 0;
  for (;;) {
    QByteArray frame;
    if (!takeOneFrame(frame)) break;
    frames++;

    const unsigned char *b = reinterpret_cast<const unsigned char *>(frame.constData());
    // type: U2 at offset 4, masked with 0x1FFF per SBF spec
    quint16 type = U2(b + 4) & 0x1FFF;
    quint16 rev  = U2(b + 4) >> 13;
    quint16 crcH = U2(b + 2);
    quint16 len  = U2(b + 6);

    // CRC over bytes [4..len-1]
    if (sbf_checksum(b + 4, len - 4) != crcH) {
      // 简要提示但继续处理下一帧
      if (_logTypes < 3) {
        BNC_CORE->slotMessage(_staID + ": SBF CRC error type=" + QByteArray::number(type), false);
      }
      continue;
    }

    // record type for misc scanning/output (reuse GPSDecoder::_typeList)
    _typeList.push_back(static_cast<int>(type));

    if (_logTypes < 5) {
      QByteArray msg = _staID + ": SBF type=" + QByteArray::number(type) +
                       ", len=" + QByteArray::number(U2(b + 6));
      BNC_CORE->slotMessage(msg, false);
      ++_logTypes;
    }
    if (_b2bDec) _b2bDec->input(b, len);
    // For first-stage goal, only verify header and type; do not parse payload
    // Users can check logs/misc scan that SBF blocks are received.

    // 第二阶段：当为 4242(BDSRawB2b)，将负载传入 PPP_B2bDecoder
    // if (type == 4242) {
    //   const unsigned char *payload = b + 8;
    //   int payload_len = len - 8;

    //   // 打印逻辑：仅当 PRN 为 C59 时打印（不按分钟采样）
    //   if (payload_len >= 12) {
    //     quint32 TOW   = U4(payload + 0) / 1000; // 单位由毫秒转为秒
    //     quint16 WNc   = U2(payload + 4);
    //     quint8  SVIDb = *(payload + 6);
    //     quint8  CRCp  = *(payload + 7);
    //     quint8  Src   = *(payload + 9);
    //     quint8  RxCh  = *(payload + 11);

    //     QString prnMask = svid2prn(SVIDb);
    //     if (prnMask == "C60") {
    //     // 前4个 NAVBits（若长度足够）便于与 SBF 视图对照
    //     QString navBitsStr;
    //     if (payload_len >= 12 + 4 * 4) {
    //       quint32 NB0 = U4(payload + 12 + 0 * 4);
    //       quint32 NB1 = U4(payload + 12 + 1 * 4);
    //       quint32 NB2 = U4(payload + 12 + 2 * 4);
    //       quint32 NB3 = U4(payload + 12 + 3 * 4);
    //       quint32 NB31 = U4(payload + 12 + 30 * 4);
    //       navBitsStr = QString(
    //           ", NAVBits[0..3]=0x%1, 0x%2, 0x%3, 0x%4, 0x%5")
    //           .arg(QString("%1").arg(NB0, 8, 16, QLatin1Char('0')).toUpper())
    //           .arg(QString("%1").arg(NB1, 8, 16, QLatin1Char('0')).toUpper())
    //           .arg(QString("%1").arg(NB2, 8, 16, QLatin1Char('0')).toUpper())
    //           .arg(QString("%1").arg(NB3, 8, 16, QLatin1Char('0')).toUpper())
    //           .arg(QString("%1").arg(NB31, 8, 16, QLatin1Char('0')).toUpper());
    //     }

    //     QString qmsg = QString("%1: B2b_SSR: BDSRawB2b head TOW=%2, WNc=%3, PRN=%4, CRCPassed=%5, Source=%6, RxChannel=%7%8")
    //                        .arg(QString::fromLatin1(_staID))
    //                        .arg(TOW)
    //                        .arg(WNc)
    //                        .arg(prnMask)
    //                        .arg(CRCp)
    //                        .arg(Src)
    //                        .arg(RxCh)
    //                        .arg(navBitsStr);
    //     BNC_CORE->slotMessage(qmsg.toUtf8(), false);

    //     // 对 C59 的 31 个 NAVBits（124 字节，对应 248 个十六进制字符）做一次 LDPC 流水线处理并打印预览
    //     // int navStart = 12;
    //     // int navLenBytes = 31 * 4; // 31 个 32bit NAVBits
    //     // if (payload_len >= navStart + navLenBytes) {
    //     //   QString navHex;
    //     //   navHex.reserve(navLenBytes * 2);
    //     //   for (int w = 0; w < 31; ++w) {
    //     //     quint32 NBw = U4(payload + navStart + w * 4);
    //     //     navHex += QString("%1").arg(NBw, 8, 16, QLatin1Char('0')).toUpper();
    //     //   }

    //     //   QByteArray decoded = SBFcoDecoder::decode_LDPC_navbitsRaw(navHex.toUtf8());
    //     //   BNC_CORE->slotMessage(navHex.toUtf8(), false);
    //     //   int previewN = qMin(decoded.size(), 124);
    //     //   QByteArray preview = decoded.left(previewN);
    //     //   QString previewEsc;
    //     //   previewEsc.reserve(preview.size() * 4 + 2);
    //     //   previewEsc += "b'";
    //     //   for (int i = 0; i < preview.size(); ++i) {
    //     //     previewEsc += "\\x";
    //     //     previewEsc += QString("%1").arg((unsigned char)preview.at(i), 2, 16, QLatin1Char('0')).toUpper();
    //     //   }
    //     //   previewEsc += "'";
    //     //   QString out = QString("C59 NAVBits decoded preview (%1 bytes): %2")
    //     //                   .arg(previewN)
    //     //                   .arg(toRawArray(parseBytesLiteral(previewEsc)));
    //     //   BNC_CORE->slotMessage(out.toUtf8(), false);
    //     //  QByteArray cssrBuf = parseBytesLiteral(previewEsc);
    //     //  CSSRDecoder cs;
    //     //  cs.setEpoch(WNc, TOW);
    //     //  int mt = cs.decode_cssr(reinterpret_cast<const uint8_t*>(cssrBuf.constData()), cssrBuf.size(), 0);
    //     //  QString mtMsg = QString("CSSR mt=%1").arg(mt);
    //     //  BNC_CORE->slotMessage(mtMsg.toUtf8(), true);
    //     //  if (mt == 1) {
    //     //    bool ok = cs.self_check_mt1();
    //     //    QString chk = QString("MT1 self-check %1").arg(ok ? "OK" : "NG");
    //     //    BNC_CORE->slotMessage(chk.toUtf8(), false);
    //     //  }
    //     // }
    //     }
    //   }
    // }
  }
  if (frames > 0) {
    _totalFrames += frames;
    QByteArray msg = _staID + ": B2b_SSR: SBF frames received: " + QByteArray::number(frames)
                     + ", total=" + QByteArray::number(_totalFrames);
    BNC_CORE->slotMessage(msg, false);
    return success;
  }

  return failure;
}
