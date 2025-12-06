// Part of BNC, a utility for retrieving decoding and
// converting GNSS data streams from NTRIP broadcasters.
//
// Minimal SBF decoder: frame sync and type probing
//
// This file introduces a lightweight SBFDecoder that only performs
// Septentrio SBF header synchronization and block type extraction,
// returning success for recognized blocks and failure otherwise.

#ifndef INC_SBFDECODER_H
#define INC_SBFDECODER_H

#include <QtCore>
#include <vector>
#include <string>

#include "GPSDecoder.h"
#include "PPPB2bDecoder.h"

class SBFDecoder : public GPSDecoder {
public:
  explicit SBFDecoder(const QByteArray &staID);
  ~SBFDecoder();

  // Minimal Decode: detect SBF sync (0x24,0x40), read length at [6..7],
  // check available bytes, then extract block type (U2 at [4] & 0x1FFF).
  virtual t_irc Decode(char *buffer, int bufLen,
                       std::vector<std::string> &errmsg) override;

  // 将 SVID 转换为 PRN 掩码字符串（Gxx/Rxx/Cxx/Exx/Sxx/Jxx），超出范围则返回 UNK_<SVID>
  static QString svid2prn(quint16 svid);
  // 将字节数组转换为十六进制字符串（每个字节用空格分隔）

  PPPB2bDecoder* getB2bDecoder() const { return _b2bDec; }

private:
  QByteArray _staID;
  QByteArray _acc; // accumulate bytes across calls
  int        _logTypes = 0; // limit type logging
  PPPB2bDecoder* _b2bDec = nullptr;
  quint64     _totalB2b = 0;
  int         _logB2b = 0;

  bool trySync();
  bool hasWholeFrame(quint16 &len) const;
  bool takeOneFrame(QByteArray &frame);
  static quint16 U2(const unsigned char *p);
  static quint32 U4(const unsigned char *p);
  static unsigned short sbf_checksum(const unsigned char *buff, int len);
  // 识别阶段统计
  quint64         _totalFrames = 0;
  // 采样打印：每分钟打印最多10帧负载
  qint64          _lastMinuteKey = -1; // 秒/60
  int             _minutePrintCnt = 0; // 该分钟内已打印的帧数
  int             _sampleBytesMax = 256; // 每帧打印的最大字节数（过长截断）
};

#endif // INC_SBFDECODER_H