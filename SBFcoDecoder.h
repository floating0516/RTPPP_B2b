#ifndef SBF_CO_DECODER_H
#define SBF_CO_DECODER_H

#include <QByteArray>
#include <QString>
#include <vector>
#include <utility>

class SBFcoDecoder {
public:
  static QByteArray decode_LDPC_navbitsRaw(const QByteArray& navBits);


private:
  static std::vector<uint8_t> hexToBytesSanitized(const QString& hex);
  static QString bytesToHex(const std::vector<uint8_t>& bytes);
  static std::vector<uint8_t> bytesToBitsBE(const std::vector<uint8_t>& bytes);
  static std::vector<uint8_t> bitsToBytesBE(const std::vector<uint8_t>& bits);
  static QString bitsToHex(const std::vector<uint8_t>& bits);
  static std::pair<std::vector<uint8_t>, int> decode_LDPC_BCNV3(const std::vector<uint8_t>& errData);
  static std::vector<uint8_t> read_hex_bits(const QString& hex);
  static QString hex_str_from_bits(const std::vector<uint8_t>& bits);
  static std::pair<std::vector<uint8_t>, int> decode_LDPC_BCNV3_bits(const std::vector<std::vector<int>>& H_idx,
                                                                      const std::vector<std::vector<uint8_t>>& H_ele,
                                                                      int m,
                                                                      int n,
                                                                      const std::vector<uint8_t>& syms);

};

#endif