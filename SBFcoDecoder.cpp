#include "SBFcoDecoder.h"

#include <cstring>
#include <cctype>
#include <cmath>
#include <algorithm>

// Helpers similar to Python read_hex, hex_str, and sdr_ldpc.decode_LDPC

static inline uint8_t hexNibble(QChar c) {
  if (c >= '0' && c <= '9') return uint8_t(c.unicode() - '0');
  if (c >= 'a' && c <= 'f') return uint8_t(c.unicode() - 'a' + 10);
  if (c >= 'A' && c <= 'F') return uint8_t(c.unicode() - 'A' + 10);
  return 0;
}

std::vector<uint8_t> SBFcoDecoder::hexToBytesSanitized(const QString& hex) {
  QString s;
  s.reserve(hex.size());
  for (int i = 0; i < hex.size(); ++i) {
    QChar c = hex.at(i);
    if (c.isSpace()) continue;
    if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
      s.append(c);
    }
  }
  if (s.size() % 2 == 1) {
    s.append('0');
  }
  std::vector<uint8_t> out;
  out.reserve(s.size() / 2);
  for (int i = 0; i + 1 < s.size(); i += 2) {
    uint8_t hi = hexNibble(s.at(i));
    uint8_t lo = hexNibble(s.at(i + 1));
    out.push_back(uint8_t((hi << 4) | lo));
  }
  return out;
}

QString SBFcoDecoder::bytesToHex(const std::vector<uint8_t>& bytes) {
  static const char* lut = "0123456789abcdef";
  QString out;
  out.reserve(bytes.size() * 2);
  for (uint8_t b : bytes) {
    out.append(lut[(b >> 4) & 0xF]);
    out.append(lut[b & 0xF]);
  }
  return out;
}

std::vector<uint8_t> SBFcoDecoder::bytesToBitsBE(const std::vector<uint8_t>& bytes) {
  std::vector<uint8_t> bits;
  bits.reserve(bytes.size() * 8);
  for (uint8_t b : bytes) {
    for (int i = 7; i >= 0; --i) {
      bits.push_back(uint8_t((b >> i) & 0x1));
    }
  }
  return bits;
}

std::vector<uint8_t> SBFcoDecoder::bitsToBytesBE(const std::vector<uint8_t>& bits) {
  size_t n = bits.size();
  size_t nb = (n + 7) / 8;
  std::vector<uint8_t> out(nb, 0);
  for (size_t i = 0; i < n; ++i) {
    size_t byteIndex = i / 8;
    int bitPos = 7 - int(i % 8);
    out[byteIndex] |= uint8_t(bits[i] & 1) << bitPos;
  }
  return out;
}

QString SBFcoDecoder::bitsToHex(const std::vector<uint8_t>& bits) {
  return bytesToHex(bitsToBytesBE(bits));
}



QByteArray SBFcoDecoder::decode_LDPC_navbitsRaw(const QByteArray& navBits) {
  // 输入即 16 进制文本（例如 248 个十六进制字符）。
  // 1) 转为 UTF-8 字符串
  QString navHex = QString::fromUtf8(navBits);
  // 2) 去掉末尾两个十六进制字符
  if (navHex.size() >= 2) {
    navHex.chop(2);
  }
  // 3) 读取 16 进制内容为比特数组（按 nibble 映射）
  std::vector<uint8_t> bits = read_hex_bits(navHex);
  // 4) 丢弃前 12 个比特
  if (bits.size() >= 12) {
    bits.erase(bits.begin(), bits.begin() + 12);
  } else {
    bits.clear();
  }
  {
    QString bitsStr;
    bitsStr.reserve(bits.size());
    // for (size_t i = 0; i < bits.size(); ++i) bitsStr.append(bits[i] ? '1' : '0');
    // fprintf(stderr, "[B2b] bits after drop12 (len=%zu): %s\n", bits.size(), bitsStr.toUtf8().constData());
  }
  
  // 5) 占位 LDPC 解码
  // 构建 BCNV3 矩阵并解码（162,81）
  static const int N_GF = 6;
  static const int M = 81;
  static const int N = 162;
  // H_BCNV3_idx/H_BCNV3_ele 按规范 [6] 6.2.2
  static const int H_idx_raw[81][4] = {
    {19,67,109,130},{27,71,85,161},{31,78,96,122},{2,44,83,125},
    {26,71,104,132},{30,39,93,154},{4,46,85,127},{21,62,111,127},
    {13,42,101,146},{18,66,108,129},{27,72,100,153},{29,70,84,160},
    {23,61,113,126},{8,50,89,131},{34,74,111,157},{12,44,100,145},
    {22,60,112,128},{0,49,115,151},{6,47,106,144},{33,53,82,140},
    {3,45,84,126},{38,80,109,147},{9,60,96,141},{1,43,82,124},
    {20,77,88,158},{37,54,122,159},{3,65,104,149},{5,47,86,128},
    {0,42,81,123},{32,79,97,120},{35,72,112,158},{15,57,93,138},
    {22,75,107,143},{24,69,102,133},{1,50,116,152},{24,57,119,135},
    {17,59,95,140},{7,45,107,145},{34,51,83,138},{14,43,99,144},
    {21,77,106,142},{16,58,94,139},{20,68,110,131},{2,48,114,150},
    {10,52,91,133},{25,70,103,134},{32,41,95,153},{14,56,91,137},
    {33,73,113,156},{28,73,101,154},{4,63,102,147},{6,48,87,129},
    {8,46,105,146},{30,80,98,121},{41,68,119,150},{35,52,81,139},
    {16,63,114,124},{13,55,90,136},{31,40,94,155},{10,61,97,142},
    {36,56,121,161},{29,74,99,155},{5,64,103,148},{18,75,89,156},
    {36,78,110,148},{19,76,87,157},{15,65,116,123},{11,53,92,134},
    {25,58,117,136},{39,66,117,151},{11,62,98,143},{9,51,90,132},
    {38,55,120,160},{7,49,88,130},{17,64,115,125},
    {28,69,86,159},{23,76,105,141},{12,54,92,135},
    {40,67,118,152},{37,79,108,149},{26,59,118,137}
  };
  static const uint8_t H_ele_raw[81][4] = {
    {46,45,44,15},{15,24,50,37},{24,50,37,15},{15,32,18,61},
    {58,56,60,62},{37,53,61,29},{46,58,18,6},{36,19,3,57},
    {54,7,38,23},{51,59,63,47},{9,3,43,29},{56,8,46,13},
    {26,22,14,2},{63,26,41,12},{17,32,58,37},{38,23,55,22},
    {35,1,31,44},{44,51,35,13},{30,1,44,7},{27,5,2,62},
    {16,63,20,9},{27,56,8,43},{1,44,30,24},{5,26,27,37},
    {42,47,37,32},{38,12,25,51},{43,34,48,57},{39,9,30,48},
    {63,13,54,10},{2,46,56,35},{47,20,33,26},{62,54,56,60},
    {1,21,25,7},{43,58,19,49},{28,4,52,44},{46,44,14,15},
    {41,48,2,27},{49,21,7,35},{40,21,44,17},{24,23,45,11},
    {46,25,22,48},{13,29,53,61},{52,17,24,61},{29,41,10,16},
    {60,24,4,50},{32,49,58,19},{43,34,48,57},{29,7,10,16},
    {25,11,7,1},{32,49,58,19},{42,14,24,33},{39,56,30,48},
    {13,27,56,8},{53,40,61,18},{8,43,27,56},{18,40,32,61},
    {60,48,2,27},{50,54,60,62},{58,19,32,49},{9,3,63,43},
    {53,35,16,13},{23,25,30,16},{18,6,61,21},{15,1,42,45},
    {20,16,63,9},{27,37,5,26},{29,7,10,16},{11,60,6,49},
    {43,47,18,20},{42,14,24,33},{43,22,41,20},{22,15,12,33},
    {9,41,57,58},{5,31,51,30},{9,3,63,43},
    {37,53,61,29},{6,45,56,19},{33,45,36,34},
    {19,24,42,14},{1,45,15,6},{8,43,27,56}
  };
  std::vector<std::vector<int>> H_idx(M, std::vector<int>(4));
  std::vector<std::vector<uint8_t>> H_ele(M, std::vector<uint8_t>(4));
  for (int i = 0; i < M; ++i) {
    for (int j = 0; j < 4; ++j) { H_idx[i][j] = H_idx_raw[i][j]; H_ele[i][j] = H_ele_raw[i][j]; }
  }
  auto decPair = decode_LDPC_BCNV3_bits(H_idx, H_ele, M, N, bits);
  std::vector<uint8_t>& decBits = decPair.first;
  // 6) 将比特转回十六进制，并在奇数长度时补齐到偶数后 unhexlify
  QString hexTxt = hex_str_from_bits(decBits);
  if (hexTxt.size() % 2 == 1) {
    hexTxt.append('0');
  }
  std::vector<uint8_t> outBytes = hexToBytesSanitized(hexTxt);
  return QByteArray(reinterpret_cast<const char*>(outBytes.data()), int(outBytes.size()));
}

// Python-equivalent helpers for nibble-wise hex<->bits
std::vector<uint8_t> SBFcoDecoder::read_hex_bits(const QString& hex) {
  // N = len(str0) * 4; data[i] = (int(str0[i // 4], 16) >> (3 - i % 4)) & 1
  QString s;
  s.reserve(hex.size());
  for (int i = 0; i < hex.size(); ++i) {
    QChar c = hex.at(i);
    if (c.isSpace()) continue;
    if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
      s.append(c);
    }
  }
  int N = s.size() * 4;
  std::vector<uint8_t> bits;
  bits.resize(N);
  for (int i = 0; i < N; ++i) {
    int idx = i / 4;
    uint8_t nib = hexNibble(s.at(idx));
    int shift = 3 - (i % 4);
    bits[i] = (nib >> shift) & 1;
  }
  return bits;
}

QString SBFcoDecoder::hex_str_from_bits(const std::vector<uint8_t>& bits) {
  // Build hex string from bit array, 4 bits per hex digit, MSB-first in each nibble
  QString out;
  int hex = 0;
  for (size_t i = 0; i < bits.size(); ++i) {
    hex = (hex << 1) + (bits[i] & 1);
    if (i % 4 == 3) {
      out += QString::number(hex, 16).toUpper();
      hex = 0;
    }
  }
  int rem = bits.size() % 4;
  if (rem != 0) {
    hex <<= (4 - rem);
    out += QString::number(hex, 16).toUpper();
  }
  return out;
}

std::pair<std::vector<uint8_t>, int> SBFcoDecoder::decode_LDPC_BCNV3_bits(const std::vector<std::vector<int>>& H_idx,
                                                                          const std::vector<std::vector<uint8_t>>& H_ele,
                                                                          int m,
                                                                          int n,
                                                                          const std::vector<uint8_t>& syms) {
  const int N_GF = 6;
  const int Q_GF = 64;
  const int MAX_ITER = 15;
  const int NM_EMS = 4;
  const double ERR_PROB = 1e-5;
  static const uint8_t GF_VEC[63] = {1,2,4,8,16,32,3,6,12,24,48,35,5,10,20,40,19,38,15,30,60,59,53,41,17,34,7,14,28,56,51,37,9,18,36,11,22,44,27,54,47,29,58,55,45,25,50,39,13,26,52,43,21,42,23,46,31,62,63,61,57,49,33};
  static const uint8_t GF_POW[64] = {0,0,1,6,2,12,7,26,3,32,13,35,8,48,27,18,4,24,33,16,14,52,36,54,9,45,49,38,28,41,19,56,5,62,25,11,34,31,17,47,15,23,53,51,37,44,55,40,10,61,46,30,50,22,39,43,29,60,42,21,20,59,57,58};
  static std::vector<std::vector<uint8_t>> GF_MUL;
  if (GF_MUL.empty()) {
    GF_MUL.assign(Q_GF, std::vector<uint8_t>(Q_GF, 0));
    for (int i = 1; i < Q_GF; ++i) {
      for (int j = 1; j < Q_GF; ++j) {
        GF_MUL[i][j] = GF_VEC[(GF_POW[i] + GF_POW[j]) % (Q_GF - 1)];
      }
    }
  }
  int nvars = int(syms.size()) / N_GF;
  if (nvars <= 0) return {std::vector<uint8_t>(), 0};
  std::vector<uint8_t> code(nvars, 0);
  for (int i = 0; i < nvars; ++i) {
    uint8_t v = 0;
    for (int j = 0; j < N_GF; ++j) v = uint8_t((v << 1) | (syms[i * N_GF + j] & 1));
    code[i] = v;
  }
  auto gf2bin = [&](const std::vector<uint8_t>& c) {
    std::vector<uint8_t> syms(c.size() * N_GF, 0);
    for (size_t i = 0; i < c.size(); ++i) {
      for (int j = 0; j < N_GF; ++j) syms[i * N_GF + j] = uint8_t((c[i] >> (N_GF - 1 - j)) & 1);
    }
    return syms;
  };
  auto bitcnt6 = [&](uint8_t x) { return int(__builtin_popcount(uint32_t(x & 0x3F))); };
  std::vector<int> ie;
  std::vector<int> je;
  std::vector<uint8_t> he;
  int ne = 0;
  std::vector<std::vector<float>> V2C;
  std::vector<std::vector<float>> C2V;
  std::vector<std::vector<float>> L(nvars, std::vector<float>(Q_GF, 0.0f));
  for (int i = 0; i < nvars; ++i) {
    for (int j = 0; j < Q_GF; ++j) {
      L[i][j] = float(-std::log(ERR_PROB) * bitcnt6(uint8_t(code[i] ^ j)));
    }
  }
  for (int i = 0; i < (int)H_idx.size(); ++i) {
    for (int j = 0; j < (int)H_idx[i].size(); ++j) {
      ie.push_back(i);
      je.push_back(H_idx[i][j]);
      he.push_back(H_ele[i][j]);
    }
  }
  ne = int(ie.size());
  V2C.assign(ne, std::vector<float>(Q_GF, 0.0f));
  C2V.assign(ne, std::vector<float>(Q_GF, 0.0f));
  for (int i = 0; i < ne; ++i) {
    for (int x = 0; x < Q_GF; ++x) V2C[i][GF_MUL[he[i]][x]] = L[je[i]][x];
  }
  auto check_parity = [&]() {
    std::vector<uint8_t> s(m, 0);
    for (int i = 0; i < ne; ++i) s[ie[i]] ^= GF_MUL[he[i]][code[je[i]]];
    for (int i = 0; i < m; ++i) if (s[i] != 0) return false;
    return true;
  };
  auto ext_min_sum = [&](const std::vector<float>& A, const std::vector<float>& B) {
    if (A.empty()) return B;
    std::vector<int> idxA(Q_GF), idxB(Q_GF);
    for (int i = 0; i < Q_GF; ++i) { idxA[i] = i; idxB[i] = i; }
    std::sort(idxA.begin(), idxA.end(), [&](int a, int b){ return A[a] < A[b]; });
    std::sort(idxB.begin(), idxB.end(), [&](int a, int b){ return B[a] < B[b]; });
    float maxL = A[idxA[NM_EMS - 1]] + B[idxB[NM_EMS - 1]];
    std::vector<float> Ls(Q_GF, maxL);
    for (int ia = 0; ia < NM_EMS; ++ia) {
      int iA = idxA[ia];
      for (int ib = 0; ib < NM_EMS; ++ib) {
        int iB = idxB[ib];
        int idx = iA ^ iB;
        float v = A[iA] + B[iB];
        if (v < Ls[idx]) Ls[idx] = v;
      }
    }
    return Ls;
  };
  for (int it = 0; it < MAX_ITER; ++it) {
    if (check_parity()) {
      std::vector<uint8_t> syms_dec = gf2bin(code);
      int nerr = 0;
      size_t Lmin = std::min(syms_dec.size(), syms.size());
      for (size_t i = 0; i < Lmin; ++i) if ((syms_dec[i] ^ syms[i]) & 1) ++nerr;
      return {syms_dec, nerr};
    }
    for (int i = 0; i < ne; ++i) {
      std::vector<float> Ls;
      for (int j = 0; j < ne; ++j) if (ie[i] == ie[j] && i != j) Ls = ext_min_sum(Ls, V2C[j]);
      float mn = *std::min_element(Ls.begin(), Ls.end());
      for (float& v : Ls) v -= mn;
      for (int x = 0; x < Q_GF; ++x) C2V[i][x] = Ls[GF_MUL[he[i]][x]];
    }
    for (int i = 0; i < ne; ++i) {
      std::vector<float> Ls = L[je[i]];
      for (int j = 0; j < ne; ++j) if (je[i] == je[j] && i != j) for (int x = 0; x < Q_GF; ++x) Ls[x] += C2V[j][x];
      float mn = *std::min_element(Ls.begin(), Ls.end());
      for (float& v : Ls) v -= mn;
      for (int x = 0; x < Q_GF; ++x) V2C[i][GF_MUL[he[i]][x]] = Ls[x];
    }
    for (int i = 0; i < nvars; ++i) {
      std::vector<float> Li = L[i];
      for (int j = 0; j < ne; ++j) if (i == je[j]) for (int x = 0; x < Q_GF; ++x) Li[x] += C2V[j][x];
      float mn = *std::min_element(Li.begin(), Li.end());
      for (float& v : Li) v -= mn;
      int argmin = 0;
      for (int x = 1; x < Q_GF; ++x) if (Li[x] < Li[argmin]) argmin = x;
      code[i] = uint8_t(argmin);
    }
  }
  std::vector<uint8_t> syms_dec = gf2bin(code);
  return {syms_dec, -1};
}

