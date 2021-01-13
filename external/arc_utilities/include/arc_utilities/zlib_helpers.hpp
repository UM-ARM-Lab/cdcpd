#include <stdio.h>
#include <stdlib.h>
#include <zlib.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef ZLIB_HELPERS_HPP
#define ZLIB_HELPERS_HPP

namespace ZlibHelpers {
std::vector<uint8_t> DecompressBytes(const std::vector<uint8_t>& compressed);

std::vector<uint8_t> CompressBytes(const std::vector<uint8_t>& uncompressed);

std::vector<uint8_t> LoadFromFileAndDecompress(const std::string& path);

void CompressAndWriteToFile(const std::vector<uint8_t>& uncompressed, const std::string& path);
}  // namespace ZlibHelpers

#endif  // ZLIB_HELPERS_HPP
