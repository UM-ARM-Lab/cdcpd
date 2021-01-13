#include <stdio.h>
#include <stdlib.h>
#include <zlib.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef BASE64_HELPERS_HPP
#define BASE64_HELPERS_HPP

namespace Base64Helpers {
std::vector<uint8_t> Decode(const std::string& encoded);

std::string Encode(const std::vector<uint8_t>& binary);
}  // namespace Base64Helpers

#endif  // BASE64_HELPERS_HPP
