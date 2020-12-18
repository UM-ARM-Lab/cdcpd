<<<<<<< HEAD
#include <stdio.h>
#include <stdlib.h>
#include <zlib.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
=======
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default

#ifndef BASE64_HELPERS_HPP
#define BASE64_HELPERS_HPP

<<<<<<< HEAD
namespace Base64Helpers {
std::vector<uint8_t> Decode(const std::string& encoded);

std::string Encode(const std::vector<uint8_t>& binary);
}  // namespace Base64Helpers

#endif  // BASE64_HELPERS_HPP
=======
namespace Base64Helpers
{
    std::vector<uint8_t> Decode(const std::string& encoded);

    std::string Encode(const std::vector<uint8_t>& binary);
}

#endif // BASE64_HELPERS_HPP
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
