<<<<<<< HEAD
#include <boost/filesystem.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>

=======
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <stdexcept>
#include <mutex>
#include <boost/filesystem.hpp>
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
#include "arc_utilities/filesystem.hpp"

#ifndef LOG_HPP
#define LOG_HPP

<<<<<<< HEAD
#define ARC_LOG(log, message) \
  (log).logMessage(           \
      static_cast<std::ostringstream&>(std::ostringstream().flush() << std::setprecision(12) << (message)).str())

#define ARC_LOG_STREAM(log, message) \
  (log).logMessage(                  \
      static_cast<std::ostringstream&>(std::ostringstream().flush() << std::setprecision(12) << message).str())

#define ARC_LOG_COND(log, cond, message) \
  if ((cond)) ARC_LOG(log, message)

#define ARC_LOG_COND_STREAM(log, cond, message) \
  if ((cond)) ARC_LOG_STREAM(log, message)

// TODO: confirm that I havn't made any mistakes in this file
namespace Log {
class Log {
 public:
  Log(const std::string& filename, bool add_header = false) : filename_(filename) {
    // Create the parent folder if needed
    boost::filesystem::path p(filename_);
    arc_utilities::CreateDirectory(p.parent_path());

    out_file_.open(filename, std::ios_base::out | std::ios_base::trunc);
    // check if we've succesfully opened the file
    if (!out_file_.is_open()) {
      std::cerr << "\x1b[31;1m Unable to create folder/file to log to: " << filename << "\x1b[37m \n";
      throw std::invalid_argument("filename must be write-openable");
    }

    if (add_header) {
      time_t rawtime;
      tm* timeinfo;
      char buffer[80];

      time(&rawtime);
      timeinfo = localtime(&rawtime);

      strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

      out_file_ << buffer << std::endl;
    }
  }

  /** Copy constructor */
  Log(const Log& other) : filename_(other.filename_), out_file_(filename_, std::ios_base::out | std::ios_base::app) {}

  /** Move constructor */
  Log(Log&& other) : filename_(other.filename_), out_file_(filename_, std::ios_base::out | std::ios_base::app) {
    other.out_file_.close();
  }

  /** Destructor */
  ~Log() {
    if (out_file_.is_open()) {
      out_file_.close();
    }
  }

  /** Copy assignment operator */
  Log& operator=(const Log& other) {
    Log tmp(other);          // re-use copy-constructor
    *this = std::move(tmp);  // re-use move-assignment
    return *this;
  }

  /** Move assignment operator */
  Log& operator=(Log&& other) {
    std::swap(filename_, other.filename_);
    other.out_file_.close();

    if (out_file_.is_open()) {
      out_file_.close();
    }

    out_file_.open(filename_, std::ios_base::out | std::ios_base::app);

    return *this;
  }

  void logMessage(const std::string& message) {
    std::lock_guard<std::mutex> lock(mtx_);
    out_file_ << message << std::endl;
  }

 private:
  std::mutex mtx_;
  std::string filename_;
  std::ofstream out_file_;
};
}  // namespace Log

#endif  // LOG_HPP
=======
#define ARC_LOG(log, message)                   \
    (log).logMessage(                       \
        static_cast<std::ostringstream&>(   \
            std::ostringstream().flush()    \
            << std::setprecision(12)        \
            << (message)                    \
       ).str()                              \
   )

#define ARC_LOG_STREAM(log, message)            \
    (log).logMessage(                       \
        static_cast<std::ostringstream&>(   \
            std::ostringstream().flush()    \
            << std::setprecision(12)        \
            << message                      \
       ).str()                              \
   )

#define ARC_LOG_COND(log, cond, message)        \
    if ((cond)) ARC_LOG(log, message)


#define ARC_LOG_COND_STREAM(log, cond, message) \
    if ((cond)) ARC_LOG_STREAM(log, message)

// TODO: confirm that I havn't made any mistakes in this file
namespace Log
{
    class Log
    {
        public:
            Log(const std::string& filename, bool add_header = false)
                : filename_(filename)
            {
                // Create the parent folder if needed
                boost::filesystem::path p(filename_);
                arc_utilities::CreateDirectory(p.parent_path());

                out_file_.open(filename, std::ios_base::out | std::ios_base::trunc);
                // check if we've succesfully opened the file
                if (!out_file_.is_open())
                {
                    std::cerr << "\x1b[31;1m Unable to create folder/file to log to: " << filename << "\x1b[37m \n";
                    throw std::invalid_argument("filename must be write-openable");
                }

                if (add_header)
                {
                    time_t rawtime;
                    tm * timeinfo;
                    char buffer[80];

                    time(&rawtime);
                    timeinfo = localtime(&rawtime);

                    strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

                    out_file_ << buffer << std::endl;
                }
            }

            /** Copy constructor */
            Log(const Log& other)
                : filename_(other.filename_)
                , out_file_(filename_, std::ios_base::out | std::ios_base::app)
            {
            }

            /** Move constructor */
            Log(Log&& other)
                : filename_(other.filename_)
                , out_file_(filename_, std::ios_base::out | std::ios_base::app)
            {
                other.out_file_.close();
            }

            /** Destructor */
            ~Log()
            {
                if (out_file_.is_open())
                {
                    out_file_.close();
                }
            }

            /** Copy assignment operator */
            Log& operator= (const Log& other)
            {
                Log tmp(other); // re-use copy-constructor
                *this = std::move(tmp); // re-use move-assignment
                return *this;
            }

            /** Move assignment operator */
            Log& operator= (Log&& other)
            {
                std::swap(filename_, other.filename_);
                other.out_file_.close();

                if (out_file_.is_open())
                {
                    out_file_.close();
                }

                out_file_.open(filename_, std::ios_base::out | std::ios_base::app);

                return *this;
            }

            void logMessage(const std::string& message)
            {
                std::lock_guard<std::mutex> lock(mtx_);
                out_file_ << message << std::endl;
            }

        private:
            std::mutex mtx_;
            std::string filename_;
            std::ofstream out_file_;
    };
}

#endif // LOG_HPP
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
