#include "arc_utilities/timing.hpp"

#include <algorithm>
#include <cstdio>
#include <iostream>

namespace arc_utilities {
double GlobalStopwatch(const StopwatchControl control) {
  static Stopwatch global_stopwatch;
  return global_stopwatch(control);
}
}  // namespace arc_utilities

using namespace arc_utilities;

Profiler* Profiler::m_instance = NULL;

Profiler* Profiler::getInstance() {
  if (m_instance == NULL) {
    m_instance = new Profiler();
  }
  return m_instance;
}

void Profiler::reset_and_preallocate(size_t num_names, size_t num_events) {
  Profiler* monitor = getInstance();
  monitor->data.clear();
  monitor->timed_double_data.clear();
  monitor->prealloc_buffer.resize(num_names);
  for (size_t i = 0; i < num_names; i++) {
    monitor->prealloc_buffer[i].reserve(num_events);
  }
}

void Profiler::reset(std::string name) {
  Profiler* m = getInstance();
  if (m->data.find(name) != m->data.end()) {
    m->data[name].resize(0);
    startTimer(name);
  }
}

void Profiler::addData(std::string name, double datum) {
  Profiler* m = getInstance();
  if (m->data.find(name) == m->data.end()) {
    m->data[name] = std::vector<double>();
    if (m->prealloc_buffer.size() > 0) {
      m->data[name].swap(m->prealloc_buffer.back());
      m->prealloc_buffer.pop_back();
    }
  }
  m->data[name].push_back(datum);
}

void Profiler::startTimer(std::string timer_name) {
  Profiler* m = getInstance();
  if (m->timers.find(timer_name) == m->timers.end()) {
    m->timers[timer_name] = Stopwatch();
  }
  m->timers[timer_name](RESET);
}

bool Profiler::isTimerStarted(std::string timer_name) {
  Profiler* m = getInstance();
  return m->timers.count(timer_name) > 0;
}

double Profiler::record(std::string timer_name) {
  Profiler* m = getInstance();
  if (!m->isTimerStarted(timer_name)) {
    throw std::logic_error("Attempted to record timer " + timer_name + " before starting");
  }

  double time_elapsed = m->timers[timer_name]();
  m->addData(timer_name, time_elapsed);
  return time_elapsed;
}

double Profiler::recordDouble(std::string timer_name, double datum) {
  Profiler* m = getInstance();

  double time_elapsed = 0;
  if (m->isTimerStarted(timer_name)) {
    time_elapsed = m->timers[timer_name]();
  }

  if (m->timed_double_data.find(timer_name) == m->timed_double_data.end()) {
    m->timed_double_data[timer_name] = std::vector<TimedDouble>();
  }
  m->timed_double_data[timer_name].push_back(TimedDouble(time_elapsed, datum));
  return time_elapsed;
}

std::vector<double> Profiler::getData(std::string name) {
  Profiler* m = getInstance();
  return m->data[name];
}

void Profiler::printSingleSummary(std::string name) {
  Profiler* m = getInstance();
  std::string box = std::string(2 + name.length(), '=');
  std::cout << " ." << box << ". "
            << "\n";
  std::cout << "|| " << name << " || Summary :\n";
  std::cout << " '" << box << "' "
            << "\n";
  if (m->data.find(name) == m->data.end()) {
    std::cout << name << " never called\n\n";
    return;
  }

  std::vector<double> data = m->getData(name);

  size_t n = data.size();
  double sum = 0;
  for (auto& num : data) {
    sum += num;
  }

  std::cout << "total time : " << sum << " s\n";
  std::cout << "called " << n << " times\n";
  std::cout << "min time   : " << *std::min_element(data.begin(), data.end()) << "s\n";
  std::cout << "max time   : " << *std::max_element(data.begin(), data.end()) << "s\n";
  std::cout << "average    : " << sum / (double)n << "s\n";

  std::cout << "\n";
}

void Profiler::printGroupSummary(const std::vector<std::string>& names) {
  Profiler* m = getInstance();
  std::cout << " .=======================. \n";
  std::cout << "||    Profile Summary    ||\n";
  std::cout << " '=======================' \n";

  std::size_t label_len =
      max_element(names.begin(), names.end(),
                  [](const std::string& a, const std::string& b) { return a.length() < b.length(); })
          ->length() +
      2;

  label_len = std::max(label_len, (size_t)8);

  const std::string label_format = ("%-" + std::to_string(label_len) + "s");

  printf(label_format.c_str(), "Label");
  printf("%16s", "tot time (s)");
  printf("%16s", "num_calls");
  printf("%16s", "avg time (s)");
  printf("\n");

  std::string seperator = std::string(label_len - 2, '~') + "      " + std::string(12, '.') + "       " +
                          std::string(9, '~') + "    " + std::string(12, '.');

  for (const auto& name : names) {
    if (name.find("~~~~~") == 0) {
      printf("%s\n", seperator.c_str());
      continue;
    }
    printf(label_format.c_str(), name.c_str());
    double tot_time = 0.0;
    double avg_time = 0.0;
    size_t num_calls = 0;
    if (m->data.find(name) != m->data.end()) {
      std::vector<double>& data = m->data[name];
      tot_time = 0;
      for (auto& val : data) {
        tot_time += val;
      }
      num_calls = data.size();
      avg_time = tot_time / (double)num_calls;
    }
    printf(" %15f %15ld %15f\n", tot_time, num_calls, avg_time);
  }
}

std::vector<std::string> Profiler::getAllNames() {
  std::vector<std::string> all_names;
  Profiler* m = getInstance();

  all_names.reserve(m->data.size());

  for (auto const& imap : m->data) {
    all_names.push_back(imap.first);
  }
  for (auto const& imap : m->timed_double_data) {
    all_names.push_back(imap.first);
  }

  std::sort(all_names.begin(), all_names.end());
  return all_names;
}

void Profiler::printAllSummary() { printGroupSummary(getAllNames()); }

void Profiler::writeAllSummary(const std::string& filename) { writeGroupSummary(filename, getAllNames()); }

void addHeader(FILE* outfile) {
  time_t rawtime;
  tm* timeinfo;
  char buffer[80];

  time(&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer, 80, "%Y-%m-%d %H:%M:%S", timeinfo);

  fprintf(outfile, "%s\n\n", buffer);
}

void Profiler::writeGroupSummary(const std::string& filename, const std::vector<std::string>& names) {
  FILE* outfile;
  outfile = std::fopen(filename.c_str(), "a+");

  addHeader(outfile);

  Profiler* m = getInstance();
  // std::cout << " .=======================. \n";
  // std::cout << "||    Profile Summary    ||\n";
  // std::cout << " '=======================' \n";

  std::size_t label_len =
      max_element(names.begin(), names.end(),
                  [](const std::string& a, const std::string& b) { return a.length() < b.length(); })
          ->length() +
      2;

  label_len = std::max(label_len, (size_t)8);

  const std::string label_format = ("%-" + std::to_string(label_len) + "s");

  fprintf(outfile, label_format.c_str(), "Label");
  fprintf(outfile, "%16s", "tot time (s)");
  fprintf(outfile, "%16s", "num_calls");
  fprintf(outfile, "%16s", "avg time (s)");
  fprintf(outfile, "%16s", "total value");
  fprintf(outfile, "%16s", "avg value");
  fprintf(outfile, "\n");

  std::string seperator = std::string(label_len - 2, '~') + "      " + std::string(12, '.') + "       " +
                          std::string(9, '~') + "    " + std::string(12, '.');

  for (const auto& name : names) {
    if (name.find("~~~~~") == 0) {
      fprintf(outfile, "%s\n", seperator.c_str());
      continue;
    }
    fprintf(outfile, label_format.c_str(), name.c_str());
    double tot_time = 0.0;
    double avg_time = 0.0;
    size_t num_calls = 0;
    if (m->data.find(name) != m->data.end()) {
      if (m->timed_double_data.find(name) != m->timed_double_data.end()) {
        std::cout << "!!!\nWarning\n!!!\n, using " << name
                  << " as both time and TimedDouble name. Unsupporred summary output.\n";
      }
      std::vector<double>& data = m->data[name];
      tot_time = 0;
      for (auto& val : data) {
        tot_time += val;
      }
      num_calls = data.size();
      avg_time = tot_time / (double)num_calls;
      fprintf(outfile, " %15f %15ld %15f\n", tot_time, num_calls, avg_time);
    } else if (m->timed_double_data.find(name) != m->timed_double_data.end()) {
      double tot_value = 0;
      double avg_value = 0;
      std::vector<TimedDouble>& data = m->timed_double_data[name];
      for (auto& val : data) {
        tot_time += val.time;
        tot_value += val.value;
      }
      num_calls = data.size();
      avg_time = tot_time / (double)num_calls;
      avg_value = tot_value / (double)num_calls;
      fprintf(outfile, " %15f %15ld %15f %15f %15f\n", tot_time, num_calls, avg_time, tot_value, avg_value);
    } else {
      fprintf(outfile, " %15f %15ld %15f\n", tot_time, num_calls, avg_time);
    }
  }
  fprintf(outfile, "\n");
  std::fclose(outfile);
}

void Profiler::writeAll(const std::string& filename, size_t limit_per_name) {
  Profiler* m = getInstance();

  FILE* outfile;
  outfile = std::fopen(filename.c_str(), "a+");

  addHeader(outfile);

  std::vector<std::string> all_names;
  for (auto const& imap : m->data) {
    all_names.push_back(imap.first);
  }
  for (auto const& imap : m->timed_double_data) {
    all_names.push_back(imap.first);
  }

  std::sort(all_names.begin(), all_names.end());

  std::size_t label_len =
      max_element(all_names.begin(), all_names.end(),
                  [](const std::string& a, const std::string& b) { return a.length() < b.length(); })
          ->length() +
      2;

  label_len = std::max(label_len, (size_t)8);

  const std::string label_format = ("%-" + std::to_string(label_len) + "s");
  fprintf(outfile, label_format.c_str(), "Label");
  fprintf(outfile, "%16s", "time");
  fprintf(outfile, "%16s", "value");
  fprintf(outfile, "\n");

  std::string seperator =
      std::string(label_len - 2, '~') + "      " + std::string(12, '.') + "       " + std::string(9, '~');

  fprintf(outfile, "%s\n", seperator.c_str());

  for (const auto& name : all_names) {
    if (m->data.find(name) != m->data.end() && m->data[name].size() < limit_per_name) {
      for (auto& val : m->data[name]) {
        fprintf(outfile, label_format.c_str(), name.c_str());
        fprintf(outfile, " %15f \n", val);
      }
    }

    if (m->timed_double_data.find(name) != m->timed_double_data.end() &&
        m->timed_double_data[name].size() < limit_per_name) {
      for (auto& val : m->timed_double_data[name]) {
        fprintf(outfile, label_format.c_str(), name.c_str());
        fprintf(outfile, " %15f %15f \n", val.time, val.value);
      }
    }
  }

  fprintf(outfile, "\n");
  std::fclose(outfile);
}
