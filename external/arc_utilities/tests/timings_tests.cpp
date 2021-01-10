
#define ENABLE_PROFILING
#include <gtest/gtest.h>

#include <thread>

#include "arc_utilities/timing.hpp"

using namespace arc_utilities;

TEST(TimerTest, Timer_is_monotonically_increasing_with_sequential_records) {
  Profiler::reset_and_preallocate(10, 10);
  Profiler::startTimer("timer1");
  double t_elapsed = Profiler::record("timer1");
  EXPECT_TRUE(t_elapsed > 0) << "Timer recorded negative elapsed time";

  for (size_t i = 2; i < 100; i++) {
    Profiler::record("timer1");
    std::vector<double> times = Profiler::getData("timer1");
    ASSERT_EQ(i, times.size()) << "Num events and number of times getData called do not match";
    EXPECT_TRUE(times[i - 1] > times[i - 2]) << "Time not monotonitcally increasing";
  }
}

TEST(TimerTest, Timer_is_approximately_accurate) {
  Profiler::startTimer("sleepy");
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  double t_elapsed = Profiler::record("sleepy");
  EXPECT_TRUE(t_elapsed < 0.052) << "Sleep for 50ms took too long";
  EXPECT_TRUE(t_elapsed > 0.0500) << "Recorded time less than sleep time";
}

TEST(TimerTest, Multiple_timers_can_run_simultaneously) {
  Profiler::reset_and_preallocate(10, 10);
  Profiler::startTimer("timer1");
  Profiler::startTimer("timer2");
  double t2_elapsed = Profiler::record("timer2");
  double t1_elapsed = Profiler::record("timer1");
  EXPECT_GT(t1_elapsed, t2_elapsed) << "Timer1 started first but not registering longer time";
  EXPECT_GT(t1_elapsed, 0) << "Timer 1 recorded non-positive elapsed time";
  EXPECT_GT(t2_elapsed, 0) << "Timer 2 recorded non-positive elapsed time";
}

TEST(TimerTest, startTimer_restarts_the_timer_from_zero) {
  int sleep_s = 10;
  Profiler::reset_and_preallocate(10, 10);
  Profiler::startTimer("timer1");
  Profiler::startTimer("timer2");
  std::this_thread::sleep_for(std::chrono::seconds(sleep_s));

  EXPECT_GE(Profiler::record("timer1"), sleep_s) << "Slept for " << sleep_s << " seconds. Inconsistent with timer 1";
  Profiler::startTimer("timer1");
  double t1_elapsed_restarted = Profiler::record("timer1");
  EXPECT_LT(t1_elapsed_restarted, sleep_s) << "Restarted timer 1, but long time measured";

  EXPECT_GE(Profiler::record("timer2"), sleep_s)
      << "Slept for " << sleep_s << " seconds. Inconsistent with timer2 after resetting timer 1";
  Profiler::startTimer("timer2");
  double t2_elapsed_restarted = Profiler::record("timer2");
  EXPECT_LT(t2_elapsed_restarted, sleep_s) << "Restarted timer 2, but long time measured";
}

TEST(TimerTest, recording_before_starting_throws_error) {
  PROFILE_START("Started timer");
  EXPECT_NO_THROW(PROFILE_RECORD("Started timer"));
  EXPECT_THROW(PROFILE_RECORD("Unstarted timer"), std::logic_error)
      << "Attempting to record a timer before starting should throw an error";
}

TEST(TimerTest, PROFILE_COMMAND_macros_run) {
  PROFILE_REINITIALIZE(100, 100);
  PROFILE_START("testmacro1");
  PROFILE_START("testmacro2");
  PROFILE_RECORD("testmacro2");
  PROFILE_RECORD("testmacro1");

  // Uncomment this for viewing output.
  // Commented as to not pollute the testing output
  // PROFILE_PRINT_SUMMARY_FOR_GROUP("testmacro1");

  double t1_elapsed = Profiler::getData("testmacro1")[0];
  double t2_elapsed = Profiler::getData("testmacro2")[0];
  EXPECT_GT(t1_elapsed, t2_elapsed) << "Macro timer1 started first but recorded less time";
}

TEST(TimerTest, Printing_to_screen_runs_without_error) {
  PROFILE_REINITIALIZE(100, 1000);
  PROFILE_START("name_01");
  PROFILE_START("name_02");
  PROFILE_START("really really really long name");
  PROFILE_RECORD("testmacro2");
  PROFILE_RECORD("testmacro1");
  for (int i = 0; i < 100; i++) {
    PROFILE_RECORD("really really really long name");
  }
  std::vector<std::string> names = {"name_01", "unused name", "~~~~~~~~~~~~~~", "really really really long name",
                                    "name_02"};

  // Uncomment this for viewing output.
  // Commented as to not pollute the testing output
  // PROFILE_PRINT_SUMMARY_FOR_GROUP(names);
  // TODO: Automate validation of summary text
}

TEST(TimerTest, Writing_summary_to_file_runs_without_error) {
  PROFILE_REINITIALIZE(100, 100);
  PROFILE_START("one");
  PROFILE_START("two");
  PROFILE_START("three");
  PROFILE_START("double values");
  PROFILE_RECORD("one");
  PROFILE_RECORD("two");
  PROFILE_RECORD("three");
  PROFILE_RECORD_DOUBLE("double values", 3.2);
  PROFILE_RECORD_DOUBLE("double values", -1.966);

  PROFILE_RECORD_DOUBLE("unstarted double", 1.2345);

  // TODO: Automate validation of summary text
  PROFILE_WRITE_SUMMARY_FOR_ALL("testing_output.txt");
}

TEST(TimerTest, Writing_all_to_file_runs_without_error) {
  PROFILE_REINITIALIZE(100, 100);
  PROFILE_START("one");
  PROFILE_START("two");
  PROFILE_START("three");
  PROFILE_START("double values");
  PROFILE_RECORD("one");
  PROFILE_RECORD("two");
  PROFILE_RECORD("three");
  PROFILE_RECORD_DOUBLE("double values", 1.1);
  PROFILE_RECORD_DOUBLE("double values", 2.2);
  PROFILE_RECORD_DOUBLE("double values", 3.3);
  PROFILE_RECORD_DOUBLE("double values", 4.4);
  PROFILE_RECORD_DOUBLE("double values", 5.5);

  // TODO: Automate validation of summary text
  PROFILE_WRITE_ALL("testing_output_full.txt");
  PROFILE_WRITE_ALL_FEWER_THAN("testing_output_limited.txt", 3);
}

GTEST_API_ int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
