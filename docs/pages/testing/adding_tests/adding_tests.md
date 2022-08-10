---
title: Adding gtest Tests
nav_order: 1
parent: Testing
---

# Adding gtest Tests

This project uses the `googletest` (`gtest` for short) framework for testing. This is the process of writing a new test suite:

Note: Test suite is used to refer to a new group of tests indepenedent from other tests. In this repository, we're making a new header file for each test suite and writing a new test suite for each class.

1. Make a new header file in the `cdcpd/tests/include` directory of the repository.
  - The name of the new header file should be the name of the class (or functionality) under test concatenated with Test.h, e.g. if you're testing a class named `MyClass`, the name of the header file would be `MyClassTest.h`.
2. In the new <test_suite>.h file, write the following as boilerplate:
    ```
    #include "<where_the_class_you're_testing_is_declared>"
    #include "gtest/gtest.h"

    // insert any test fixtures here.

    TEST(<test_suite_name>, <test_name>) {
      // insert code for test here using:
      //    ASSERT_*(x, y) when you want the test to halt on failure.
      //    EXPECT_*(x, y) when you want the test to continue upon failure.
      //                   The EXPECT_* is best practice.
    }
    ```
    where the "`*`" in `ASSERT_*` and `EXPECT_*` is meant to be filled in with whatever you would like to assert or expect, e.g. `EXPECT_TRUE(XXX)` if `XXX` should evaluate to be `true`.
    - Note that neither `<test_suite_name>` nor `<test_name>` should have any underscores in the name. This doesn't play nicely with `gtest`'s internal macro naming scheme.
    - You can read more about test fixtures in the google test documentation. They're *very* handy!
3. Write the first test that you would like.
4. In tests/main.cpp, `#include` your new test suite header file.

From here, the `cdcpd/tests/main.cpp` file will automatically discover all tests in your newly written test suite. No need to manually add function calls!