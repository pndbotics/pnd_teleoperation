#include "adam_retarget/utils.hpp"

#include <gtest/gtest.h>

TEST(adam_retarget, transform) {
  casadi::DM T = transform(1, 2, 3, 0, 0, 0, 1);
  ASSERT_EQ(T(0, 0).scalar(), 1);
  ASSERT_EQ(T(0, 1).scalar(), 0);
  ASSERT_EQ(T(0, 2).scalar(), 0);
  ASSERT_EQ(T(0, 3).scalar(), 1);
  ASSERT_EQ(T(1, 0).scalar(), 0);
  ASSERT_EQ(T(1, 1).scalar(), 1);
  ASSERT_EQ(T(1, 2).scalar(), 0);
  ASSERT_EQ(T(1, 3).scalar(), 2);
  ASSERT_EQ(T(2, 0).scalar(), 0);
  ASSERT_EQ(T(2, 1).scalar(), 0);
  ASSERT_EQ(T(2, 2).scalar(), 1);
  ASSERT_EQ(T(2, 3).scalar(), 3);
  ASSERT_EQ(T(3, 0).scalar(), 0);
  ASSERT_EQ(T(3, 1).scalar(), 0);
  ASSERT_EQ(T(3, 2).scalar(), 0);
  ASSERT_EQ(T(3, 3).scalar(), 1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}