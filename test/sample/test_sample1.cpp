#include "gtest/gtest.h"
#include <string>
#include <iostream>

namespace {

// テスト対象となるクラス Foo のためのフィクスチャ
class SampleFooTest : public ::testing::Test {
 protected:
  // 以降の関数で中身のないものは自由に削除できます．
  //

  SampleFooTest() {
    // テスト毎に実行される set-up をここに書きます．
  }

  virtual ~SampleFooTest() {
    // テスト毎に実行される，例外を投げない clean-up をここに書きます．
  }

  // コンストラクタとデストラクタでは不十分な場合．
  // 以下のメソッドを定義することができます：

virtual void SetUp() {
  // このコードは，コンストラクタの直後（各テストの直前）
  // に呼び出されます．
}

virtual void TearDown() {
  // このコードは，各テストの直後（デストラクタの直前）
  // に呼び出されます．
}

// ここで宣言されるオブジェクトは，テストケース内の全てのテストで利用できます．
};

// Abc を行う Foo::Bar() メソッドをテストします．
TEST_F(SampleFooTest, MethodBarDoesAbc) {
  EXPECT_EQ(0, 0);
}

// Xyz を行う Foo をテストします．
TEST_F(SampleFooTest, DoesXyz) {
  // Foo の Xyz を検査
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}