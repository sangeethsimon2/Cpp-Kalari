#include </home/cfd/simon/.local/include/gtest/gtest.h>

#include<cmath>
#include<iostream>

TEST(A, B){}

int main(int argc, char** argv){
    ::testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}