#include "gtest/gtest.h"

#include "airmap/monitor/timer.h"

#include <chrono>
#include <tuple>

using airmap::stitcher::monitor::ElapsedTime;
using airmap::stitcher::monitor::Timer;

TEST(ElapsedTimeTest, ElapsedTimeConstructors)
{
    ElapsedTime empty;
    EXPECT_EQ(empty.get(), ElapsedTime::DurationType{0});

    int elapsedIntValue = 1000;
    ElapsedTime elapsedInt{elapsedIntValue};
    EXPECT_EQ(elapsedInt.get(), ElapsedTime::DurationType{1000});

    int64_t elapsedInt64Value = 1000;
    ElapsedTime elapsedInt64{elapsedInt64Value};
    EXPECT_EQ(elapsedInt64.get(), ElapsedTime::DurationType{1000});

    ElapsedTime elapsedTemporaryInt{1000};
    EXPECT_EQ(elapsedTemporaryInt.get(), ElapsedTime::DurationType{1000});

    ElapsedTime elapsedString{"00:00:00.000"};
    EXPECT_EQ(elapsedString.get(), ElapsedTime::DurationType{0});
    elapsedString = {"01:02:03.004"};
    EXPECT_EQ(elapsedString.get(),
              ElapsedTime::DurationType{(1 * 60 * 60 * 1000) + (2 * 60 * 1000) +
                                        (3 * 1000) + 4});

    EXPECT_THROW(ElapsedTime{""}, std::invalid_argument);
    EXPECT_THROW(ElapsedTime{"0:0:0.0"}, std::invalid_argument);
    EXPECT_THROW(ElapsedTime{"0"}, std::invalid_argument);
    EXPECT_THROW(ElapsedTime{"0:0:0"}, std::invalid_argument);
}

TEST(ElapsedTimeTest, ElapsedTimeOperators)
{
    ElapsedTime a{"01:02:03.004"};
    ElapsedTime b("01:02:03.004");

    EXPECT_EQ(a, b);

    ElapsedTime c = a;
    EXPECT_EQ(a, c);

    c *= 2.;
    EXPECT_EQ(c, ElapsedTime::fromValues(std::make_tuple(2, 4, 6, 8)));

    c /= 2.;
    EXPECT_EQ(c, a);

    ElapsedTime d{a};
    EXPECT_EQ(a, d);

    ElapsedTime e{"04:03:02.001"};
    EXPECT_NE(a, e);

    ElapsedTime f = a + b;
    ElapsedTime g{"02:04:06.008"};
    EXPECT_EQ(f, g);

    ElapsedTime h{f - a};
    EXPECT_EQ(h, b);

    ElapsedTime i{a * 2};
    EXPECT_EQ(i, f);

    ElapsedTime j{i / 2};
    EXPECT_EQ(j, a);

    double k{a / b};
    EXPECT_EQ(k, 1.);

    double l{f / a};
    EXPECT_EQ(l, 2.);

    double m{a / f};
    EXPECT_EQ(m, 0.5);
}

TEST(ElapsedTimeTest, ElapsedTimeFrom)
{
    ElapsedTime a = ElapsedTime::fromHours(1);
    EXPECT_EQ(a, ElapsedTime{"01:00:00.000"});
    EXPECT_EQ(a, ElapsedTime::DurationType{1 * 60 * 60 * 1000});

    ElapsedTime b = ElapsedTime::fromMinutes(1);
    EXPECT_EQ(b, ElapsedTime{"00:01:00.000"});
    EXPECT_EQ(b, ElapsedTime::DurationType{1 * 60 * 1000});

    ElapsedTime c = ElapsedTime::fromSeconds(1);
    EXPECT_EQ(c, ElapsedTime{"00:00:01.000"});
    EXPECT_EQ(c, ElapsedTime::DurationType{1 * 1000});

    ElapsedTime d = ElapsedTime::fromMilliseconds(1);
    EXPECT_EQ(d, ElapsedTime{"00:00:00.001"});
    EXPECT_EQ(d, ElapsedTime::DurationType{1});

    ElapsedTime e = ElapsedTime::fromValues(std::make_tuple(1, 2, 3, 4));
    EXPECT_EQ(e, ElapsedTime("01:02:03.004"));
    EXPECT_NE(e, ElapsedTime("01:02:03.005"));
}

TEST(ElapsedTimeTest, ElapsedTimeValues)
{
    ElapsedTime a{"01:02:03.004"};
    EXPECT_EQ(a.hours(), 1);
    EXPECT_EQ(a.minutes(), 2);
    EXPECT_EQ(a.seconds(), 3);
    EXPECT_EQ(a.milliseconds(), 4);
}

TEST(ElapsedTimeTest, ElapsedTimeValuesNoRemainder)
{
    ElapsedTime a{"01:02:03.004"};
    EXPECT_EQ(a.hours(false), 1);
    EXPECT_EQ(a.minutes(false), (1 * 60) + 2);
    EXPECT_EQ(a.seconds(false), (1 * 60 * 60) + (2 * 60) + 3);
    EXPECT_EQ(a.milliseconds(false),
              (1 * 60 * 60 * 1000) + (2 * 60 * 1000) + (3 * 1000) + 4);
}

TEST(ElapsedTimeTest, ElapsedTimeString)
{
    std::string elapsedTimeString = "01:02:03.004";
    ElapsedTime elapsedTime{elapsedTimeString};
    EXPECT_EQ(elapsedTime.str(), elapsedTimeString);
    EXPECT_EQ(elapsedTime.str(false), elapsedTimeString.substr(0, 8));
}

TEST(TimerTest, TimerTest)
{
    Timer timer;
    timer.start();

    int j = 0;
    for (int i = 0; i < 100000000; i++) {
        j++;
    }

    timer.stop();

    EXPECT_GT(timer.elapsed().milliseconds(false), 0);

    ElapsedTime elapsedTime = timer.elapsed();

    j = 0;
    for (int i = 0; i < 100000000; i++) {
        j++;
    }

    timer.stop();

    EXPECT_GT(timer.elapsed().milliseconds(false),
              elapsedTime.milliseconds(false));
}
