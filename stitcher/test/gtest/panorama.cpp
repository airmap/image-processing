#include "gtest/gtest.h"
#include "airmap/panorama.h"

#include <boost/filesystem/path.hpp>

using airmap_path = airmap::filesystem::path;
using boost_path = boost::filesystem::path;

class PathTest : public ::testing::Test
{
protected:
    std::vector<std::string> paths()
    {
        return {
            "/",
            "a",
            "a/",
            "/a",
            "a.jpg",
            "/a.jpg",
            "/a/b",
            "/a/.b",
            "/a/b.txt",
            "a/b.txt",
            "a.b.txt",
            "a.b.c.txt",
            "/a.b.c.txt",
            "a/b/c.txt"
            "a/b/c.d.txt",
            "a.b.c.d/e.f.g.h"
            "a.b.c.d/.e.f.g.h"
        };
    }
};

TEST_F(PathTest, filename)
{
    for (auto &path : paths()) {
        airmap_path p(path);
        boost_path bp(path);
        EXPECT_EQ(p.filename().string(), bp.filename().string());
    }
}

TEST_F(PathTest, parent_path)
{
    for (auto &path : paths()) {
        airmap_path p(path);
        boost_path bp(path);
        EXPECT_EQ(p.parent_path().string(), bp.parent_path().string());
    }
}

TEST_F(PathTest, path)
{
    for (auto &path : paths()) {
        airmap_path p(path);
        boost_path bp(path);
        EXPECT_EQ(p.string(), bp.string());
    }
}

TEST_F(PathTest, stem)
{
    for (auto &path : paths()) {
        airmap_path p(path);
        boost_path bp(path);
        EXPECT_EQ(p.stem().string(), bp.stem().string());
    }
}
