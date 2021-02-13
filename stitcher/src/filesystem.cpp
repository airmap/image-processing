#include "airmap/filesystem.h"

namespace airmap {
namespace filesystem {

path::path() {}

path::path(const char &p)
    : _path(std::string(1, p))
{
}

path::path(const std::string &p)
    : _path(p)
{
}

path path::filename() const
{
    size_t pos = _path.find_last_of("/");
    if (pos == 0 && _path.size() == 1) {
        return {_path};
    }
    if (pos == _path.size() - 1) {
        return dot();
    }
    if (pos == std::string::npos) {
        return {_path};
    }
    return {_path.substr(pos + 1)};
}

path path::parent_path() const
{
    size_t pos = _path.find_last_of(separator());
    if (pos == std::string::npos) {
        return {};
    }
    if (pos == 0 && _path.size() > 1) {
        return {separator()};
    }
    return {_path.substr(0, pos)};
}

path path::stem() const
{
    path name(filename());
    if (name == dot_path() || name == dot_dot_path()) {
        return name;
    }
    size_t pos(name.string().find_last_of(dot()));
    return pos == std::string::npos
        ? name
        : name.string().substr(0, pos);
}

} // namespace filesystem
} // namespace airmap
