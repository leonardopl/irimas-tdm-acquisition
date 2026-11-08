#include "run_folder.h"
#include "acquisition_protocol.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

namespace {
bool plain_name(const std::string& name)
{
    return !name.empty() && name != "." && name != ".." &&
           name.find('/') == std::string::npos && name.find('\0') == std::string::npos;
}
}

RunFolder::RunFolder(const std::string& path)
    : path_(path), fd_(open(path.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC)),
      open_error_(fd_ < 0 ? errno : 0)
{
}

RunFolder::~RunFolder()
{
    if (fd_ >= 0) close(fd_);
}

bool RunFolder::fail(const std::string& name, int code, std::string& error, Failure* failure) const
{
    const bool taken = code == EEXIST || code == ELOOP || code == EISDIR || code == ENOTEMPTY;
    if (failure) *failure = taken ? Failure::taken : Failure::io;
    const std::string file = acquisition_protocol::diagnosticWord(path_ + "/" + name);
    error = taken ? "refusing to write " + file +
        ": an entry with that name appeared in the run folder during the run; nothing was overwritten"
        : file + ": " + std::strerror(code);
    return false;
}

bool RunFolder::create(const std::string& name, const void* bytes, std::size_t size,
                       std::string& error, Failure* failure) const
{
    error.clear();
    if (failure) *failure = Failure::none;
    if (!plain_name(name)) return fail(name, EINVAL, error, failure);
    const int file = openat(fd_, name.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW | O_CLOEXEC, 0666);
    if (file < 0) return fail(name, errno, error, failure);
    const char* data = static_cast<const char*>(bytes);
    int code = 0;
    while (size > 0) {
        const ssize_t count = write(file, data, size);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) { code = count < 0 ? errno : EIO; break; }
        data += count;
        size -= static_cast<std::size_t>(count);
    }
    // On Linux close releases the fd even on EINTR; retrying could close a reused descriptor.
    if (close(file) != 0 && !code) code = errno;
    if (!code) return true;
    fail(name, code, error, failure);
    error += "; the incomplete file is left for the next run's cleanup";
    return false;
}

bool RunFolder::publish(const std::string& name, const std::string& bytes,
                        std::string& error, Failure* failure) const
{
    error.clear();
    if (failure) *failure = Failure::none;
    if (!plain_name(name)) return fail(name, EINVAL, error, failure);
    const std::string temporary = name + ".tmp";
    if (!create(temporary, bytes.data(), bytes.size(), error, failure)) return false;
    int result;
#ifdef TDM_RUN_FOLDER_TEST
    if (force_linkat_) { result = -1; errno = EOPNOTSUPP; }
    else
#endif
        result = renameat2(fd_, temporary.c_str(), fd_, name.c_str(), RENAME_NOREPLACE);
    if (result == 0) return true;
    int code = errno;
    if (code == EINVAL || code == ENOSYS || code == EOPNOTSUPP) {
        if (linkat(fd_, temporary.c_str(), fd_, name.c_str(), 0) == 0) {
            if (unlinkat(fd_, temporary.c_str(), 0) == 0) return true;
            fail(temporary, errno, error, failure);
            error += "; the output was published, but " + acquisition_protocol::diagnosticWord(temporary) +
                     " is left for the next run's cleanup";
            return false;
        }
        code = errno;
    }
    fail(name, code, error, failure);
    error += "; " + acquisition_protocol::diagnosticWord(temporary) + " is left for the next run's cleanup";
    return false;
}
