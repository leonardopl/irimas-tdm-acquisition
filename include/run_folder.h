#pragma once

#include <cstddef>
#include <string>

// One directory descriptor for cleanup and every output of a run, even if its path is later replaced.
// Failed writes/publications leave incomplete files or .tmp entries for the next run's cleanup: the name may
// already refer to another process's entry. A successful linkat publication removes its .tmp link once;
// failure to remove that link still fails the run, without retrying the deletion.
class RunFolder {
public:
    enum class Failure { none, taken, io };

    explicit RunFolder(const std::string& path);
    ~RunFolder();
    RunFolder(const RunFolder&) = delete;
    RunFolder& operator=(const RunFolder&) = delete;

    int fd() const { return fd_; }
    int open_error() const { return open_error_; }
    const std::string& path() const { return path_; }

    bool create(const std::string& name, const void* bytes, std::size_t size,
                std::string& error, Failure* failure = NULL) const;
    bool publish(const std::string& name, const std::string& bytes,
                 std::string& error, Failure* failure = NULL) const;

#ifdef TDM_RUN_FOLDER_TEST
    void force_linkat_for_test() { force_linkat_ = true; }
#endif

private:
    const std::string path_;
    int fd_, open_error_;
#ifdef TDM_RUN_FOLDER_TEST
    bool force_linkat_ = false;
#endif
    bool fail(const std::string& name, int code, std::string& error, Failure* failure) const;
};
