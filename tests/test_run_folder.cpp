#include "run_folder.h"
#include "acquisition_protocol.h"

#include <cassert>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <fstream>
#include <ftw.h>
#include <iostream>
#include <iterator>
#include <signal.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {
int remove_entry(const char* path, const struct stat*, int, struct FTW*) { return std::remove(path); }
struct Scratch {
    std::string path;
    Scratch() {
        char name[] = "/tmp/tdm_run_folder_XXXXXX";
        const char* made = mkdtemp(name);
        assert(made);
        path = made;
    }
    ~Scratch() { nftw(path.c_str(), remove_entry, 32, FTW_DEPTH | FTW_PHYS); }
};

void put(const std::string& path, const std::string& bytes)
{
    std::ofstream out(path.c_str(), std::ios::binary);
    out << bytes;
    out.close();
    assert(out.good());
}
std::string get(const std::string& path)
{
    std::ifstream in(path.c_str(), std::ios::binary);
    assert(in.good());
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}
bool exists(const std::string& path)
{
    struct stat info;
    return lstat(path.c_str(), &info) == 0;
}

void blockers(bool publish, bool fallback)
{
    for (bool temporary : {false, true}) {
        if (temporary && !publish) continue;
        for (int type = 0; type < 4; ++type) {
            Scratch scratch;
            const std::string dir = scratch.path + "/run", outside = scratch.path + "/outside";
            assert(mkdir(dir.c_str(), 0700) == 0);
            RunFolder folder(dir);
            if (fallback) folder.force_linkat_for_test();
            const std::string name = "output.csv", occupied = name + (temporary ? ".tmp" : "");
            const std::string path = dir + "/" + occupied;
            put(outside, "outside sentinel");
            if (type == 0) assert(symlink(outside.c_str(), path.c_str()) == 0);
            if (type == 1) assert(symlink((scratch.path + "/missing").c_str(), path.c_str()) == 0);
            if (type == 2) put(path, "existing sentinel");
            if (type == 3) assert(mkdir(path.c_str(), 0700) == 0);
            std::string error;
            RunFolder::Failure kind = RunFolder::Failure::none;
            const bool ok = publish ? folder.publish(name, "new data", error, &kind)
                                    : folder.create(name, "new data", 8, error, &kind);
            assert(!ok && kind == RunFolder::Failure::taken);
            assert(error.find(acquisition_protocol::diagnosticWord(path)) != std::string::npos);
            assert(error.find("nothing was overwritten") != std::string::npos);
            assert(get(outside) == "outside sentinel" && !exists(scratch.path + "/missing"));
            struct stat info;
            assert(lstat(path.c_str(), &info) == 0);
            if (type < 2) {
                assert(S_ISLNK(info.st_mode));
                char target[1024];
                const ssize_t size = readlink(path.c_str(), target, sizeof target);
                assert(size > 0 && std::string(target, size) ==
                       (type == 0 ? outside : scratch.path + "/missing"));
            }
            if (type == 2) assert(S_ISREG(info.st_mode) && get(path) == "existing sentinel");
            if (type == 3) assert(S_ISDIR(info.st_mode));
            if (publish) {
                // Leave our complete temporary after a publication failure; never remove an occupied entry.
                assert(exists(dir + "/" + name + ".tmp"));
                assert(exists(dir + "/" + name) == !temporary);
                if (!temporary) {
                    assert(get(dir + "/" + name + ".tmp") == "new data");
                    assert(error.find("\"output.csv.tmp\" is left for the next run's cleanup") != std::string::npos);
                }
            }
        }
    }
}

void happy_paths(bool fallback)
{
    Scratch scratch;
    RunFolder folder(scratch.path);
    assert(folder.fd() >= 0 && folder.open_error() == 0);
    assert(fcntl(folder.fd(), F_GETFD) & FD_CLOEXEC);
    if (fallback) folder.force_linkat_for_test();
    const mode_t old_mask = umask(0027);
    const std::string bytes("binary\0data\n", 12);
    std::string error = "old error";
    RunFolder::Failure kind = RunFolder::Failure::taken;
    assert(folder.create("frame.pgm", bytes.data(), bytes.size(), error, &kind));
    assert(error.empty() && kind == RunFolder::Failure::none);
    assert(folder.publish("table.csv", bytes, error, &kind));
    assert(error.empty() && kind == RunFolder::Failure::none);
    assert(folder.create("empty", NULL, 0, error));
    umask(old_mask);
    for (const std::string& name : {"frame.pgm", "table.csv", "empty"}) {
        struct stat info;
        assert(stat((scratch.path + "/" + name).c_str(), &info) == 0);
        assert((info.st_mode & 0777) == (0666 & ~0027));
        assert(get(scratch.path + "/" + name) == (name == "empty" ? "" : bytes));
    }
    assert(!exists(scratch.path + "/table.csv.tmp"));
    for (const std::string& name : std::vector<std::string>{"", ".", "..", "../escape", "a/b", std::string("a\0b", 3)}) {
        assert(!folder.create(name, NULL, 0, error, &kind) && kind == RunFolder::Failure::io);
        assert(!folder.publish(name, "", error, &kind) && kind == RunFolder::Failure::io);
        assert(error.find(acquisition_protocol::diagnosticWord(scratch.path + "/" + name)) != std::string::npos);
    }
    // Other I/O failures carry the file name but are not a taken name.
    RunFolder missing(scratch.path + "/missing");
    assert(missing.fd() < 0 && missing.open_error() == ENOENT);
    assert(!missing.create("frame.pgm", NULL, 0, error, &kind) && kind == RunFolder::Failure::io);
    assert(error.find("missing/frame.pgm") != std::string::npos);
    assert(!missing.publish("table.csv", "text", error, &kind) && kind == RunFolder::Failure::io);
    if (geteuid() != 0) {
        assert(chmod(scratch.path.c_str(), 0500) == 0);
        assert(!folder.create("denied", NULL, 0, error, &kind) && kind == RunFolder::Failure::io);
        assert(!folder.publish("denied", "text", error, &kind) && kind == RunFolder::Failure::io);
        assert(!exists(scratch.path + "/denied.tmp"));
        assert(chmod(scratch.path.c_str(), 0700) == 0);
    }
}

void replaced_folder(bool fallback)
{
    Scratch scratch;
    const std::string path = scratch.path + "/run", moved = scratch.path + "/moved";
    assert(mkdir(path.c_str(), 0700) == 0);
    RunFolder folder(path);
    if (fallback) folder.force_linkat_for_test();
    assert(rename(path.c_str(), moved.c_str()) == 0);
    assert(mkdir(path.c_str(), 0700) == 0);
    put(path + "/frame.pgm", "replacement sentinel");
    put(path + "/table.csv", "replacement sentinel");
    std::string error;
    assert(folder.create("frame.pgm", "frame", 5, error));
    assert(folder.publish("table.csv", "table", error));
    assert(get(moved + "/frame.pgm") == "frame" && get(moved + "/table.csv") == "table");
    assert(get(path + "/frame.pgm") == "replacement sentinel" && get(path + "/table.csv") == "replacement sentinel");
    assert(!exists(moved + "/table.csv.tmp") && !exists(path + "/table.csv.tmp"));
}

void failed_writes()
{
    Scratch scratch;
    RunFolder folder(scratch.path);
    struct rlimit original;
    assert(getrlimit(RLIMIT_FSIZE, &original) == 0);
    struct rlimit limited = original;
    limited.rlim_cur = 3; // A real short write, then EFBIG, without depending on free disk space.
    const auto previous_signal = signal(SIGXFSZ, SIG_IGN);
    assert(previous_signal != SIG_ERR && setrlimit(RLIMIT_FSIZE, &limited) == 0);
    std::string error;
    RunFolder::Failure kind;
    assert(!folder.create("frame.pgm", "too large", 9, error, &kind) && kind == RunFolder::Failure::io);
    assert(error.find("/frame.pgm") != std::string::npos && get(scratch.path + "/frame.pgm") == "too");
    assert(error.find("the incomplete file is left for the next run's cleanup") != std::string::npos);
    assert(!folder.publish("table.csv", "too large", error, &kind) && kind == RunFolder::Failure::io);
    assert(error.find("/table.csv.tmp") != std::string::npos);
    assert(error.find("the incomplete file is left for the next run's cleanup") != std::string::npos);
    assert(!exists(scratch.path + "/table.csv") && get(scratch.path + "/table.csv.tmp") == "too");
    assert(setrlimit(RLIMIT_FSIZE, &original) == 0 && signal(SIGXFSZ, previous_signal) != SIG_ERR);
    // Both names occupied: a failed create must not remove somebody else's temporary.
    put(scratch.path + "/table.csv", "final sentinel");
    put(scratch.path + "/table.csv.tmp", "temporary sentinel");
    assert(!folder.publish("table.csv", "new", error, &kind) && kind == RunFolder::Failure::taken);
    assert(get(scratch.path + "/table.csv") == "final sentinel");
    assert(get(scratch.path + "/table.csv.tmp") == "temporary sentinel");
}
}

int main()
{
    blockers(false, false);
    for (bool fallback : {false, true}) {
        blockers(true, fallback);
        happy_paths(fallback);
        replaced_folder(fallback);
    }
    failed_writes();
    std::cout << "RunFolder: 21 taken-name refusals, short-write failures, modes, fallback and directory replacement passed.\n";
}
