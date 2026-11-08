// Exercise the real entry point in child processes, injecting only a simulated LabJack.
// Aravis is isolated to Fake before every entry; production has no simulation launch switch.
#define TDM_ACQUISITION_TEST 1
#define main acquisition_main
#include "../main.cpp"
#undef main
#include <dirent.h>
#include <ftw.h>
#include <sys/wait.h>

static void require(bool condition, const string& message)
{
    if (!condition) throw runtime_error(message);
}

static string contents(const string& path)
{
    ifstream file(path.c_str(), ios::binary);
    return string(istreambuf_iterator<char>(file), istreambuf_iterator<char>());
}

static void put(const string& path, const string& text)
{
    ofstream file(path.c_str());
    file << text;
    require(file.good(), "cannot write fixture " + path);
}

static bool exists(const string& path) { return access(path.c_str(), F_OK) == 0; }
static int remove_entry(const char* path, const struct stat*, int, struct FTW*) { return remove(path); }

struct Scratch {
    string path, config;
    Scratch() {
        char name[] = "/tmp/tdm_session_XXXXXX";
        char* made = mkdtemp(name);
        require(made != NULL, "mkdtemp failed");
        path = made;
        config = directory("config");
        directory(".config"); directory("cache"); directory("data"); directory("default");
        put(path + "/.config/gui_tomo.conf", "CHEMIN_CONFIG " + config + "\nCHEMIN_ACQUIS " + path + "/default\n");
        writeConfig(false);
        put(config + "/processing.conf", "AMPLI_REF 0\n");
    }
    string directory(const string& name) {
        const string result = path + "/" + name;
        require(mkdir(result.c_str(), 0700) == 0, "mkdir failed: " + result);
        return result;
    }
    void writeConfig(bool no_roi) {
        put(config + "/acquisition.conf", string("CONFIG_SCHEMA 3\nDIM_ROI ") + (no_roi ? "-1" : "32") +
            "\nOFFSET_ROI_X " + (no_roi ? "-1" : "0") + "\nOFFSET_ROI_Y " + (no_roi ? "-1" : "0") +
            "\nPIXEL_FORMAT Mono8\nNXMAX 8\nNB_HOLO 3\nSCAN_PATTERN FERMAT\nSCAN_SEED 42\n"
            "NB_CIRCLES_ANNULAR 2\nNA_COND_LIM 1\nVXMIN -1\nVXMAX 1\nVYMIN -1\nVYMAX 1\n"
            "AMPLI_REF 0\nFSM_SETTLE_MS 0\nFSM_OVERLAP 0\nFSM_OVERLAP_MARGIN_US 500\n");
    }
    ~Scratch() { if (!path.empty()) nftw(path.c_str(), remove_entry, 32, FTW_DEPTH | FTW_PHYS); }
};

static void fakeOnly()
{
    for (unsigned int i = 0; i < arv_get_n_interfaces(); ++i) {
        const string id = arv_get_interface_id(i);
        if (id != "Fake") arv_disable_interface(id.c_str());
    }
    arv_disable_interface("USB3Vision");
    arv_disable_interface("GigEVision");
    arv_enable_interface("Fake");
}

struct Child {
    pid_t pid = -1;
    int input = -1, output = -1;
    string audit, log;
    acquisition_protocol::LineReader framing;
    deque<string> lines;
    bool eof = false;

    Child(const Scratch& scratch, const string& name, const vector<string>& args, bool driver_exit = false,
          const string& fail_image_path = "", const function<void()>& setup = {}) {
        audit = scratch.path + "/" + name + ".audit";
        log = scratch.path + "/" + name + ".log";
        int in[2], out[2];
        require(pipe(in) == 0 && pipe(out) == 0, "pipe failed");
        cout.flush(); cerr.flush(); fflush(NULL);
        pid = fork();
        require(pid >= 0, "fork failed");
        if (pid == 0) {
            close(in[1]); close(out[0]);
            dup2(in[0], STDIN_FILENO); dup2(out[1], STDOUT_FILENO);
            close(in[0]); close(out[1]);
            const int diagnostic = open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
            dup2(diagnostic, STDERR_FILENO); close(diagnostic);
            setenv("HOME", scratch.path.c_str(), 1);
            setenv("XDG_CONFIG_HOME", (scratch.path + "/.config").c_str(), 1);
            setenv("XDG_CACHE_HOME", (scratch.path + "/cache").c_str(), 1);
            setenv("XDG_DATA_HOME", (scratch.path + "/data").c_str(), 1);
            fakeOnly();
            int result;
            {
                Ljack simulated(true);
                simulated.set_simulate(0);
                test_labjack = &simulated;
                if (driver_exit) test_after_dac_open = []() { std::exit(1); };
                bool image_failure_injected = false;
                test_hardware_event = [&](const string& event, double x, double y) {
                    ofstream file(audit.c_str(), ios::app);
                    file << event << " " << setprecision(17) << x << " " << y << "\n";
                    // Inject after clearing, at the centre move, so clearing cannot remove it.
                    if (!fail_image_path.empty() && !image_failure_injected && event == "dac") {
                        require(mkdir(fail_image_path.c_str(), 0700) == 0, "cannot inject image-write failure");
                        image_failure_injected = true;
                    }
                };
                if (setup) setup();
                vector<string> words = {"tdm_acquisition"};
                words.insert(words.end(), args.begin(), args.end());
                vector<char*> argv;
                for (string& word : words) argv.push_back(&word[0]);
                argv.push_back(NULL);
                result = acquisition_main(static_cast<int>(words.size()), argv.data());
                test_labjack = NULL;
                test_hardware_event = {};
            }
            cout.flush(); cerr.flush(); fflush(NULL);
            _exit(result);
        }
        close(in[0]); close(out[1]);
        input = in[1]; output = out[0];
    }
    void sendRaw(const string& text) {
        size_t done = 0;
        while (done < text.size()) {
            const ssize_t n = write(input, text.data() + done, text.size() - done);
            if (n < 0 && errno == EINTR) continue;
            require(n > 0, "cannot write to child; " + contents(log));
            done += n;
        }
    }
    void send(const vector<string>& words) { sendRaw(acquisition_protocol::joinWords(words) + "\n"); }
    void closeInput() { if (input >= 0) { close(input); input = -1; } }
    bool readMore(int timeout = 10000) {
        pollfd fd = {output, POLLIN, 0};
        int ready;
        do { ready = poll(&fd, 1, timeout); } while (ready < 0 && errno == EINTR);
        require(ready > 0, "child reply timeout; " + contents(log));
        char bytes[4096];
        const ssize_t n = read(output, bytes, sizeof(bytes));
        require(n >= 0, "read child failed");
        if (!n) {
            string incomplete;
            require(framing.finish(incomplete), "child produced incomplete reply: " + incomplete);
            eof = true;
            return false;
        }
        const auto complete = framing.append(bytes, static_cast<size_t>(n));
        lines.insert(lines.end(), complete.begin(), complete.end());
        return true;
    }
    acquisition_protocol::Reply reply(const string& keyword) {
        while (lines.empty() && readMore()) {}
        require(!lines.empty(), "unexpected child EOF; " + contents(log));
        const string line = lines.front(); lines.pop_front();
        acquisition_protocol::Reply result;
        string error;
        require(acquisition_protocol::parseReply(line, result, error), "non-protocol stdout: " + line + "; " + error);
        require(result.keyword == keyword, "expected " + keyword + ", received " + line + "; " + contents(log));
        return result;
    }
    void expectEof() {
        require(lines.empty(), "unexpected replies after stop");
        while (!eof) { readMore(); require(lines.empty(), "unexpected reply before EOF"); }
    }
    int wait() {
        const auto deadline = chrono::steady_clock::now() + chrono::seconds(10);
        int status = 0;
        while (waitpid(pid, &status, WNOHANG) == 0) {
            require(chrono::steady_clock::now() < deadline, "child failed to exit; " + contents(log));
            this_thread::sleep_for(chrono::milliseconds(10));
        }
        pid = -1;
        require(WIFEXITED(status), "child terminated without normal cleanup; " + contents(log));
        return WEXITSTATUS(status);
    }
    void cleaned() const {
        const string events = contents(audit);
        const string cleanup = "dac 0 0\nfree_run 1 0\n";
        require(events.size() >= cleanup.size() && events.substr(events.size() - cleanup.size()) == cleanup,
                "DAC not parked before camera teardown: " + events);
        require(events.find("free_run 0") == string::npos, "free-run restore failed");
    }
    ~Child() {
        closeInput();
        if (output >= 0) close(output);
        if (pid > 0) { kill(pid, SIGKILL); waitpid(pid, NULL, 0); }
    }
};

static vector<string> launch(const string& identity, const string& format = "Mono8")
{
    return {"--SESSION", "1", "--CAMERA_ID", identity, "--DIM_ROI", "32", "--OFFSET_ROI_X", "0",
            "--OFFSET_ROI_Y", "0", "--PIXEL_FORMAT", format};
}

static vector<string> run(const string& folder, int count = 3, int settle = 0)
{
    return {"RUN", "--CHEMIN_ACQUIS", folder, "--NXMAX", "8", "--NB_HOLO", to_string(count),
            "--FSM_SETTLE_MS", to_string(settle)};
}

static set<string> files(const string& folder)
{
    DIR* directory = opendir(folder.c_str());
    require(directory != NULL, "cannot list output");
    set<string> names;
    while (dirent* entry = readdir(directory)) {
        const string name = entry->d_name;
        if (name != "." && name != "..") names.insert(name);
    }
    closedir(directory);
    return names;
}

static string stdoutText(Child& child)
{
    string result;
    while (!child.eof) {
        child.readMore();
        while (!child.lines.empty()) { result += child.lines.front() + '\n'; child.lines.pop_front(); }
    }
    return result;
}

static void argumentSafety(Scratch& scratch)
{
    const string explicit_folder = scratch.directory("argument-safety");
    const string default_sentinel = scratch.path + "/default/i777.pgm";
    const string explicit_sentinel = explicit_folder + "/i777.pgm";
    put(default_sentinel, "default untouched"); put(explicit_sentinel, "explicit untouched");
    const vector<vector<string>> helps = {{"--help"}, {"-h"}, {"--NB_HOLO", "3", "--help"},
        {"--CHEMIN_ACQUIS", explicit_folder, "--NB_HOLO", "3", "--help"},
        {"--CHEMIN_ACQUIS", "-h"}, {"bare", "--SESSION", "0", "--help"}, {"--SESSION", "1", "--help"}};
    int index = 0;
    auto untouched = [&](const Child& child) {
        require(contents(child.audit).empty(), "argument handling wrote the DAC");
        require(contents(default_sentinel) == "default untouched" && contents(explicit_sentinel) == "explicit untouched",
                "argument handling cleared an output folder");
        require(files(scratch.path + "/default") == set<string>{"i777.pgm"} &&
                files(explicit_folder) == set<string>{"i777.pgm"}, "argument handling wrote output files");
    };
    for (const auto& words : helps) {
        Child child(scratch, "help-" + to_string(index++), words);
        const string output = stdoutText(child);
        require(child.wait() == 0 && output.find("Usage") != string::npos, "help did not exit 0 with usage");
        for (const auto& key : acquisition_protocol::argumentKeys())
            require(output.find(std::string("--") + key.name) != string::npos, "help omitted a table key");
        require(output.find("Reading paths") == string::npos && contents(child.log).empty(), "help read configuration");
        untouched(child);
    }
    struct BadLaunch { vector<string> words; string bad; bool session; };
    const vector<BadLaunch> bad_launches = {
        {{"--NB_HOLO"}, "--NB_HOLO", false},
        {{"bare"}, "bare", false}, {{"-x", "1"}, "-x", false},
        {{"--CHEMIN_ACQUIS", "--NB_HOLO", "3"}, "--CHEMIN_ACQUIS", false},
        {{"--FOO", "1"}, "--FOO", false}, {{"--NB_HOLO=3"}, "--NB_HOLO=3", false},
        {{"--NB_HOLO", "3", "--NB_HOLO", "4"}, "--NB_HOLO", false},
        {{"--SESSION", "0"}, "0", false},
        {{"--SESSION", "1", "--DIM_ROI", "32", "--NB_HOLO", "3"}, "--NB_HOLO", true},
        {{"--SESSION", "1", "--CAMERA_ID"}, "--CAMERA_ID", true},
        {{"--CHEMIN_ACQUIS", explicit_folder, "--NB_HOLO"}, "--NB_HOLO", false}
    };
    for (const auto& test : bad_launches) {
        Child child(scratch, "bad-argument-" + to_string(index++), test.words);
        const string output = stdoutText(child);
        require(child.wait() == 2, "invalid launch did not exit 2 before startup");
        const string error = contents(child.log);
        const string valid = "valid keys: " + acquisition_protocol::validKeys(acquisition_protocol::ArgumentOrigin::Launch, test.session);
        require(error.find("tdm_acquisition: ") == 0 && error.find(test.bad) != string::npos &&
                error.find(valid) != string::npos &&
                error.find("(--help lists the keys)") != string::npos, "usage error omitted bad word, valid keys or hint");
        if (test.session) {
            // The GUI's session client reads only stdout: exactly one ERROR reply, the same error as stderr.
            acquisition_protocol::Reply reply;
            string reply_error;
            require(count(output.begin(), output.end(), '\n') == 1 &&
                    acquisition_protocol::parseReply(output.substr(0, output.size() - 1), reply, reply_error) &&
                    reply.keyword == "ERROR" && reply.values.size() == 1 && reply.values.count("message"),
                    "session usage error did not reply one ERROR on stdout: " + output);
            const string& message = reply.values.at("message");
            require(message.find(test.bad) != string::npos && message.find(valid) != string::npos &&
                    error.find("tdm_acquisition: " + message + " (--help lists the keys)") == 0,
                    "session usage ERROR omitted the bad word or valid keys: " + message);
        } else
            require(output.empty(), "one-shot usage error wrote to stdout: " + output);
        untouched(child);
    }
    // B278: a one-shot launch names its output folder; gui_tomo.conf's default (live: /ramdisk) is never cleared.
    const vector<vector<string>> no_folder = {{}, {"--NB_HOLO", "3"}, {"--CHEMIN_ACQUIS", ""}};
    for (const auto& words : no_folder) {
        Child child(scratch, "no-folder-" + to_string(index++), words);
        const string output = stdoutText(child);
        require(child.wait() == 2, "one-shot launch without --CHEMIN_ACQUIS did not exit 2");
        const string error = contents(child.log);
        require(error.find("tdm_acquisition: ") == 0 && error.find("--CHEMIN_ACQUIS") != string::npos &&
                error.find("--help") != string::npos && count(error.begin(), error.end(), '\n') == 1,
                "missing-folder error did not name --CHEMIN_ACQUIS and --help on one line: " + error);
        require(output.empty(), "missing-folder usage error wrote to stdout: " + output);
        untouched(child);
    }
}

// Every entry of the folder must be an expected output (a regular file) or one of the entries the test seeded there.
static vector<FrameTiming> outputs(const string& folder, int count, int dimension = 32, bool reference = false,
                                   int cv_type = CV_8UC1, const set<string>& seeded = {})
{
    set<string> expected = {"fsm_frame_commands.csv", "frame_timing.csv", "frame_timing_summary.json"};
    for (int k = 0; k < count; ++k) {
        const string name = cv::format("i%03d.pgm", k);
        expected.insert(name);
        const cv::Mat image = imread(folder + "/" + name, IMREAD_UNCHANGED);
        require(!image.empty() && image.rows == dimension && image.cols == dimension && image.type() == cv_type, "wrong frame geometry: " + name);
    }
    if (reference) expected.insert("Intensite_ref.pgm");
    for (const string& name : expected) {
        struct stat info;
        require(lstat((folder + "/" + name).c_str(), &info) == 0 && S_ISREG(info.st_mode), "missing output " + name + " in " + folder);
    }
    for (const string& name : files(folder))
        require(expected.count(name) || seeded.count(name), "unexpected entry " + name + " in " + folder);
    vector<FrameTiming> rows;
    string error;
    int settle;
    require(read_frame_timing_csv(folder + "/frame_timing.csv", rows, settle, error), error);
    require(rows.size() == static_cast<size_t>(count), "incorrect per-run timing row count");
    require(rows.front().image_index == 0 && rows.front().dac_start_ms < 50, "clock/index did not restart");
    for (int k = 0; k < count; ++k)
        require(rows[k].image_index == static_cast<size_t>(k) && rows[k].written_ms >= 0, "missing saved frame timing");
    require(contents(folder + "/frame_timing_summary.json").find("\"frames\": " + to_string(count)) != string::npos,
            "wrong frame summary count");
    return rows;
}

static void done(Child& child, int status, int count)
{
    const auto reply = child.reply("DONE");
    require(stoi(reply.values.at("status")) == status && stoi(reply.values.at("frames")) == count,
            "incorrect DONE result; " + contents(child.log));
    require((status == 0) == reply.values.at("error").empty(), "incorrect DONE error");
}

static void protocolRuns(Scratch& scratch, const string& identity, double expected_exposure_us)
{
    const string first = scratch.directory("first run µ"), second = scratch.directory("second"), third = scratch.directory("third");
    put(first + "/sentinel", "keep until RUN");
    Child child(scratch, "protocol", launch(identity));
    const auto hello = child.reply("HELLO");
    require(hello.values.at("protocol") == "1" && hello.values.at("camera") == identity &&
            hello.values.at("roi") == "32,0,0" && hello.values.at("format") == "Mono8", "incorrect HELLO");
    const double exposure_us = stod(hello.values.at("exposure_us"));
    require(std::isfinite(exposure_us) && exposure_us > 0 && exposure_us == expected_exposure_us,
            "HELLO exposure did not match the Fake camera");
    require(exists(first + "/sentinel") && contents(child.audit).empty(), "startup cleared files or moved DAC");
    child.send({"EXPOSURE", "1234.6"});
    auto exposure = child.reply("EXPOSURE");
    require(stod(exposure.values.at("us")) == 1234 && exposure.values.at("auto") == "0" &&
            exposure.values.at("error").empty(), "exposure did not use camera read-back");
    child.send({"EXPOSURE", "nan"});
    exposure = child.reply("EXPOSURE");
    require(!exposure.values.at("error").empty() && stod(exposure.values.at("us")) == 1234, "failed exposure lacked reply");
    const string command = acquisition_protocol::joinWords(run(first));
    child.sendRaw(command.substr(0, command.size()/2));
    pollfd fd = {child.output, POLLIN, 0};
    require(poll(&fd, 1, 40) == 0 && exists(first + "/sentinel"), "partial command was executed");
    child.sendRaw(command.substr(command.size()/2) + "\r\n");
    done(child, 0, 3);
    const auto first_rows = outputs(first, 3, 32, false, CV_8UC1, {"sentinel"});
    require(contents(first + "/sentinel") == "keep until RUN", "successful RUN removed the user's sentinel");
    // Every geometry/format mismatch is refused before clearing or any DAC write (including parks).
    for (const auto& mismatch : vector<pair<string,string>>{{"DIM_ROI", "16"}, {"DIM_ROI", "-1"}, {"OFFSET_ROI_X", "1"},
            {"OFFSET_ROI_Y", "1"}, {"PIXEL_FORMAT", "Mono16"}}) {
        put(second + "/sentinel", "untouched");
        const string before = contents(child.audit);
        auto command = run(second); command.push_back("--" + mismatch.first); command.push_back(mismatch.second);
        child.send(command); done(child, 1, 0);
        require(contents(second + "/sentinel") == "untouched" && files(second).size() == 1 && contents(child.audit) == before,
                "refused RUN cleared files or moved hardware");
    }
    child.sendRaw("\"unterminated\nUNKNOWN\nRUN --NB_HOLO\nQUIT extra\n");
    for (int i = 0; i < 4; ++i) child.reply("ERROR");
    for (const auto& bad : vector<pair<vector<string>, string>>{
            {{"RUN", "--FOO", "1"}, "--FOO"},
            {{"RUN", "--NB_HOLO", "3", "--NB_HOLO", "4"}, "--NB_HOLO"},
            {{"RUN", "--SESSION", "1"}, "--SESSION"}}) {
        const string before = contents(child.audit);
        child.send(bad.first);
        const string message = child.reply("ERROR").values.at("message");
        require(message.find(bad.second) != string::npos &&
                message.find("valid keys: " + acquisition_protocol::validKeys(acquisition_protocol::ArgumentOrigin::Run)) != string::npos,
                "RUN argument error omitted bad word or valid keys");
        require(contents(child.audit) == before && files(second) == set<string>{"sentinel"},
                "malformed RUN touched files or hardware");
    }
    // Queue exposure, another RUN and QUIT during a RUN. Replies must retain command order.
    child.send(run(second, 4, 20));
    child.send({"EXPOSURE", "2000.75"});
    auto reference = run(third, 2); reference.insert(reference.end(), {"--AMPLI_REF", "1"});
    child.send(reference);
    child.send({"QUIT"});
    done(child, 0, 4);
    exposure = child.reply("EXPOSURE");
    require(stod(exposure.values.at("us")) == 2000 && exposure.values.at("error").empty(), "queued exposure failed");
    done(child, 0, 2);
    child.reply("BYE"); child.expectEof();
    require(child.wait() == 0, "QUIT exit was nonzero"); child.cleaned();
    const auto second_rows = outputs(second, 4, 32, false, CV_8UC1, {"sentinel"});
    require(contents(second + "/sentinel") == "untouched", "queued RUN removed the user's sentinel");
    outputs(third, 2, 32, true);
    require(first_rows.back().frame_id != second_rows.front().frame_id, "buffer crossed RUN boundaries");
}

static void idleEof(Scratch& scratch, const string& identity, bool incomplete)
{
    const string name = incomplete ? "incomplete" : "idle";
    Child child(scratch, name, launch(identity));
    child.reply("HELLO");
    if (incomplete) child.sendRaw("RUN --CHEMIN_ACQUIS \"unfinished");
    child.closeInput(); child.expectEof();
    require(child.wait() == 0, "idle EOF exit was nonzero"); child.cleaned();
    require(contents(child.audit) == "dac 0 0\nfree_run 1 0\n", "idle EOF did not park before camera teardown");
}

static void deferredRunConfig(Scratch& scratch, const string& identity)
{
    const string valid_config = contents(scratch.config + "/acquisition.conf");
    const vector<pair<string, string>> invalid_settings = {
        {"NXMAX 8\n", ""}, // Missing optics make derivation impossible without the RUN's --NXMAX.
        {"SCAN_PATTERN FERMAT\n", "SCAN_PATTERN invalid\n"},
        {"FSM_SETTLE_MS 0\n", "FSM_SETTLE_MS -2\n"},
        {"FSM_OVERLAP 0\n", "FSM_OVERLAP 2\n"},
        {"FSM_OVERLAP_MARGIN_US 500\n", "FSM_OVERLAP_MARGIN_US -1\n"}
    };
    for (size_t i = 0; i < invalid_settings.size(); ++i) {
        string config = valid_config;
        const auto& setting = invalid_settings[i];
        const size_t position = config.find(setting.first);
        require(position != string::npos, "missing fixture setting " + setting.first);
        config.replace(position, setting.first.size(), setting.second);
        put(scratch.config + "/acquisition.conf", config);
        const string name = "deferred-config-" + to_string(i), folder = scratch.directory(name);
        put(folder + "/sentinel", "untouched");
        // Exercise both launch overrides and settings taken from the live config.
        const vector<string> args = i % 2 == 0 ? launch(identity)
            : vector<string>{"--SESSION", "1", "--CAMERA_ID", identity};
        Child child(scratch, name, args);
        const auto hello = child.reply("HELLO");
        require(hello.values.at("roi") == "32,0,0" && hello.values.at("format") == "Mono8",
                "invalid live run setting blocked connection setup");
        require(contents(child.audit).empty() && exists(folder + "/sentinel"),
                "startup cleared files or moved the DAC");
        // Invalid live values must still be refused by RUN before any files or hardware change.
        child.send({"RUN", "--CHEMIN_ACQUIS", folder}); done(child, 1, 0);
        require(contents(folder + "/sentinel") == "untouched" && files(folder).size() == 1 && contents(child.audit).empty(),
                "invalid RUN cleared files or moved hardware");
        auto command = run(folder);
        command.insert(command.end(), {"--SCAN_PATTERN", "FERMAT", "--FSM_OVERLAP", "0", "--FSM_OVERLAP_MARGIN_US", "500"});
        child.send(command); done(child, 0, 3);
        child.send({"QUIT"}); child.reply("BYE"); child.expectEof();
        require(child.wait() == 0, "overridden RUN poisoned the session"); child.cleaned();
        outputs(folder, 3, 32, false, CV_8UC1, {"sentinel"});
        require(contents(folder + "/sentinel") == "untouched", "overridden RUN removed the user's sentinel");
    }
    scratch.writeConfig(false);
}

static void randomPolarStops(Scratch& scratch, const string& identity)
{
    vector<ScanFrameCommand> commands;
    string error;
    int checks = 0;
    require(!build_scan_frame_commands("RANDOM_POLAR", 64, Var2D{8, 8}, 3,
                -1, 1, -1, 1, 1, 42, commands, error, [&]() { return ++checks == 8; }) &&
            checks == 8 && commands.empty() && error.find("stopped") != string::npos,
            "RANDOM_POLAR retry loop ignored the stop predicate");

    const string folder = scratch.directory("random-polar-limit");
    put(folder + "/sentinel", "untouched");
    Child child(scratch, "random-polar-limit", launch(identity));
    child.reply("HELLO");
    auto command = run(folder, 64);
    command[4] = "4"; // Replace run()'s NXMAX rather than providing a duplicate key.
    command.insert(command.end(), {"--SCAN_PATTERN", "RANDOM_POLAR"});
    const string before = contents(child.audit);
    child.send(command);
    const auto reply = child.reply("DONE");
    require(reply.values.at("status") == "1" && reply.values.at("frames") == "0" &&
            reply.values.at("error").find("RANDOM_POLAR exhausted its sampling attempt limit") != string::npos,
            "unreachable RANDOM_POLAR count did not fail with a clear error");
    require(contents(folder + "/sentinel") == "untouched" && files(folder).size() == 1 &&
            contents(child.audit) == before, "RANDOM_POLAR failure cleared files or moved hardware");
    child.send({"QUIT"}); child.reply("BYE"); child.expectEof();
    require(child.wait() == 0, "RANDOM_POLAR failure poisoned the session"); child.cleaned();
}

static void cliPromptStop(Scratch& scratch)
{
    const string folder = scratch.directory("cli-prompt-stop");
    auto args = run(folder); args.erase(args.begin());
    Child child(scratch, "cli-prompt-stop", args, false, "", []() {
        // Fake exposes one camera; exercise the multiple-camera prompt while the
        // real CLI stack owns its open DAC, so its signal cleanup is also checked.
        test_after_dac_open = []() { PromptCameraSelection(2); };
    });
    const auto deadline = chrono::steady_clock::now() + chrono::seconds(5);
    string diagnostic;
    while (diagnostic.find("Select camera [0-1]: ") == string::npos) {
        require(chrono::steady_clock::now() < deadline, "CLI did not reach camera prompt");
        // The prompt has no newline; read raw CLI diagnostics until it is visible.
        pollfd fd = {child.output, POLLIN, 0};
        const int ready = poll(&fd, 1, 50);
        if (ready < 0 && errno == EINTR) continue;
        require(ready >= 0, "cannot wait for camera prompt");
        if (!ready) continue;
        char bytes[4096];
        const ssize_t count = read(child.output, bytes, sizeof(bytes));
        require(count > 0, "CLI exited before camera prompt; " + contents(child.log));
        diagnostic.append(bytes, static_cast<size_t>(count));
    }
    child.sendRaw(" 1"); // Partial formatted input must not bypass the stop checks.
    require(kill(child.pid, SIGTERM) == 0, "cannot stop camera prompt");
    while (!child.eof) { child.readMore(); child.lines.clear(); }
    require(child.wait() == 1, "camera prompt stop did not return status 1");
    require(contents(child.audit) == "dac 0 0\n", "camera prompt stop did not park the open DAC");
}

static void eofAfterRun(Scratch& scratch, const string& identity)
{
    const string folder = scratch.directory("eof-after-run"), ready = scratch.path + "/before-done";
    Child child(scratch, "eof-after-run", launch(identity), false, "", [&]() {
        test_before_done = [&]() {
            put(ready, "ready");
            const auto deadline = chrono::steady_clock::now() + chrono::seconds(5);
            while (!StopRequested()) {
                require(chrono::steady_clock::now() < deadline, "EOF did not arrive before DONE");
                this_thread::sleep_for(chrono::milliseconds(1));
            }
        };
    });
    child.reply("HELLO"); child.send(run(folder));
    const auto deadline = chrono::steady_clock::now() + chrono::seconds(5);
    while (!exists(ready)) {
        require(chrono::steady_clock::now() < deadline, "RUN did not reach DONE");
        this_thread::sleep_for(chrono::milliseconds(1));
    }
    child.closeInput();
    done(child, 0, 3); child.expectEof();
    require(child.wait() == 0, "EOF after RUN changed the idle exit status");
    child.cleaned(); outputs(folder, 3);
}

static void waitForFrame(const string& folder)
{
    const auto deadline = chrono::steady_clock::now() + chrono::seconds(5);
    while (!exists(folder + "/i000.pgm")) {
        require(chrono::steady_clock::now() < deadline, "RUN did not produce its first frame");
        this_thread::sleep_for(chrono::milliseconds(2));
    }
}

static void activeStop(Scratch& scratch, const string& identity, int signal_number, bool cli = false, bool broken_pipe = false)
{
    const string name = cli ? "cli-stop" : (broken_pipe ? "broken-pipe" : (signal_number ? "signal" : "active-eof"));
    const string folder = scratch.directory(name);
    vector<string> args = cli ? run(folder, 100, 30) : launch(identity);
    if (cli) args.erase(args.begin());
    Child child(scratch, name, args);
    if (!cli) { child.reply("HELLO"); child.send(run(folder, 100, 30)); }
    waitForFrame(folder);
    if (broken_pipe) { close(child.output); child.output = -1; }
    if (signal_number) require(kill(child.pid, signal_number) == 0, "kill failed");
    else child.closeInput();
    int frames = 0;
    if (broken_pipe) {
        require(child.wait() == 1, "broken reply pipe bypassed normal stop");
        for (const auto& name : files(folder)) if (name.size() >= 5 && name.substr(name.size()-4) == ".pgm") ++frames;
    } else if (!cli) {
        const auto reply = child.reply("DONE");
        frames = stoi(reply.values.at("frames"));
        require(reply.values.at("status") == "1" && frames > 0 && frames < 100 && !reply.values.at("error").empty(),
                "active stop did not return partial DONE status 1");
        child.expectEof();
    } else {
        while (!child.eof) { child.readMore(); child.lines.clear(); } // CLI diagnostics use stdout.
        for (const auto& name : files(folder)) if (name.size() >= 5 && name.substr(name.size()-4) == ".pgm") ++frames;
    }
    if (!broken_pipe) require(child.wait() == 1, "active stop did not preserve original SIGINT exit status");
    child.cleaned();
    vector<FrameTiming> rows;
    int settle;
    string error;
    require(read_frame_timing_csv(folder + "/frame_timing.csv", rows, settle, error), "partial timing missing: " + error);
    require(rows.size() >= static_cast<size_t>(frames) && rows.size() < 100, "incorrect partial timing");
    require(exists(folder + "/frame_timing_summary.json"), "partial summary missing");
}

// A write failure keeps the original -1 -> shell 255 mapping, then leaves the session usable.
static void failedRun(Scratch& scratch, const string& identity)
{
    const string failed = scratch.directory("write-failure"), valid = scratch.directory("after-failure");
    Child child(scratch, "failed-run", launch(identity), false, failed + "/i000.pgm");
    child.reply("HELLO");
    child.send(run(failed)); done(child, 255, 0);
    require(exists(failed + "/frame_timing.csv") && exists(failed + "/frame_timing_summary.json"), "failure timing absent");
    child.send(run(valid)); done(child, 0, 3);
    const string probe = scratch.directory("probe"), schedule = scratch.path + "/probe.csv";
    const string schedule_text = "vx_v,vy_v,settle_ms,label\n0.2,0.1,2,first\n-0.2,-0.1,3,second\n";
    put(schedule, schedule_text);
    auto command = run(probe);
    command.insert(command.end(), {"--FSM_SCHEDULE", schedule, "--FSM_OVERLAP", "1", "--AMPLI_REF", "1"});
    child.send(command); done(child, 0, 2);
    require(contents(probe + "/fsm_schedule.csv") == schedule_text, "session probe changed the schedule copy");
    require((files(probe) == set<string>{"fsm_schedule.csv", "fsm_frame_commands.csv", "frame_timing.csv",
            "frame_timing_summary.json", "i000.pgm", "i001.pgm"}), "incorrect probe file list/reference suppression");
    vector<FrameTiming> rows;
    string error;
    int settle;
    require(read_frame_timing_csv(probe + "/frame_timing.csv", rows, settle, error) && rows.size() == 2 &&
            rows[0].settle_ms == 2 && rows[1].settle_ms == 3, "probe lost per-frame settles");
    require(contents(probe + "/frame_timing_summary.json").find(fsm_schedule_sequential_reason) != string::npos,
            "probe did not force sequential capture");
    child.send({"QUIT"}); child.reply("BYE"); child.expectEof();
    require(child.wait() == 0, "failed RUN poisoned the session"); child.cleaned(); outputs(valid, 3);
}

// Inject after cleanup, using an arm file outside the run folder between RUNs.
static void takenOutputs(Scratch& scratch, const string& identity)
{
    const string folder = scratch.directory("taken-outputs"), arm = scratch.path + "/arm-output";
    const vector<string> names = {"i001.pgm", "frame_timing.csv", "fsm_frame_commands.csv",
                                  "Intensite_ref.pgm", "frame_timing_summary.json", "fsm_schedule.csv"};
    const string schedule = scratch.path + "/taken-schedule.csv";
    put(schedule, "vx_v,vy_v,settle_ms,label\n0,0,0,centre\n0.1,0.1,0,step\n");
    Child child(scratch, "taken-outputs", launch(identity), false, "", [&]() {
        const auto audit_event = test_hardware_event;
        test_hardware_event = [&, audit_event](const string& event, double x, double y) {
            audit_event(event, x, y);
            if (event != "dac" || !exists(arm)) return;
            istringstream names(contents(arm));
            require(unlink(arm.c_str()) == 0, "cannot disarm injection");
            string name;
            while (getline(names, name))
                require(symlink((scratch.path + "/outside-" + name).c_str(), (folder + "/" + name).c_str()) == 0,
                        "cannot plant output symlink");
        };
    });
    child.reply("HELLO");
    for (const string& name : names) {
        const string outside = scratch.path + "/outside-" + name;
        put(outside, "outside content " + name);
        put(arm, name);
        auto command = run(folder);
        if (name == "Intensite_ref.pgm") command.insert(command.end(), {"--AMPLI_REF", "1"});
        if (name == "fsm_schedule.csv") command.insert(command.end(), {"--FSM_SCHEDULE", schedule});
        child.send(command);
        const auto reply = child.reply("DONE");
        const bool before_camera = name == "fsm_frame_commands.csv" || name == "fsm_schedule.csv";
        require(reply.values.at("status") == (before_camera ? "1" : "255") &&
                reply.values.at("error").find(folder + "/" + name) != string::npos &&
                reply.values.at("error").find("nothing was overwritten") != string::npos,
                "taken output lacked its failure/name: " + contents(child.log));
        require(contents(outside) == "outside content " + name, "output write followed symlink " + name);
        struct stat info;
        require(lstat((folder + "/" + name).c_str(), &info) == 0 && S_ISLNK(info.st_mode),
                "output write replaced symlink " + name);
        child.send(run(folder)); done(child, 0, 3);
        outputs(folder, 3);
    }
    // Keep the first scan error and still warn about timing collisions after a failed scan.
    put(arm, "i001.pgm\nframe_timing.csv\nframe_timing_summary.json");
    child.send(run(folder));
    const auto reply = child.reply("DONE");
    require(reply.values.at("status") == "255" && reply.values.at("frames") == "1" &&
            reply.values.at("error").find("i001.pgm") != string::npos,
            "timing collision replaced the first scan error");
    const string log = contents(child.log);
    for (const string& name : {"frame_timing.csv", "frame_timing_summary.json"}) {
        const string label = name == "frame_timing.csv" ? "frame timing" : "frame timing summary";
        require(log.find("Warning: cannot write the " + label + ": refusing to write \"" + folder + "/" + name) != string::npos,
                "timing collision after scan failure did not warn");
    }
    for (const string& name : names)
        require(contents(scratch.path + "/outside-" + name) == "outside content " + name,
                "failed scan changed outside content " + name);
    child.send(run(folder)); done(child, 0, 3); outputs(folder, 3);
    child.send({"QUIT"}); child.reply("BYE"); child.expectEof();
    require(child.wait() == 0, "taken output poisoned session"); child.cleaned();
}

static void cliClearRefusal(Scratch& scratch, const string& folder, const string& name)
{
    auto args = run(folder); args.erase(args.begin());
    Child child(scratch, name, args);
    string diagnostic;
    while (!child.eof) {
        child.readMore();
        while (!child.lines.empty()) { diagnostic += child.lines.front() + '\n'; child.lines.pop_front(); }
    }
    require(child.wait() == 1, "CLI did not refuse the failed clear with exit 1");
    require(contents(child.log).find(folder) != string::npos, "CLI clear error omitted the folder");
    require(contents(child.audit).empty(), "CLI opened/moved hardware after a failed clear");
}

static void cleanupPreservation(Scratch& scratch, const string& identity)
{
    for (bool session : {false, true}) {
        const string name = session ? "session-cleanup" : "cli-cleanup", folder = scratch.directory(name);
        const vector<string> stale = {"i007.pgm", "i0000.pgm", "Intensite_ref.pgm", "frame_timing.csv",
            "frame_timing_summary.json", "fsm_frame_commands.csv", "fsm_frame_commands.csv.tmp",
            "fsm_schedule.csv", "fsm_schedule.csv.tmp"};
        for (const string& file : stale) put(folder + "/" + file, "stale");
        set<string> kept = {"notes.txt", "Mask.png", ".hidden", "i01.pgm", "i000.pgm.bak",
            "acquisition.conf", "processing.conf", "config/", "user-link"};
        for (const string& file : {"notes.txt", "Mask.png", ".hidden", "i01.pgm", "i000.pgm.bak",
                "acquisition.conf", "processing.conf"}) put(folder + "/" + file, "user content");
        require(mkdir((folder + "/config").c_str(), 0700) == 0, "cannot make preserved directory");
        put(folder + "/config/inside", "nested content");
        const string target = scratch.path + "/" + name + "-target";
        put(target, "outside target");
        require(symlink(target.c_str(), (folder + "/i009.pgm").c_str()) == 0, "cannot create own-output symlink");
        require(symlink("notes.txt", (folder + "/user-link").c_str()) == 0, "cannot create user symlink");
        require(symlink("missing-target", (folder + "/i011.pgm").c_str()) == 0, "cannot create dangling own-output symlink");
        // Exercise warning sorting and the limit without hiding any of the named user entries above.
        for (int k = 0; k < 25; ++k) {
            const string file = cv::format("z-user-%02d", k);
            put(folder + "/" + file, "user content"); kept.insert(file);
        }
        auto args = run(folder); args.erase(args.begin());
        Child child(scratch, name, session ? launch(identity) : args);
        if (session) {
            child.reply("HELLO"); child.send(run(folder)); done(child, 0, 3);
            child.send({"QUIT"}); child.reply("BYE"); child.expectEof();
        } else stdoutText(child);
        require(child.wait() == 0, "cleanup preservation run failed: " + contents(child.log)); child.cleaned();
        set<string> seeded;
        for (const string& entry : kept) seeded.insert(entry.back() == '/' ? entry.substr(0, entry.size() - 1) : entry);
        outputs(folder, 3, 32, false, CV_8UC1, seeded);
        for (const string& file : {"i007.pgm", "i0000.pgm", "Intensite_ref.pgm", "fsm_frame_commands.csv.tmp",
                "fsm_schedule.csv", "fsm_schedule.csv.tmp", "i009.pgm", "i011.pgm"}) {
            struct stat info;
            require(lstat((folder + "/" + file).c_str(), &info) != 0 && errno == ENOENT, "stale output survived: " + file);
        }
        require(contents(target) == "outside target", "cleanup followed an own-output symlink");
        for (const string& entry : kept) {
            const string path = folder + "/" + entry;
            struct stat info;
            require(lstat(path.c_str(), &info) == 0, "user entry disappeared: " + entry);
            if (entry.back() == '/') require(contents(path + "inside") == "nested content", "nested user file changed");
            else require(contents(path) == "user content", "user file changed: " + entry);
        }
        const string log = contents(child.log), prefix = "Warning: kept entries in \"" + folder + "\": ";
        string expected = prefix;
        size_t count = 0;
        for (const string& entry : kept) {
            if (count == 20) break;
            if (count++) expected += ", ";
            expected += '"' + entry + '"';
        }
        expected += ", ... and " + to_string(kept.size() - 20) + " more\n";
        const size_t position = log.find(expected);
        require(position != string::npos && log.find(prefix, position + expected.size()) == string::npos,
                "kept-entry warning was missing, unsorted, repeated or unbounded: " + log);
    }
    // CHEMIN_ACQUIS through a symlink: the cleanup works in the target folder and keeps its user files.
    const string target = scratch.directory("linked-cleanup-target"), link = scratch.path + "/linked-cleanup";
    require(symlink(target.c_str(), link.c_str()) == 0, "cannot create folder symlink");
    for (const string& file : {"i007.pgm", "frame_timing.csv"}) put(target + "/" + file, "stale");
    put(target + "/notes.txt", "user content");
    auto args = run(link); args.erase(args.begin());
    Child child(scratch, "linked-cleanup", args);
    stdoutText(child);
    require(child.wait() == 0, "cleanup through a folder symlink failed: " + contents(child.log)); child.cleaned();
    outputs(target, 3, 32, false, CV_8UC1, {"notes.txt"});
    struct stat info;
    require(lstat((target + "/i007.pgm").c_str(), &info) != 0 && errno == ENOENT,
            "stale output survived behind the symlink");
    require(contents(target + "/notes.txt") == "user content" && lstat(link.c_str(), &info) == 0 &&
            S_ISLNK(info.st_mode), "cleanup through a folder symlink changed a user file or the link");
}

static void clearRefusals(Scratch& scratch, const string& identity)
{
    const string valid = scratch.directory("after-clear-refusal");
    Child child(scratch, "clear-refusal", launch(identity));
    child.reply("HELLO");
    auto refused = [&](const string& folder, const string& reason) {
        const string before = contents(child.audit);
        child.send(run(folder));
        const auto reply = child.reply("DONE");
        require(reply.values.at("status") == "1" && reply.values.at("frames") == "0" &&
                reply.values.at("error").find(folder) != string::npos &&
                reply.values.at("error").find(reason) != string::npos, "failed clear lacked DONE status 1/folder/reason");
        require(contents(child.audit) == before, "failed clear moved hardware");
    };
    for (const string& archived : {"i000.jxl", "i003.png"}) {
        const string folder = scratch.directory("archived-" + archived);
        put(folder + "/" + archived, "archived frame");
        put(folder + "/i000.pgm", "earlier frame"); // Sorts before the PNG; must not be removed first.
        put(folder + "/frame_timing.csv", "earlier timing");
        refused(folder, '"' + archived + "\"; choose an empty folder");
        cliClearRefusal(scratch, folder, "cli-archived-" + archived);
        require(contents(folder + "/" + archived) == "archived frame" &&
                contents(folder + "/i000.pgm") == "earlier frame" &&
                contents(folder + "/frame_timing.csv") == "earlier timing" && files(folder).size() == 3,
                "archive refusal deleted or wrote files");
    }
    // An own-output name that is not a regular file or symlink would be written by the run: refused, nothing deleted.
    for (const string& blocker : {"i008.pgm", "i010.pgm", "frame_timing.csv"}) {
        const string folder = scratch.directory("not-regular-" + blocker);
        put(folder + "/i000.pgm", "earlier frame"); // Sorts first and is deletable; must survive the refusal.
        put(folder + "/notes.txt", "user content");
        const bool directory = blocker == "i008.pgm";
        if (directory) {
            require(mkdir((folder + "/" + blocker).c_str(), 0700) == 0, "cannot make own-output directory");
            put(folder + "/" + blocker + "/inside", "nested content");
        } else require(mkfifo((folder + "/" + blocker).c_str(), 0600) == 0, "cannot create own-output FIFO");
        refused(folder, "own output name \"" + blocker + "\" is not a regular file; choose an empty folder");
        cliClearRefusal(scratch, folder, "cli-not-regular-" + blocker);
        struct stat info;
        require(lstat((folder + "/" + blocker).c_str(), &info) == 0 &&
                (directory ? S_ISDIR(info.st_mode) && contents(folder + "/" + blocker + "/inside") == "nested content"
                           : S_ISFIFO(info.st_mode)) &&
                contents(folder + "/i000.pgm") == "earlier frame" && contents(folder + "/notes.txt") == "user content" &&
                files(folder).size() == 3, "not-regular own-output refusal deleted or wrote entries");
        require(contents(scratch.path + "/cli-not-regular-" + blocker + ".log").find("Warning: kept entries") == string::npos,
                "a refused cleanup printed a partial kept-entry warning");
    }
    refused(scratch.path + "/missing-folder", "opendir");
    cliClearRefusal(scratch, scratch.path + "/missing-folder", "cli-missing-folder");
    if (geteuid() != 0) {
        const string unreadable = scratch.directory("unreadable");
        put(unreadable + "/sentinel", "untouched");
        {
            struct RestoreMode {
                const string& path;
                ~RestoreMode() { chmod(path.c_str(), 0700); }
            } restore{unreadable};
            require(chmod(unreadable.c_str(), 0) == 0, "cannot make folder unreadable");
            refused(unreadable, "opendir");
            cliClearRefusal(scratch, unreadable, "cli-unreadable");
        }
        require(contents(unreadable + "/sentinel") == "untouched", "unreadable folder was cleared");
        const string blocked = scratch.directory("failed-removal");
        put(blocked + "/i007.pgm", "stale frame");
        {
            struct RestoreMode {
                const string& path;
                ~RestoreMode() { chmod(path.c_str(), 0700); }
            } restore{blocked};
            require(chmod(blocked.c_str(), 0500) == 0, "cannot make folder unwritable");
            refused(blocked, "i007.pgm");
            cliClearRefusal(scratch, blocked, "cli-failed-removal");
        }
        require(contents(blocked + "/i007.pgm") == "stale frame" && files(blocked).size() == 1,
                "failed unlink continued the run");
    } else {
        cout << "SKIP assertion: root bypasses folder permission checks.\n";
    }
    // A folder-open failure leaves the connection usable for a successful RUN and normal cleanup.
    child.send(run(valid)); done(child, 0, 3);
    child.send({"QUIT"}); child.reply("BYE"); child.expectEof();
    require(child.wait() == 0, "failed clear poisoned the session"); child.cleaned(); outputs(valid, 3);
}

static void startupCases(Scratch& scratch, const string& identity, const string& device_id)
{
    {
        Child child(scratch, "driver-exit", launch(identity), true);
        require(child.reply("ERROR").values.at("message").find("hardware driver") != string::npos,
                "direct driver exit omitted startup ERROR");
        child.expectEof(); require(child.wait() == 1, "direct driver exit changed status");
        require(contents(child.audit) == "dac 0 0\n", "direct driver exit skipped the DAC park");
    }
    {
        auto args = launch(identity); args.erase(args.begin() + 2, args.begin() + 4);
        Child child(scratch, "single-no-id", args);
        require(child.reply("HELLO").values.at("camera") == identity, "single-camera selection failed");
        child.send({"QUIT"}); child.reply("BYE"); child.expectEof();
        require(child.wait() == 0, "single-camera session failed"); child.cleaned();
    }
    {
        Child child(scratch, "missing-id", launch("does-not-exist"));
        require(child.reply("ERROR").values.at("message") == "selected camera does-not-exist not found",
                "startup camera selection failure lacks the requested identity");
        child.expectEof(); require(child.wait() == 255, "camera failure exit code changed");
        require(contents(child.audit) == "dac 0 0\n", "startup camera failure did not park DAC");
    }
    {
        // Path resolution is still required before opening the connection.
        put(scratch.config + "/config_manip.txt", "DIM_ROI 32\n");
        Child child(scratch, "invalid-config", launch(identity));
        child.reply("ERROR"); child.expectEof();
        require(child.wait() == 1 && contents(child.audit).empty(), "invalid startup config opened hardware");
        require(remove((scratch.config + "/config_manip.txt").c_str()) == 0, "cannot remove ambiguous config");
    }
    {
        const string folder = scratch.directory("mono16");
        Child child(scratch, "device-id-mono16", launch(device_id, "Mono16"));
        const auto hello = child.reply("HELLO");
        require(hello.values.at("camera") == identity && hello.values.at("format") == "Mono16", "device-id/format selection failed");
        auto command = run(folder); command.insert(command.end(), {"--PIXEL_FORMAT", "Mono16"});
        child.send(command); done(child, 0, 3);
        child.send({"QUIT"}); child.reply("BYE"); child.expectEof();
        require(child.wait() == 0, "Mono16 session failed"); child.cleaned();
        outputs(folder, 3, 32, false, CV_16UC1);
    }
}

static void adapterFreshness(ArvCamera* camera)
{
    const RunFolder folder(".");
    const vector<ScanFrameCommand> commands;
    FrameLoopOptions options;
    AcquisitionFrameIO first(camera, NULL, NULL, folder, commands, options, 1, CV_8UC1);
    first.first_latch.valid = true; first.first_latch.camera_ns = 12345;
    first.frames = 7; first.doodle_index = 4;
    this_thread::sleep_for(chrono::milliseconds(2));
    AcquisitionFrameIO second(camera, NULL, NULL, folder, commands, options, 1, CV_8UC1);
    require(second.origin > first.origin && second.now_ms() < first.now_ms(), "run adapter reused the clock origin");
    require(!second.first_latch.valid && second.first_latch.camera_ns == 0 && second.frames == 0 && second.doodle_index == 0,
            "run adapter retained latch/progress state");
}

static void timingOutputFailures(Scratch& scratch, ArvCamera* camera)
{
    const vector<ScanFrameCommand> commands;
    FrameLoopOptions options;
    options.exposure_us = 1000;
    vector<FrameTiming> rows(3);
    for (size_t k = 0; k < rows.size(); ++k) {
        FrameTiming& row = rows[k];
        row.image_index = k;
        row.frame_id = k + 1;
        row.camera_timestamp_ns = 1000000000ULL + k * 10000000ULL;
        row.dac_start_ms = k * 10;
        row.dac_done_ms = row.dac_start_ms + 0.5;
        row.trigger_start_ms = row.dac_start_ms + 1;
        row.trigger_done_ms = row.dac_start_ms + 1.1;
        row.buffer_ms = row.dac_start_ms + 3;
        row.written_ms = row.dac_start_ms + 4;
    }
    require(!check_frame_timing(rows, options).suspect, "timing output fixture has invalid rows");
    auto finish = [&](AcquisitionFrameIO& io, bool success) {
        const string log = scratch.path + "/timing-finish.log";
        {
            cerr.flush(); fflush(stderr);
            const int saved = dup(STDERR_FILENO);
            require(saved >= 0, "cannot save stderr for timing test");
            struct RestoreStderr {
                int fd;
                ~RestoreStderr() { cerr.flush(); fflush(stderr); dup2(fd, STDERR_FILENO); close(fd); }
            } restore{saved};
            const int output = open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
            require(output >= 0, "cannot open timing test log");
            const int redirected = dup2(output, STDERR_FILENO);
            close(output);
            require(redirected >= 0, "cannot redirect timing test stderr");
            io.finish(rows, success, options);
        }
        return contents(log);
    };
    {
        const string path = scratch.directory("timing-read-only");
        const RunFolder folder(path);
        require(folder.fd() >= 0, "cannot open read-only timing test folder");
        struct RestoreMode {
            const string& path;
            ~RestoreMode() { chmod(path.c_str(), 0700); }
        } restore{path};
        require(chmod(path.c_str(), 0500) == 0, "cannot make timing folder read-only");
        const int probe = openat(folder.fd(), "permission-probe", O_WRONLY | O_CREAT | O_EXCL | O_NOFOLLOW, 0600);
        if (probe >= 0) {
            close(probe);
            require(unlinkat(folder.fd(), "permission-probe", 0) == 0, "cannot remove timing permission probe");
            cout << "SKIP assertion: process can write to a 0500 directory (root/permission override); timing I/O case skipped.\n";
        } else {
            require(errno == EACCES, "timing permission probe failed for a reason other than EACCES");
            AcquisitionFrameIO io(camera, NULL, NULL, folder, commands, options, 1, CV_8UC1);
            const string log = finish(io, true);
            require(io.output_error.empty(), "timing I/O warning failed an otherwise completed scan");
            for (const string& label : {"frame timing", "frame timing summary"})
                require(log.find("Warning: cannot write the " + label + ": ") != string::npos,
                        "timing I/O error omitted warning for " + label);
            require(log.find(strerror(EACCES)) != string::npos && files(path).empty(),
                    "read-only timing failure did not report EACCES or created output files");
        }
    }
    {
        const string path = scratch.directory("timing-taken"), outside = scratch.path + "/timing-outside";
        put(outside, "outside timing sentinel");
        require(symlink(outside.c_str(), (path + "/frame_timing.csv").c_str()) == 0, "cannot plant timing symlink");
        const RunFolder folder(path);
        require(folder.fd() >= 0, "cannot open taken timing test folder");
        AcquisitionFrameIO io(camera, NULL, NULL, folder, commands, options, 1, CV_8UC1);
        finish(io, true);
        require(io.output_error.find(path + "/frame_timing.csv") != string::npos &&
                io.output_error.find("nothing was overwritten") != string::npos, "taken timing name did not fail the scan");
        require(contents(outside) == "outside timing sentinel", "timing write changed outside target");
        require(files(path) == set<string>{"frame_timing.csv", "frame_timing_summary.json"} &&
                contents(path + "/frame_timing_summary.json").find("\"frames\": 3") != string::npos,
                "taken CSV prevented timing summary from being written");
    }
    {
        const RunFolder folder(scratch.directory("timing-incomplete"));
        require(folder.fd() >= 0, "cannot open incomplete timing test folder");
        AcquisitionFrameIO io(camera, NULL, NULL, folder, commands, options, 1, CV_8UC1);
        require(io.output_error.empty(), "fresh timing adapter retained an error");
        finish(io, false);
        require(io.output_error == "hologram acquisition did not complete", "failed scan lost its default error");
    }
}

static void cliRun(Scratch& scratch, bool no_roi)
{
    scratch.writeConfig(no_roi);
    const string name = no_roi ? "cli-roi-free" : "cli";
    const string folder = scratch.directory(name);
    auto args = run(folder); args.erase(args.begin());
    Child child(scratch, name, args);
    string diagnostic;
    while (!child.eof) {
        child.readMore();
        while (!child.lines.empty()) { diagnostic += child.lines.front() + '\n'; child.lines.pop_front(); }
    }
    require(child.wait() == 0, "CLI acquisition failed: " + diagnostic + contents(child.log)); child.cleaned();
    outputs(folder, 3, no_roi ? 512 : 32);
    // The named folder is the only output: gui_tomo.conf's CHEMIN_ACQUIS keeps argumentSafety's sentinel alone.
    require(files(scratch.path + "/default") == set<string>{"i777.pgm"} &&
            contents(scratch.path + "/default/i777.pgm") == "default untouched", "CLI run touched gui_tomo.conf's folder");
    if (no_roi) require(diagnostic.find("keeping camera's current ROI") != string::npos, "ROI-free behavior changed");
    scratch.writeConfig(false);
}

int main()
{
    try {
        signal(SIGPIPE, SIG_IGN);
        Scratch scratch;
        setenv("HOME", scratch.path.c_str(), 1);
        setenv("XDG_CONFIG_HOME", (scratch.path + "/.config").c_str(), 1);
        setenv("XDG_CACHE_HOME", (scratch.path + "/cache").c_str(), 1);
        setenv("XDG_DATA_HOME", (scratch.path + "/data").c_str(), 1);
        argumentSafety(scratch);
        bool has_fake = false;
        for (unsigned int i = 0; i < arv_get_n_interfaces(); ++i)
            has_fake = has_fake || string(arv_get_interface_id(i)) == "Fake";
        if (!has_fake) { cout << "SKIP: Aravis Fake interface missing\n"; return 77; }
        fakeOnly(); arv_update_device_list();
        require(arv_get_n_devices() == 1, "Fake interface did not expose exactly one camera");
        const string identity = arv_get_device_serial_nbr(0), device_id = arv_get_device_id(0);
        GError* error = NULL;
        ArvCamera* camera = arv_camera_new(device_id.c_str(), &error);
        require(camera && !error, "cannot open Fake camera");
        const double exposure_us = arv_camera_get_exposure_time(camera, &error);
        require(!error && std::isfinite(exposure_us) && exposure_us > 0, "cannot read Fake camera exposure");
        adapterFreshness(camera);
        timingOutputFailures(scratch, camera);
        require(!arv_device_get_feature(arv_camera_get_device(camera), "ExposureAuto") &&
                !arv_device_get_feature(arv_camera_get_device(camera), "TimestampLatch"), "Fake capabilities changed; extend the fixture");
        g_object_unref(camera);
        arv_shutdown();
        startupCases(scratch, identity, device_id);
        protocolRuns(scratch, identity, exposure_us);
        deferredRunConfig(scratch, identity);
        randomPolarStops(scratch, identity);
        cliPromptStop(scratch);
        eofAfterRun(scratch, identity);
        clearRefusals(scratch, identity);
        cleanupPreservation(scratch, identity);
        failedRun(scratch, identity);
        takenOutputs(scratch, identity);
        idleEof(scratch, identity, false);
        idleEof(scratch, identity, true);
        activeStop(scratch, identity, 0);
        activeStop(scratch, identity, 0, false, true);
        activeStop(scratch, identity, SIGTERM);
        activeStop(scratch, identity, SIGINT, true);
        cliRun(scratch, false);
        cliRun(scratch, true);
        cout << "Session/CLI tests passed with Aravis Fake and simulated LabJack.\n"
             << "Fake lacks ExposureAuto and TimestampLatch/Value: absent auto is reported off; latch checks use the absent-node path.\n";
    } catch (const exception& error) { cerr << error.what() << '\n'; return 1; }
}
