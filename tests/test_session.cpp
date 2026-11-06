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

static vector<FrameTiming> outputs(const string& folder, int count, int dimension = 32, bool reference = false, int cv_type = CV_8UC1)
{
    set<string> expected = {"fsm_frame_commands.csv", "frame_timing.csv", "frame_timing_summary.json"};
    for (int k = 0; k < count; ++k) {
        const string name = cv::format("i%03d.pgm", k);
        expected.insert(name);
        const cv::Mat image = imread(folder + "/" + name, IMREAD_UNCHANGED);
        require(!image.empty() && image.rows == dimension && image.cols == dimension && image.type() == cv_type, "wrong frame geometry: " + name);
    }
    if (reference) expected.insert("Intensite_ref.pgm");
    require(files(folder) == expected, "unexpected file list in " + folder);
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
    const auto first_rows = outputs(first, 3);
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
    const auto second_rows = outputs(second, 4);
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
        outputs(folder, 3);
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
    command.insert(command.end(), {"--SCAN_PATTERN", "RANDOM_POLAR", "--NXMAX", "4"});
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

static void clearRefusals(Scratch& scratch, const string& identity)
{
    const string blocked = scratch.directory("failed-removal"), valid = scratch.directory("after-clear-refusal");
    require(mkdir((blocked + "/nonempty").c_str(), 0700) == 0, "cannot create removal failure");
    put(blocked + "/nonempty/sentinel", "untouched");
    Child child(scratch, "clear-refusal", launch(identity));
    child.reply("HELLO");
    auto refused = [&](const string& folder) {
        const string before = contents(child.audit);
        child.send(run(folder));
        const auto reply = child.reply("DONE");
        require(reply.values.at("status") == "1" && reply.values.at("frames") == "0" &&
                reply.values.at("error").find(folder) != string::npos, "failed clear lacked DONE status 1/folder error");
        require(contents(child.audit) == before, "failed clear moved hardware");
    };
    child.send(run(blocked)); done(child, 0, 3);
    require(contents(blocked + "/nonempty/sentinel") == "untouched" && exists(blocked + "/i002.pgm") &&
            contents(child.log).find("Warning: cannot remove " + blocked + "/nonempty") != string::npos,
            "failed removal did not warn, keep the subdirectory and continue the RUN");
    auto args = run(blocked); args.erase(args.begin());
    {
        Child cli(scratch, "cli-failed-removal", args);
        while (!cli.eof) { cli.readMore(); cli.lines.clear(); }
        require(cli.wait() == 0, "CLI did not continue after a failed removal"); cli.cleaned();
        require(contents(blocked + "/nonempty/sentinel") == "untouched" && exists(blocked + "/i002.pgm") &&
                contents(cli.log).find("Warning: cannot remove " + blocked + "/nonempty") != string::npos,
                "CLI failed removal did not preserve the subdirectory and warn");
    }
    refused(scratch.path + "/missing-folder");
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
            refused(unreadable);
            cliClearRefusal(scratch, unreadable, "cli-unreadable");
        }
        require(contents(unreadable + "/sentinel") == "untouched", "unreadable folder was cleared");
    } else {
        cout << "SKIP assertion: root can read mode-0 folders; removal-failure checks still run.\n";
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
    const string path = "unused";
    const vector<ScanFrameCommand> commands;
    FrameLoopOptions options;
    AcquisitionFrameIO first(camera, NULL, NULL, path, commands, options, 1, CV_8UC1);
    first.first_latch.valid = true; first.first_latch.camera_ns = 12345;
    first.frames = 7; first.doodle_index = 4;
    this_thread::sleep_for(chrono::milliseconds(2));
    AcquisitionFrameIO second(camera, NULL, NULL, path, commands, options, 1, CV_8UC1);
    require(second.origin > first.origin && second.now_ms() < first.now_ms(), "run adapter reused the clock origin");
    require(!second.first_latch.valid && second.first_latch.camera_ns == 0 && second.frames == 0 && second.doodle_index == 0,
            "run adapter retained latch/progress state");
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
        failedRun(scratch, identity);
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
