#include "acquisition_protocol.h" // Must compile without any other includes first.
#include <cassert>
#include <climits>
#include <sys/wait.h>
#include <unistd.h>

using namespace acquisition_protocol;

int main()
{
    assert(session_protocol_version == 1);
    assert(std::string(session_protocol_marker) == "TDM_SESSION_PROTOCOL=1");
    const std::vector<std::string> words = {"plain", "two words", "quote\"here", "back\\slash", "", "µλ波長", "a\tb", "="};
    std::string error;
    std::vector<std::string> parsed;
    assert(splitWords(joinWords(words), parsed, error) && error.empty() && parsed == words);
    for (const auto& word : words) {
        assert(splitWords(quoteWord(word), parsed, error));
        assert(parsed == std::vector<std::string>{word});
    }
    assert(quoteWord("plain") == "plain" && quoteWord("") == "\"\"");
    assert(splitWords(" \t  ", parsed, error) && parsed.empty());
    assert(splitWords("a\tb  c", parsed, error) && (parsed == std::vector<std::string>{"a", "b", "c"}));
    for (const std::string line : {"\"unfinished", "\"slash\\", "trailing\\", "a\\b", "\"a\"glued", "a\"b\"", "\"bad\\n\"", "\"bad\\t\"", "\"bad\\q\""}) {
        parsed = {"previous"};
        assert(!splitWords(line, parsed, error) && parsed.empty() && !error.empty());
    }
    for (int c = 0; c <= 127; ++c) {
        if (!(c < 32 || c == 127) || c == '\t') continue;
        const std::string bad = "a" + std::string(1, static_cast<char>(c)) + "b";
        bool threw = false;
        try { quoteWord(bad); } catch (const std::invalid_argument&) { threw = true; }
        assert(threw);
        assert(!splitWords(bad, parsed, error));
        assert(!splitWords('"' + bad + '"', parsed, error));
    }
    LineReader reader;
    assert(reader.append("HE").empty());
    assert((reader.append("LLO\r\nRUN \"two ") == std::vector<std::string>{"HELLO"}));
    assert(reader.append("words\"").empty());
    assert((reader.append("\n\nQUIT\r\nleft") == std::vector<std::string>{"RUN \"two words\"", "", "QUIT"}));
    std::string incomplete;
    assert(!reader.finish(incomplete) && incomplete == "left");
    assert(reader.finish(incomplete) && incomplete.empty());
    assert(reader.append("\r").empty());
    assert(!reader.finish(incomplete) && incomplete == "\r");
    assert((reader.append("a\r\r\n") == std::vector<std::string>{"a\r"}));
    assert(!splitWords("a\r", parsed, error)); // Only one CR is removed.

    Reply reply;
    assert(parseReply("DONE status=255 frames=2 error=\"two words\" future=\"a\\\"b\\\\c\"", reply, error));
    assert(reply.keyword == "DONE" && reply.values.size() == 4 && reply.values.at("future") == "a\"b\\c");
    assert(reply.values.at("error") == "two words");
    const auto values = reply.values;
    assert(parseReply(formatReply("DONE", values), reply, error) && reply.values == values);
    assert(parseReply("EXPOSURE us=1234 auto=0 error=\"\"", reply, error) && reply.values.at("error").empty());
    assert(parseReply("BYE", reply, error) && reply.keyword == "BYE" && reply.values.empty());
    for (const std::string line : {"", "1BAD", "DONE status", "DONE =0", "DONE 1key=0", "DONE key=", "DONE key=\"a\"b", "DONE key=\"a", "DONE key=bad\\", "DONE key=\"bad\\x\"", "DONE key=x key=y", "DONE key=x key=\"x\"", "DONE bad-key=x", "DONE key=\n"}) {
        assert(!parseReply(line, reply, error) && !error.empty() && reply.keyword.empty() && reply.values.empty());
    }
    for (const std::string key : {"", "two words", "a=b", "bad-key", "1key"}) {
        bool threw = false;
        try { formatReply("DONE", {{key, "value"}}); } catch (const std::invalid_argument&) { threw = true; }
        assert(threw);
        threw = false;
        try { formatReply(key); } catch (const std::invalid_argument&) { threw = true; }
        assert(threw);
    }
    assert(doneStatus(-1) == 255 && doneStatus(0) == 0 && doneStatus(1) == 1);
    assert(doneStatus(256) == 0 && doneStatus(257) == 1 && doneStatus(-257) == 255);
    // Verify the mapping against actual process exit decoding, including values outside 0..255.
    for (int code : {-1, 0, 1, 255, 256, 257, -257, INT_MIN, INT_MAX}) {
        const pid_t child = fork();
        assert(child >= 0);
        if (child == 0) _exit(code);
        int status = 0;
        assert(waitpid(child, &status, 0) == child && WIFEXITED(status));
        assert(doneStatus(code) == WEXITSTATUS(status));
    }
}
