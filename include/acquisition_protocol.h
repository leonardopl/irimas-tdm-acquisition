#pragma once

// Shared acquisition program line protocol. C++11, header-only; no camera, GUI or platform dependencies.
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace acquisition_protocol {

constexpr int session_protocol_version = 1;
constexpr char session_protocol_marker[] = "TDM_SESSION_PROTOCOL=1";

enum class KeyScope { LaunchOnly, Connection, Run };
enum class ArgumentOrigin { Launch, Run };

struct ArgumentKey {
    const char* name;
    KeyScope scope;
    const char* meaning;
};

// The command-line vocabulary, also used by help and by clients validating RUN lines.
inline const std::vector<ArgumentKey>& argumentKeys()
{
    static const std::vector<ArgumentKey> keys = {
        {"SESSION", KeyScope::LaunchOnly, "1 opens session mode; no other value is accepted"},
        {"CHEMIN_CONFIG", KeyScope::Connection, "configuration folder"},
        {"CAMERA_ID", KeyScope::Connection, "camera serial number or Aravis device id"},
        {"DIM_ROI", KeyScope::Connection, "square camera ROI dimension in pixels; -1 with offsets -1 keeps the current ROI"},
        {"OFFSET_ROI_X", KeyScope::Connection, "camera ROI X offset in pixels"},
        {"OFFSET_ROI_Y", KeyScope::Connection, "camera ROI Y offset in pixels"},
        {"PIXEL_FORMAT", KeyScope::Connection, "Mono8, Mono12 or Mono16 (default Mono8)"},
        {"CHEMIN_ACQUIS", KeyScope::Run, "output folder; its earlier acquisition files are deleted first"},
        {"NB_HOLO", KeyScope::Run, "number of holograms"},
        {"SCAN_PATTERN", KeyScope::Run, "scan pattern name"},
        {"NB_CIRCLES_ANNULAR", KeyScope::Run, "number of circles for ANNULAR"},
        {"SCAN_SEED", KeyScope::Run, "optional unsigned seed for RANDOM_POLAR"},
        {"VXMIN", KeyScope::Run, "minimum mirror X voltage"},
        {"VXMAX", KeyScope::Run, "maximum mirror X voltage"},
        {"VYMIN", KeyScope::Run, "minimum mirror Y voltage"},
        {"VYMAX", KeyScope::Run, "maximum mirror Y voltage"},
        {"NA_COND_LIM", KeyScope::Run, "condenser NA limit in (0, 1]"},
        {"NXMAX", KeyScope::Run, "crop half-width in Fourier bins; absent: derived from CROP_NA (or NA), LAMBDA, TPCAM, RF, F_OBJ, F_TUBE, DIM_ROI"},
        {"CROP_NA", KeyScope::Run, "crop numerical aperture for derived NXMAX; absent: NA"},
        {"NA", KeyScope::Run, "objective numerical aperture used when CROP_NA is absent"},
        {"LAMBDA", KeyScope::Run, "wavelength in metres for derived NXMAX"},
        {"TPCAM", KeyScope::Run, "camera pixel pitch in metres for derived NXMAX"},
        {"RF", KeyScope::Run, "relay factor for derived NXMAX"},
        {"F_OBJ", KeyScope::Run, "objective focal length in metres for derived NXMAX"},
        {"F_TUBE", KeyScope::Run, "tube lens focal length in metres for derived NXMAX"},
        {"FSM_SETTLE_MS", KeyScope::Run, "DAC-to-exposure delay in milliseconds; absent: derived from command steps"},
        {"FSM_OVERLAP", KeyScope::Run, "0 sequential (default), 1 overlaps transfer with the next mirror move"},
        {"FSM_OVERLAP_MARGIN_US", KeyScope::Run, "exposure guard in microseconds (default 500)"},
        {"FSM_SCHEDULE", KeyScope::Run, "command line only: schedule CSV replacing the scan pattern and NB_HOLO"},
        {"AMPLI_REF", KeyScope::Run, "1 also records Intensite_ref.pgm (except with FSM_SCHEDULE)"}
    };
    return keys;
}

inline bool keyAllowed(KeyScope scope, ArgumentOrigin origin, bool session_launch = false)
{
    if (scope == KeyScope::LaunchOnly) return origin == ArgumentOrigin::Launch;
    return scope == KeyScope::Connection || !session_launch;
}

inline std::string validKeys(ArgumentOrigin origin, bool session_launch = false)
{
    std::string result;
    for (const auto& key : argumentKeys()) {
        if (!keyAllowed(key.scope, origin, session_launch)) continue;
        if (!result.empty()) result += ", ";
        result += std::string("--") + key.name;
    }
    return result;
}

// Safe for one-line diagnostics even when a CLI word or a filename contains controls.
inline std::string diagnosticWord(const std::string& word)
{
    static const char hex[] = "0123456789abcdef";
    std::string result = "\"";
    for (unsigned char c : word) {
        if (c < 32 || c == 127) {
            result += "\\x";
            result += hex[c >> 4]; result += hex[c & 15];
        } else {
            if (c == '"' || c == '\\') result += '\\';
            result += static_cast<char>(c);
        }
    }
    return result + '"';
}

// Help anywhere in launch words takes precedence over all argument errors and side effects.
inline bool helpRequested(const std::vector<std::string>& words)
{
    for (const auto& word : words)
        if (word == "--help" || word == "-h") return true;
    return false;
}

// Launch words ask for session mode when a key position holds --SESSION with the value 1.
inline bool sessionRequested(const std::vector<std::string>& words)
{
    for (std::size_t i = 0; i + 1 < words.size(); i += 2)
        if (words[i] == "--SESSION" && words[i + 1] == "1") return true;
    return false;
}

// Values are opaque: preserve blanks, '#', negative numbers and empty strings verbatim.
// On failure the output map is empty, and the error includes the context's complete vocabulary.
inline bool parseArguments(const std::vector<std::string>& words, ArgumentOrigin origin,
                           std::map<std::string, std::string>& args, std::string& error)
{
    args.clear();
    error.clear();
    const bool session_launch = origin == ArgumentOrigin::Launch && sessionRequested(words);
    const auto fail = [&](const std::string& reason) -> bool {
        error = reason + "; valid keys: " + validKeys(origin, session_launch);
        return false;
    };
    std::map<std::string, std::string> parsed;
    for (std::size_t i = 0; i < words.size(); i += 2) {
        const std::string& word = words[i];
        const ArgumentKey* key = nullptr;
        if (word.compare(0, 2, "--") == 0)
            for (const auto& candidate : argumentKeys())
                if (word.substr(2) == candidate.name) { key = &candidate; break; }
        if (!key) return fail("invalid argument " + diagnosticWord(word) + "; use --KEY VALUE");
        if (!keyAllowed(key->scope, origin, session_launch))
            return fail(diagnosticWord(word) + (session_launch ? " belongs to RUN, not session launch" : " is launch only, not allowed in RUN"));
        if (i + 1 == words.size() || words[i + 1].compare(0, 2, "--") == 0)
            return fail("missing value for " + diagnosticWord(word) + "; use --KEY VALUE");
        if (parsed.count(key->name)) return fail("duplicate key " + diagnosticWord(word));
        if (key->scope == KeyScope::LaunchOnly && words[i + 1] != "1")
            return fail("invalid value " + diagnosticWord(words[i + 1]) + " for " + diagnosticWord(word) + "; expected 1");
        parsed.emplace(key->name, words[i + 1]);
    }
    args.swap(parsed);
    return true;
}

// Process exit status, including negative and out-of-range returns, as observed by a shell.
// In particular -1 -> 255, 256 -> 0, and 257 -> 1. -1 is never a wire status.
inline int doneStatus(int main_return) { return static_cast<unsigned int>(main_return) & 0xffu; }

namespace detail {
inline bool blank(char c) { return c == ' ' || c == '\t'; }
inline bool control(unsigned char c) { return (c < 32 && c != '\t') || c == 127; }
inline bool fail(std::string& error, const char* message) { error = message; return false; }

// Read exactly one word. Tabs are separators outside quotes and literal data inside quotes.
// Backslashes are permitted only inside quotes, escaping a quote or another backslash.
inline bool word(const std::string& text, std::size_t& pos, std::string& value, std::string& error)
{
    value.clear();
    if (pos == text.size() || blank(text[pos])) return fail(error, "missing word");
    if (text[pos] == '"') {
        ++pos;
        while (pos < text.size()) {
            char c = text[pos++];
            if (c == '"') {
                if (pos < text.size() && !blank(text[pos]))
                    return fail(error, "text after closing quote");
                return true;
            }
            if (c == '\\') {
                if (pos == text.size()) return fail(error, "trailing backslash");
                c = text[pos++];
                if (c != '"' && c != '\\') return fail(error, "invalid escape");
            }
            if (control(static_cast<unsigned char>(c))) return fail(error, "control character");
            value += c;
        }
        return fail(error, "unterminated quote");
    }
    while (pos < text.size() && !blank(text[pos])) {
        const char c = text[pos++];
        if (c == '"' || c == '\\') return fail(error, "quote or backslash in bare word");
        if (control(static_cast<unsigned char>(c))) return fail(error, "control character");
        value += c;
    }
    return true;
}

inline bool identifier(const std::string& text)
{
    if (text.empty()) return false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
        if (!letter && c != '_' && !(i && c >= '0' && c <= '9')) return false;
    }
    return true;
}
} // namespace detail

// Encode one word, throwing invalid_argument for CR/LF, NUL and other controls (except tab).
// Non-ASCII bytes, including UTF-8, are preserved without interpretation.
inline std::string quoteWord(const std::string& word)
{
    bool quote = word.empty();
    for (unsigned char c : word) {
        if (detail::control(c)) throw std::invalid_argument("control character in session word");
        quote = quote || detail::blank(c) || c == '"' || c == '\\';
    }
    if (!quote) return word;
    std::string result = "\"";
    for (char c : word) {
        if (c == '"' || c == '\\') result += '\\';
        result += c;
    }
    return result + '"';
}

// Join words with spaces; every accepted word round-trips through splitWords.
inline std::string joinWords(const std::vector<std::string>& words)
{
    std::string result;
    for (std::size_t i = 0; i < words.size(); ++i) {
        if (i) result += ' ';
        result += quoteWord(words[i]);
    }
    return result;
}

// Decode a complete line (without CR/LF). On failure, words is empty and error explains why.
inline bool splitWords(const std::string& line, std::vector<std::string>& words, std::string& error)
{
    words.clear();
    error.clear();
    std::vector<std::string> parsed;
    std::size_t pos = 0;
    while (pos < line.size()) {
        if (detail::blank(line[pos])) { ++pos; continue; }
        std::string value;
        if (!detail::word(line, pos, value, error)) return false;
        parsed.push_back(value);
    }
    words.swap(parsed);
    return true;
}

// Incremental framing: append yields only newline-terminated lines, removing one CR before LF.
// finish discards any remaining bytes and returns false (with those bytes) for an incomplete EOF.
// Such bytes must never be executed or parsed. An empty EOF returns true.
class LineReader {
    std::string pending_;
public:
    std::vector<std::string> append(const char* bytes, std::size_t count) {
        pending_.append(bytes, count);
        std::vector<std::string> lines;
        std::size_t start = 0, end;
        while ((end = pending_.find('\n', start)) != std::string::npos) {
            std::string line = pending_.substr(start, end - start);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            lines.push_back(line);
            start = end + 1;
        }
        pending_.erase(0, start);
        return lines;
    }
    std::vector<std::string> append(const std::string& bytes) { return append(bytes.data(), bytes.size()); }
    bool finish(std::string& incomplete) {
        incomplete.swap(pending_);
        pending_.clear();
        return incomplete.empty();
    }
};

struct Reply {
    std::string keyword;
    std::map<std::string, std::string> values; // Ordered by key; unknown keys are retained.
};

// Encode a reply without its terminating newline. Invalid identifiers/values throw invalid_argument.
inline std::string formatReply(const std::string& keyword, const std::map<std::string, std::string>& values = {})
{
    if (!detail::identifier(keyword)) throw std::invalid_argument("invalid reply keyword");
    std::string result = keyword;
    for (const auto& pair : values) {
        if (!detail::identifier(pair.first)) throw std::invalid_argument("invalid reply key");
        result += ' ' + pair.first + '=' + quoteWord(pair.second);
    }
    return result;
}

// Decode KEYWORD key=value pairs, accepting bare/quoted values and rejecting duplicate keys.
// On failure, reply is empty. The input must already have passed line framing.
inline bool parseReply(const std::string& line, Reply& reply, std::string& error)
{
    reply = Reply();
    error.clear();
    Reply parsed;
    std::size_t pos = 0;
    while (pos < line.size() && detail::blank(line[pos])) ++pos;
    if (!detail::word(line, pos, parsed.keyword, error)) return false;
    if (!detail::identifier(parsed.keyword)) return detail::fail(error, "invalid reply keyword");
    while (pos < line.size()) {
        if (detail::blank(line[pos])) { ++pos; continue; }
        const std::size_t begin = pos;
        while (pos < line.size() && line[pos] != '=' && !detail::blank(line[pos])) ++pos;
        const std::string key = line.substr(begin, pos - begin);
        if (pos == line.size() || line[pos] != '=' || !detail::identifier(key))
            return detail::fail(error, "invalid reply key=value");
        ++pos;
        std::string value;
        if (!detail::word(line, pos, value, error)) return false;
        if (!parsed.values.emplace(key, value).second) return detail::fail(error, "duplicate reply key");
    }
    reply = parsed;
    return true;
}
} // namespace acquisition_protocol
