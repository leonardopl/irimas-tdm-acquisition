#pragma once

// Shared acquisition program line protocol. C++11, header-only; no camera, GUI or platform dependencies.
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace acquisition_protocol {

constexpr int session_protocol_version = 1;
constexpr char session_protocol_marker[] = "TDM_SESSION_PROTOCOL=1";

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
