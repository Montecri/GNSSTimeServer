"""Version-checked, project-local patches for the pinned Arduino HTTP parsers.

The installed framework packages are read only. A generated library overlay is
selected ahead of framework libraries by PlatformIO. Unexpected upstream changes
stop the build instead of silently dropping any hardening.
"""
from hashlib import sha256
from pathlib import Path
import re

PINNED = {
    "esp32": {
        "library": "WebServer", "parser": "Parsing.cpp", "header": "WebServer.h",
        "hashes": ("122de5397729899ac8600d545f7ed4b8a02298351a4f1b0fa5c7fa73f87a14d0",
                   "7770ef659d332dee92947cf7d98097874c39583a9113fd66735ca329459c8e8b"),
    },
    "esp8266": {
        "library": "ESP8266WebServer", "parser": "Parsing-impl.h", "header": "ESP8266WebServer.h",
        "hashes": ("f8fe756b04222f20c49813ea63f0c8a90a99034255324de84349e32755afe818",
                   "167b098fa4f964fc92ae9b89d531a69e3984561d4ca0729ab857f8f36d52dd67"),
    },
}


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError("HTTP patch expected one match: " + old[:100])
    return text.replace(old, new, 1)


def replace_once_re(text, pattern, new):
    patched, count = re.subn(pattern, new, text, flags=re.MULTILINE)
    if count != 1:
        raise RuntimeError("HTTP patch expected one match: " + pattern[:100])
    return patched


def function_span(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    # Target functions contain no braces in string literals/comments.
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return start, opening, end


def replace_body(text, signature, body):
    start, opening, end = function_span(text, signature)
    return text[:opening + 1] + "\n" + body + "\n}" + text[end:]


def patch_parser(source, arch):
    is32 = arch == "esp32"
    cls = "WebServer" if is32 else "ESP8266WebServerTemplate<ServerType>"
    failure = "false" if is32 else "CLIENT_MUST_STOP"
    source = replace_body(source, cls + "::_uploadReadByte(",
                          "  return _httpGuard.readByte(client);")
    # Abort safely for ordinary fields or malformed framing before a file starts.
    source = replace_body(source, cls + "::_parseFormUploadAborted()", """  if (_currentUpload && (_currentUpload->status == UPLOAD_FILE_START ||
                         _currentUpload->status == UPLOAD_FILE_WRITE)) {
    _currentUpload->status = UPLOAD_FILE_ABORTED;
    if (_currentHandler && _currentHandler->canUpload(_currentUri))
      _currentHandler->upload(*this, _currentUri, *_currentUpload);
  }
  return false;""")

    # Use the same bounded reader for headers, text fields and binary upload bytes.
    request_start, _, request_end = function_span(source, cls + "::_parseRequest(")
    request = source[request_start:request_end]
    request = replace_once(request, "  // Read the first line of HTTP request", """  _httpGuard.beginHeaders();
  _currentUpload.reset();
  bool sawContentLength = false;
  uint32_t parsedContentLength = 0;
  // Read the first line of HTTP request""")
    request = replace_once(request, "String req = client.readStringUntil('\\r');", "String req;\n  if (!_httpGuard.readLine(client, req)) return " + failure + ";")
    # Drop the LF read that paired with the request line. Anchor it to its own two-space indented
    # line: the header loops below issue the same call at a deeper indent. ESP8266 also puts a
    # debug statement between the two reads, so they cannot be matched as a single block.
    request = replace_once_re(request, r"^  client\.readStringUntil\('\\n'\);\n", "")
    request = re.sub(r"req = client\.readStringUntil\('\\r'\);\s*client\.readStringUntil\('\\n'\);",
                     "if (!_httpGuard.readLine(client, req)) return " + failure + ";", request)
    content_assignment = "_clientContentLength = headerValue.toInt();" if is32 else "contentLength = headerValue.toInt();"
    request = replace_once(request, content_assignment,
        "if (sawContentLength || !GnssHttpGuard::contentLength(headerValue, parsedContentLength)) return " + failure + ";\n"
        "        sawContentLength = true;\n        " + ("_clientContentLength" if is32 else "contentLength") + " = parsedContentLength;")
    # Reject chunked/ambiguous framing on all body-bearing requests.
    needle = '      if (headerName.equalsIgnoreCase(FPSTR(Content_Type)))'
    request = replace_once(request, needle,
        '      if (headerName.equalsIgnoreCase(F("Transfer-Encoding"))) return ' + failure + ';\n' + needle)
    needle = "    if (!isForm && _currentHandler" if is32 else "    String plainBuf;"
    gate = ("    if (_beforeBody && !_beforeBody()) return " + failure + ";\n"
            "    if (!_httpGuard.beginBody(parsedContentLength, isForm)) return " + failure + ";\n"
            "    if (isForm && (!sawContentLength || !parsedContentLength ||\n"
            "                   boundaryStr.length() == 0 || boundaryStr.length() > 70)) return " + failure + ";\n\n")
    request = replace_once(request, needle, gate + needle)
    if is32:
        request = replace_once(request, "plainLength, HTTP_MAX_POST_WAIT)", "plainLength, _httpGuard)")
        request = replace_once(request, "client.readBytes(_currentRaw->buf, HTTP_RAW_BUFLEN)",
            "_httpGuard.readBuffer(client, _currentRaw->buf,\n"
            "            std::min(size_t(HTTP_RAW_BUFLEN), size_t(_httpGuard.remaining())))")
    else:
        request = replace_once(request, "plainBuf, HTTP_MAX_POST_WAIT)", "plainBuf, _httpGuard)")
    if "readStringUntil" in request:
        raise RuntimeError("Unbounded request read remains")
    source = source[:request_start] + request + source[request_end:]

    if is32:
        start, _, end = function_span(source, "static char* readBytesWithTimeout(")
        source = source[:start] + """static char* readBytesWithTimeout(WiFiClient& client, size_t maxLength, size_t& dataLength, GnssHttpGuard& guard) {
  dataLength = 0;
  if (maxLength > GnssHttpGuard::MaxPlainBytes) return nullptr;
  char* result = static_cast<char*>(malloc(maxLength + 1));
  if (!result) return nullptr;
  dataLength = guard.readBuffer(client, reinterpret_cast<uint8_t*>(result), maxLength);
  result[dataLength] = 0;
  return result;
}""" + source[end:]
        # malloc failure at nonzero length is rejected by the caller's length test.
    else:
        source = replace_once(source, "String& data, int timeout_ms)", "String& data, GnssHttpGuard& guard)")
        source = replace_body(source, "bool readBytesWithTimeout(",
                              "  return guard.readText(client, maxLength, data);")

    start, _, end = function_span(source, cls + "::_parseForm(")
    form = source[start:end]
    form = replace_once(form, "  (void) len;", "  (void) len;\n  size_t fieldBytes = 0;")
    retry_start = form.index("  int retry = 0;")
    retry_end = form.index("  //start reading the form", retry_start)
    form = form[:retry_start] + "  if (!_httpGuard.readLine(client, line)) return false;\n" + form[retry_end:]
    form = re.sub(r"line = client\.readStringUntil\('\\r'\);\s*client\.readStringUntil\('\\n'\);",
                  "if (!_httpGuard.readLine(client, line)) return _parseFormUploadAborted();", form)
    form = re.sub(r"client\.readStringUntil\('\\r'\);\s*client\.readStringUntil\('\\n'\);",
                  "if (!_httpGuard.readLine(client, line)) return _parseFormUploadAborted();", form)
    form = replace_once(form, '              if (line.startsWith("--"+boundary)) break;',
        '              if (line == ("--"+boundary) || line == ("--"+boundary+"--")) break;\n'
        '              if (argValue.length() + line.length() + 1 > GnssHttpGuard::MaxFieldBytes) return false;\n'
        '              fieldBytes += line.length() + 1;\n'
        '              if (fieldBytes > GnssHttpGuard::MaxFieldsBytes) return false;')
    form = replace_once(form, "            RequestArgument& arg = _postArgs[_postArgsLen++];",
        "            if (_postArgsLen >= WEBSERVER_MAX_POST_ARGS) return false;\n"
        "            RequestArgument& arg = _postArgs[_postArgsLen++];")
    if is32:
        form = replace_once(form, """            } else if (_postArgsLen >= WEBSERVER_MAX_POST_ARGS) {
              log_e("Too many PostArgs (max: %d) in request.", WEBSERVER_MAX_POST_ARGS);
              return false;
""", "")
    # Validate the complete file framing BEFORE END can commit flash. Firmware
    # uploads must be the last part; the application's browser form does this.
    needle = "            // Found the boundary string, finish processing this file upload"
    form = replace_once(form, needle,
        '            if (!_httpGuard.readLine(client, line) || line != "--" || _httpGuard.remaining() != 0)\n'
        '                return _parseFormUploadAborted();\n' + needle)
    tail_start = form.index("            if (!client.connected()) return _parseFormUploadAborted();")
    tail_end = form.index("            continue;", tail_start) + len("            continue;")
    form = form[:tail_start] + "            break;" + form[tail_end:]
    form = replace_once(form, "    int iarg;", "    if (_httpGuard.remaining() != 0) return false;\n    int iarg;")
    if "readStringUntil" in form:
        raise RuntimeError("Unbounded multipart read remains")
    source = source[:start] + form + source[end:]
    return source


def patch_header(source):
    source = replace_once(source, "#include <memory>",
                          '#include <memory>\n#include "GnssHttpGuard.h"\n#define GNSS_HTTP_HARDENED 1')
    source = replace_once(source, "  typedef std::function<void(void)> THandlerFunction;",
        "  // Called after bounded headers and before any request body is consumed.\n"
        "  void onBeforeBody(std::function<bool(void)> fn) { _beforeBody = fn; }\n"
        "  typedef std::function<void(void)> THandlerFunction;")
    source = replace_once(source, "protected:",
        "protected:\n  GnssHttpGuard _httpGuard;\n  std::function<bool(void)> _beforeBody;")
    return source


def generate(framework, output, arch):
    spec = PINNED[arch]
    library = Path(framework) / "libraries" / spec["library"]
    for name, digest in zip((spec["parser"], spec["header"]), spec["hashes"]):
        raw = (library / "src" / name).read_bytes()
        # Pin normalized LF content so Git/package line endings are immaterial.
        if sha256(raw.replace(b"\r\n", b"\n")).hexdigest() != digest:
            raise RuntimeError("Review HTTP hardening for changed framework file: " + name)
    destination = Path(output) / spec["library"]
    for path in library.rglob("*"):
        if not path.is_file() or "examples" in path.parts:
            continue
        content = path.read_bytes()
        if path.name == spec["parser"]:
            content = patch_parser(content.decode().replace("\r\n", "\n"), arch).encode()
        elif path.name == spec["header"]:
            content = patch_header(content.decode().replace("\r\n", "\n")).encode()
        target = destination / path.relative_to(library)
        target.parent.mkdir(parents=True, exist_ok=True)
        if not target.exists() or target.read_bytes() != content:
            target.write_bytes(content)
    guard = Path(__file__).with_name("GnssHttpGuard.h").read_bytes()
    target = destination / "src" / "GnssHttpGuard.h"
    if not target.exists() or target.read_bytes() != guard:
        target.write_bytes(guard)
    return destination
