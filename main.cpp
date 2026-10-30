// PicoTester - Win32 GUI for Raspberry Pi Pico boards (RP2040).
//  * finds every connected Pico (USB VID 2E8A) and its COM port
//  * Start/Stop: streams CRC-protected frames to ALL connected Picos at the same time (echoed back),
//    optionally with the CPU/RAM stress test running on core 1 of each Pico
//  * Flash test: RAM test + flash surface test + flash size report (one Pico connected only)
//  * Upload UF2: reboots each Pico into BOOTSEL and copies the .uf2 found next to this exe
//
// Build: Visual Studio 2019/2022, C++14 or newer, Unicode character set.

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <initguid.h>
#include <devguid.h>
#include <setupapi.h>
#include <commctrl.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cwctype>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "comctl32.lib")
#pragma comment(linker, "/manifestdependency:\"type='win32' name='Microsoft.Windows.Common-Controls' version='6.0.0.0' processorArchitecture='*' publicKeyToken='6595b64144ccf1df' language='*'\"")

// ---------------------------------------------------------------- constants
enum { ID_START = 101, ID_STOP, ID_FLASH, ID_UF2, ID_CPU, ID_LOG, ID_PORT, ID_STATS };
enum { WM_APP_LOG = WM_APP + 1, WM_APP_BUSYDONE = WM_APP + 2 };
constexpr UINT_PTR TIMER_STATS = 1;
constexpr UINT_PTR TIMER_RESCAN = 2;
constexpr size_t FRAME = 512;            // seq(4) + payload(504) + crc32(4)
constexpr size_t BATCH = 16;             // frames per WriteFile call (8 KB)
constexpr uint32_t SEQ_MASK = 0x7FFFFFFFu;
constexpr uint32_t STATUS_FLAG = 0x80000000u;

static std::wstring wfmt(const wchar_t* f, ...) {
    wchar_t b[1024];
    va_list a;
    va_start(a, f);
    vswprintf(b, 1024, f, a);
    va_end(a);
    b[1023] = 0;
    return b;
}

// ------------------------------------------------------------------- CRC32
static uint32_t g_crcTab[256];
static void initCrc() {
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        g_crcTab[i] = c;
    }
}
static uint32_t crc32(const uint8_t* p, size_t n) {
    uint32_t c = 0xFFFFFFFFu;
    while (n--) c = g_crcTab[(c ^ *p++) & 0xFF] ^ (c >> 8);
    return ~c;
}
static void buildFrame(uint8_t* f, uint32_t seq) {
    memcpy(f, &seq, 4);
    uint32_t s = seq * 2654435761u + 1u;
    for (size_t i = 4; i < FRAME - 4; ++i) {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        f[i] = uint8_t(s >> 8);
    }
    uint32_t c = crc32(f, FRAME - 4);
    memcpy(f + FRAME - 4, &c, 4);
}

// ------------------------------------------------------------- port search
static bool portLess(const std::wstring& a, const std::wstring& b) {
    return a.size() != b.size() ? a.size() < b.size() : a < b;     // COM3 < COM10
}

static std::vector<std::wstring> FindPicoPorts() {
    std::vector<std::wstring> found;
    HDEVINFO set = SetupDiGetClassDevsW(&GUID_DEVCLASS_PORTS, nullptr, nullptr, DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return found;
    SP_DEVINFO_DATA d;
    ZeroMemory(&d, sizeof(d));
    d.cbSize = sizeof(d);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &d); ++i) {
        wchar_t hw[1024] = {};
        if (!SetupDiGetDeviceRegistryPropertyW(set, &d, SPDRP_HARDWAREID, nullptr, (PBYTE)hw, sizeof(hw) - 4, nullptr))
            continue;
        bool pico = false;                                 // REG_MULTI_SZ
        for (const wchar_t* p = hw; *p; p += wcslen(p) + 1) {
            std::wstring s = p;
            for (auto& c : s) c = (wchar_t)towupper(c);
            if (s.find(L"VID_2E8A") != std::wstring::npos) { pico = true; break; }
        }
        if (!pico) continue;
        HKEY k = SetupDiOpenDevRegKey(set, &d, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if ((HANDLE)k == INVALID_HANDLE_VALUE) continue;
        wchar_t name[64] = {};
        DWORD sz = sizeof(name) - 2, type = 0;
        if (RegQueryValueExW(k, L"PortName", nullptr, &type, (LPBYTE)name, &sz) == ERROR_SUCCESS &&
            wcsncmp(name, L"COM", 3) == 0 &&
            std::find(found.begin(), found.end(), std::wstring(name)) == found.end())
            found.push_back(name);
        RegCloseKey(k);
    }
    SetupDiDestroyDeviceInfoList(set);
    std::sort(found.begin(), found.end(), portLess);
    return found;
}

// Pico in BOOTSEL mode = USB drive labelled RPI-RP2 (RP2040) or RP2350
static std::vector<std::wstring> FindBootDrives() {
    std::vector<std::wstring> out;
    DWORD mask = GetLogicalDrives();
    for (int i = 0; i < 26; ++i) {
        if (!(mask & (1u << i))) continue;
        wchar_t root[4] = {(wchar_t)(L'A' + i), L':', L'\\', 0};
        UINT t = GetDriveTypeW(root);
        if (t != DRIVE_REMOVABLE && t != DRIVE_FIXED) continue;
        wchar_t label[64] = {};
        if (GetVolumeInformationW(root, label, 64, nullptr, nullptr, nullptr, nullptr, 0) &&
            (_wcsicmp(label, L"RPI-RP2") == 0 || _wcsicmp(label, L"RP2350") == 0))
            out.push_back(root);
    }
    return out;
}

// ------------------------------------------------------- overlapped serial
struct Io {
    OVERLAPPED ov;
    Io() { ZeroMemory(&ov, sizeof(ov)); ov.hEvent = CreateEventW(nullptr, TRUE, FALSE, nullptr); }
    ~Io() { CloseHandle(ov.hEvent); }
    Io(const Io&) = delete;
    Io& operator=(const Io&) = delete;
};

// true = completed (done may be 0 after a read timeout); false = error or cancelled
static bool xfer(HANDLE h, bool wr, void* buf, DWORD len, DWORD& done, Io& io, const std::atomic<bool>& run) {
    done = 0;
    ResetEvent(io.ov.hEvent);
    BOOL ok = wr ? WriteFile(h, buf, len, nullptr, &io.ov) : ReadFile(h, buf, len, nullptr, &io.ov);
    if (!ok) {
        if (GetLastError() != ERROR_IO_PENDING) return false;
        while (WaitForSingleObject(io.ov.hEvent, 100) == WAIT_TIMEOUT) {
            if (!run) {
                CancelIoEx(h, &io.ov);
                GetOverlappedResult(h, &io.ov, &done, TRUE);
                return false;
            }
        }
    }
    return GetOverlappedResult(h, &io.ov, &done, FALSE) != 0;
}

static bool writeAll(HANDLE h, const void* data, DWORD len, Io& io, const std::atomic<bool>& run) {
    const uint8_t* p = (const uint8_t*)data;
    while (len) {
        DWORD n = 0;
        if (!xfer(h, true, (void*)p, len, n, io, run)) return false;
        p += n; len -= n;
    }
    return true;
}

struct Link {
    HANDLE h = INVALID_HANDLE_VALUE;
    bool open(const std::wstring& port, DWORD baud = 921600) {
        std::wstring path = L"\\\\.\\" + port;
        h = CreateFileW(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, nullptr);
        if (h == INVALID_HANDLE_VALUE) return false;
        SetupComm(h, 1 << 18, 1 << 18);
        DCB dcb;
        ZeroMemory(&dcb, sizeof(dcb));
        dcb.DCBlength = sizeof(dcb);
        GetCommState(h, &dcb);
        dcb.BaudRate = baud;                      // ignored by USB CDC, except 1200 = "reset to bootloader"
        dcb.ByteSize = 8; dcb.Parity = NOPARITY; dcb.StopBits = ONESTOPBIT;
        dcb.fBinary = TRUE;
        dcb.fDtrControl = DTR_CONTROL_ENABLE;     // the Pico firmware uses DTR to detect "host connected"
        dcb.fRtsControl = RTS_CONTROL_ENABLE;
        dcb.fOutX = dcb.fInX = FALSE;
        dcb.fOutxCtsFlow = dcb.fOutxDsrFlow = FALSE;
        dcb.fAbortOnError = FALSE;
        SetCommState(h, &dcb);
        COMMTIMEOUTS to;
        ZeroMemory(&to, sizeof(to));              // read: return what is there, else wait max 50 ms
        to.ReadIntervalTimeout = MAXDWORD;
        to.ReadTotalTimeoutMultiplier = MAXDWORD;
        to.ReadTotalTimeoutConstant = 50;
        SetCommTimeouts(h, &to);
        PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);
        return true;
    }
    void close() {
        if (h != INVALID_HANDLE_VALUE) { CloseHandle(h); h = INVALID_HANDLE_VALUE; }
    }
    ~Link() { close(); }
};

// ----------------------------------------------------------------- globals
static HWND g_hwnd, g_btnStart, g_btnStop, g_btnFlash, g_btnUf2, g_chkCpu, g_log, g_lblPort, g_lblStats;
static std::vector<std::wstring> g_ports;
static bool g_streaming = false, g_busy = false;
static std::thread g_work;
static std::atomic<bool> g_workRun{false};
static std::wstring g_lastStats;

// One connected Pico while streaming
struct Device {
    std::wstring port;
    Link link;
    std::thread tx, rx;
    std::atomic<bool> run{false};
    std::atomic<bool> failed{false};
    bool cpuOn = false;
    DWORD startTick = 0;
    std::atomic<uint64_t> txBytes{0}, rxBytes{0}, ok{0}, bad{0}, lost{0}, statusCount{0};
    std::atomic<int64_t> pass{-1}, eInt{0}, eCrc{0}, eFlt{0}, eRam{0}, picoBad{0}, temp10{-99999};
    // UI-thread-only bookkeeping
    uint64_t lastTx = 0, lastRx = 0, lastErrs = 0, lastStatusSeen = 0;
    int64_t lastPassSeen = -1;
    DWORD lastErrLog = 0;
    bool warnedSilent = false, warnedStall = false;
    double txRate = 0, rxRate = 0;
    int64_t cpuErrs() const { return eInt + eCrc + eFlt + eRam; }
};
static std::vector<std::unique_ptr<Device>> g_devs;

// -------------------------------------------------------------- UI helpers
static void logLine(const std::wstring& s) {
    std::wstring t = s + L"\r\n";
    int len = GetWindowTextLengthW(g_log);
    SendMessageW(g_log, EM_SETSEL, len, len);
    SendMessageW(g_log, EM_REPLACESEL, FALSE, (LPARAM)t.c_str());
    SendMessageW(g_log, EM_SCROLLCARET, 0, 0);
}
static void postLog(const std::wstring& s) {            // from any thread
    PostMessageW(g_hwnd, WM_APP_LOG, 0, (LPARAM) new std::wstring(s));
}
static std::wstring widen(const std::string& s) {
    return std::wstring(s.begin(), s.end());           // firmware only sends ASCII
}
static void refreshUi() {
    bool idle = !g_streaming && !g_busy;
    EnableWindow(g_btnStart, idle);
    EnableWindow(g_btnStop, g_streaming && !g_busy);
    EnableWindow(g_btnFlash, idle && g_ports.size() <= 1);
    EnableWindow(g_btnUf2, idle);
    EnableWindow(g_chkCpu, idle);
}
static void scanPorts() {
    g_ports = FindPicoPorts();
    std::wstring t;
    if (g_ports.empty()) t = L"No Pico found (plug it in; a board in BOOTSEL mode shows up as a drive, not a COM port)";
    else if (g_ports.size() == 1) t = L"Pico found on " + g_ports[0];
    else {
        t = wfmt(L"%u Picos found: ", (unsigned)g_ports.size());
        for (size_t i = 0; i < g_ports.size(); ++i) t += (i ? L", " : L"") + g_ports[i];
    }
    std::vector<std::wstring> drives = FindBootDrives();
    if (!drives.empty()) {
        t += L"   |   BOOTSEL drive: ";
        for (size_t i = 0; i < drives.size(); ++i) t += (i ? L", " : L"") + drives[i];
    }
    SetWindowTextW(g_lblPort, t.c_str());
    refreshUi();
}

// ------------------------------------------------------------ status parse
static bool kv(const std::string& s, const char* key, int64_t& out) {
    std::string k = std::string(key) + "=";
    size_t p = 0;
    while ((p = s.find(k, p)) != std::string::npos) {
        if (p == 0 || s[p - 1] == ' ') {
            out = _strtoi64(s.c_str() + p + k.size(), nullptr, 10);
            return true;
        }
        p += k.size();
    }
    return false;
}

static void handleStatus(Device* d, const uint8_t* payload) {
    char txt[FRAME];
    memcpy(txt, payload, FRAME - 8);
    txt[FRAME - 8] = 0;
    std::string s(txt);
    int64_t v;
    if (kv(s, "pass", v)) d->pass = v;
    if (kv(s, "eInt", v)) d->eInt = v;
    if (kv(s, "eCrc", v)) d->eCrc = v;
    if (kv(s, "eFlt", v)) d->eFlt = v;
    if (kv(s, "eRam", v)) d->eRam = v;
    if (kv(s, "rxbad", v)) d->picoBad = v;
    size_t p = s.find("temp=");
    if (p != std::string::npos) {
        double t = atof(s.c_str() + p + 5);
        d->temp10 = (int64_t)(t * 10.0 + (t < 0 ? -0.5 : 0.5));
    }
    d->statusCount++;
}

// ---------------------------------------------------------------- streaming
static void deviceFailed(Device* d, const wchar_t* what) {
    if (d->run.exchange(false)) {
        d->failed = true;
        postLog(d->port + L": " + what + L" (device unplugged or port closed?). Press Stop to finish.");
    }
}

static void txThread(Device* d) {
    Io io;
    std::vector<uint8_t> buf(FRAME * BATCH);
    uint32_t seq = 0;
    while (d->run) {
        for (size_t i = 0; i < BATCH; ++i) buildFrame(buf.data() + i * FRAME, (seq++) & SEQ_MASK);
        if (!writeAll(d->link.h, buf.data(), (DWORD)buf.size(), io, d->run)) {
            if (d->run) deviceFailed(d, L"write failed");
            break;
        }
        d->txBytes += buf.size();
    }
}

static void rxThread(Device* d) {
    Io io;
    std::vector<uint8_t> buf(1 << 17);
    size_t fill = 0;
    uint32_t expect = 0;
    bool synced = false, inBad = false;
    while (d->run) {
        DWORD n = 0;
        if (!xfer(d->link.h, false, buf.data() + fill, (DWORD)(buf.size() - fill), n, io, d->run)) {
            if (d->run) deviceFailed(d, L"read failed");
            break;
        }
        if (n == 0) continue;
        d->rxBytes += n;
        fill += n;
        size_t pos = 0;
        while (fill - pos >= FRAME) {
            const uint8_t* f = buf.data() + pos;
            uint32_t c;
            memcpy(&c, f + FRAME - 4, 4);
            if (crc32(f, FRAME - 4) == c) {
                uint32_t seq;
                memcpy(&seq, f, 4);
                if (seq & STATUS_FLAG) {
                    handleStatus(d, f + 4);
                } else {
                    if (synced) {
                        uint32_t gap = (seq - expect) & SEQ_MASK;
                        if (gap != 0) {
                            if (gap < 0x40000000u) d->lost += gap;      // frames missing
                            else ++d->bad;                              // duplicate / out of order
                        }
                    }
                    expect = (seq + 1) & SEQ_MASK;
                    synced = true;
                    ++d->ok;
                }
                inBad = false;
                pos += FRAME;
            } else {                    // slide one byte to re-sync, count the bad stretch once
                if (!inBad) { ++d->bad; inBad = true; }
                ++pos;
            }
        }
        memmove(buf.data(), buf.data() + pos, fill - pos);
        fill -= pos;
    }
}

static std::wstring elapsed(DWORD since) {
    DWORD s = (GetTickCount() - since) / 1000;
    return wfmt(L"%02u:%02u:%02u", s / 3600, (s / 60) % 60, s % 60);
}

static bool deviceVerdict(const Device& d) {
    return d.rxBytes > 0 && d.bad == 0 && d.lost == 0 && d.picoBad == 0 && d.cpuErrs() == 0 && !d.failed &&
           (!d.cpuOn || d.pass > 0);
}

static void stopStreaming() {
    if (!g_streaming) return;
    for (auto& d : g_devs) d->run = false;
    for (auto& d : g_devs) {
        if (d->tx.joinable()) d->tx.join();
        if (d->rx.joinable()) d->rx.join();
    }
    for (auto& up : g_devs) {
        Device& d = *up;
        d.link.close();                       // closing drops DTR -> the Pico returns to command mode
        logLine(wfmt(L"%ls summary: ran %ls, TX %.1f MB, RX %.1f MB, frames ok %llu / bad %llu / lost %llu, "
                     L"Pico saw %lld bad frames, core1 passes %lld, CPU/RAM errors %lld  ->  %ls",
                     d.port.c_str(), elapsed(d.startTick).c_str(), d.txBytes.load() / 1e6, d.rxBytes.load() / 1e6,
                     (unsigned long long)d.ok.load(), (unsigned long long)d.bad.load(), (unsigned long long)d.lost.load(),
                     (long long)d.picoBad.load(), (long long)std::max<int64_t>(d.pass.load(), 0),
                     (long long)d.cpuErrs(), deviceVerdict(d) ? L"PASS" : L"FAIL"));
    }
    g_devs.clear();
    g_streaming = false;
    g_lastStats.clear();
    SetWindowTextW(g_lblStats, L"");
    logLine(L"Stopped.");
    refreshUi();
}

static void startStreaming() {
    if (g_streaming || g_busy) return;
    scanPorts();
    if (g_ports.empty()) { logLine(L"No Pico COM port found."); return; }
    bool cpu = SendMessageW(g_chkCpu, BM_GETCHECK, 0, 0) == BST_CHECKED;
    for (const auto& port : g_ports) {
        std::unique_ptr<Device> d(new Device());
        d->port = port;
        d->cpuOn = cpu;
        if (!d->link.open(port)) { logLine(L"Cannot open " + port + L" (in use by another program?)."); continue; }
        Io io;
        std::atomic<bool> yes{true};
        const char* cmd = cpu ? "STREAM CPU\n" : "STREAM\n";
        if (!writeAll(d->link.h, cmd, (DWORD)strlen(cmd), io, yes)) {
            logLine(L"Failed to send the STREAM command to " + port);
            continue;
        }
        g_devs.push_back(std::move(d));
    }
    if (g_devs.empty()) return;
    for (auto& up : g_devs) {
        Device* d = up.get();
        d->startTick = GetTickCount();
        d->run = true;
        d->rx = std::thread(rxThread, d);
        d->tx = std::thread(txThread, d);
    }
    g_streaming = true;
    refreshUi();
    std::wstring l = wfmt(L"Streaming%ls on %u Pico(s):", cpu ? L" + core-1 CPU/RAM test" : L"", (unsigned)g_devs.size());
    for (auto& d : g_devs) l += L" " + d->port;
    logLine(l);
}

static void updateStats() {
    static DWORD lastT = GetTickCount();
    DWORD now = GetTickCount();
    double dt = (now - lastT) / 1000.0;
    if (dt < 0.2) return;
    lastT = now;
    std::wstring text;
    double sumTx = 0, sumRx = 0;
    for (auto& up : g_devs) {
        Device& d = *up;
        uint64_t tx = d.txBytes, rx = d.rxBytes;
        d.txRate = (tx - d.lastTx) / dt / 1e6;
        d.rxRate = (rx - d.lastRx) / dt / 1e6;
        d.lastTx = tx; d.lastRx = rx;
        if (!d.run) d.txRate = d.rxRate = 0;
        sumTx += d.txRate; sumRx += d.rxRate;

        uint64_t errs = d.bad + d.lost + (uint64_t)d.picoBad.load() + (uint64_t)d.cpuErrs();
        if (errs > d.lastErrs && now - d.lastErrLog > 5000) {
            logLine(wfmt(L"%ls ERROR counters: bad %llu, lost %llu, Pico-side bad %lld, CPU/RAM err int/crc/float/ram %lld/%lld/%lld/%lld",
                         d.port.c_str(), (unsigned long long)d.bad.load(), (unsigned long long)d.lost.load(),
                         (long long)d.picoBad.load(), (long long)d.eInt.load(), (long long)d.eCrc.load(),
                         (long long)d.eFlt.load(), (long long)d.eRam.load()));
            d.lastErrs = errs;
            d.lastErrLog = now;
        }
        if (d.run && d.rxBytes == 0 && now - d.startTick > 3000 && !d.warnedSilent) {
            d.warnedSilent = true;
            logLine(d.port + L": no data received from the Pico. Is the tester firmware (pico_tester.ino) uploaded?");
        }
        uint64_t sc = d.statusCount;
        if (d.cpuOn && d.run && sc != d.lastStatusSeen) {
            if (sc > 1 && d.pass == d.lastPassSeen && !d.warnedStall) {
                d.warnedStall = true;
                logLine(d.port + L": core 1 did not make progress - the CPU test stalled!");
            }
            d.lastPassSeen = d.pass;
            d.lastStatusSeen = sc;
        }

        std::wstring core1 = d.cpuOn ? wfmt(L"core1 pass %lld, err %lld/%lld/%lld/%lld", (long long)d.pass.load(),
                                            (long long)d.eInt.load(), (long long)d.eCrc.load(), (long long)d.eFlt.load(),
                                            (long long)d.eRam.load())
                                     : std::wstring(L"core1 off");
        int64_t t10 = d.temp10;
        std::wstring temp = t10 > -9000 ? wfmt(L"%lld.%lld C", (long long)(t10 / 10), (long long)std::llabs(t10 % 10)) : L"-- C";
        text += wfmt(L"%ls %ls  %ls | TX %.2f RX %.2f MB/s | frames ok %llu bad %llu lost %llu | %ls | Pico rx bad %lld | %ls\r\n",
                     d.port.c_str(), d.failed ? L"FAILED" : (d.run ? L"running" : L"stopped"), elapsed(d.startTick).c_str(),
                     d.txRate, d.rxRate, (unsigned long long)d.ok.load(), (unsigned long long)d.bad.load(),
                     (unsigned long long)d.lost.load(), core1.c_str(), (long long)d.picoBad.load(), temp.c_str());
    }
    if (g_devs.size() > 1) text += wfmt(L"TOTAL  TX %.2f RX %.2f MB/s", sumTx, sumRx);
    if (text != g_lastStats) {
        g_lastStats = text;
        SetWindowTextW(g_lblStats, text.c_str());
    }
}

// ------------------------------------------------------- worker: flash test
static void flashThread(std::wstring port) {
    Link l;
    if (!l.open(port)) { postLog(L"Cannot open " + port); PostMessageW(g_hwnd, WM_APP_BUSYDONE, 0, 0); return; }
    Io io;
    std::atomic<bool> yes{true};
    const char* cmd = "FLASH\n";
    if (!writeAll(l.h, cmd, (DWORD)strlen(cmd), io, yes)) {
        postLog(L"Failed to send the FLASH command.");
        PostMessageW(g_hwnd, WM_APP_BUSYDONE, 0, 0);
        return;
    }
    std::string line;
    char buf[1024];
    DWORD lastData = GetTickCount();
    bool done = false;
    while (!done && g_workRun) {
        DWORD n = 0;
        if (!xfer(l.h, false, buf, sizeof(buf), n, io, g_workRun)) break;
        if (n == 0) {
            if (GetTickCount() - lastData > 180000) {
                postLog(L"Timeout: no data from the Pico for 180 s (is the tester firmware uploaded?).");
                break;
            }
            continue;
        }
        lastData = GetTickCount();
        for (DWORD i = 0; i < n; ++i) {
            char c = buf[i];
            if (c == '\n') {
                while (!line.empty() && line.back() == '\r') line.pop_back();
                if (!line.empty()) {
                    postLog(widen(line));
                    if (line == "DONE") done = true;
                }
                line.clear();
            } else if (line.size() < 400) {
                line.push_back(c);
            }
        }
    }
    PostMessageW(g_hwnd, WM_APP_BUSYDONE, 0, 0);
}

static void startFlashTest() {
    if (g_streaming || g_busy) return;
    scanPorts();
    if (g_ports.size() != 1) {
        logLine(g_ports.empty() ? L"No Pico COM port found."
                                : L"The flash test needs exactly ONE Pico connected (unplug the others).");
        return;
    }
    logLine(L"--- RAM + flash surface test on " + g_ports[0] + L" (wipes the flash after the firmware image) ---");
    g_busy = true;
    g_workRun = true;
    refreshUi();
    g_work = std::thread(flashThread, g_ports[0]);
}

// ------------------------------------------------------- worker: upload UF2
static std::wstring exeDir() {
    wchar_t p[MAX_PATH] = {};
    GetModuleFileNameW(nullptr, p, MAX_PATH);
    wchar_t* s = wcsrchr(p, L'\\');
    if (s) *(s + 1) = 0;
    return p;
}

static std::wstring findUf2(const std::wstring& dir) {          // newest *.uf2 next to the exe
    WIN32_FIND_DATAW fd, best;
    ZeroMemory(&best, sizeof(best));
    HANDLE h = FindFirstFileW((dir + L"*.uf2").c_str(), &fd);
    if (h == INVALID_HANDLE_VALUE) return L"";
    bool have = false;
    do {
        if (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;
        if (!have || CompareFileTime(&fd.ftLastWriteTime, &best.ftLastWriteTime) > 0) { best = fd; have = true; }
    } while (FindNextFileW(h, &fd));
    FindClose(h);
    return have ? dir + best.cFileName : L"";
}

static bool readFileAll(const std::wstring& path, std::vector<uint8_t>& data) {
    HANDLE h = CreateFileW(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    LARGE_INTEGER sz;
    bool ok = GetFileSizeEx(h, &sz) && sz.QuadPart > 0 && sz.QuadPart < (64ll << 20);
    if (ok) {
        data.resize((size_t)sz.QuadPart);
        size_t off = 0;
        while (ok && off < data.size()) {
            DWORD got = 0;
            DWORD want = (DWORD)std::min<size_t>(data.size() - off, 1u << 20);
            ok = ReadFile(h, data.data() + off, want, &got, nullptr) && got > 0;
            off += got;
        }
    }
    CloseHandle(h);
    return ok;
}

static uint32_t le32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

static bool checkUf2(const std::vector<uint8_t>& d, std::wstring& info) {
    if (d.size() < 512 || d.size() % 512) { info = L"file size is not a multiple of 512 bytes"; return false; }
    for (size_t b = 0; b < d.size(); b += 512) {
        if (le32(&d[b]) != 0x0A324655u || le32(&d[b + 4]) != 0x9E5D5157u || le32(&d[b + 508]) != 0x0AB16F30u) {
            info = L"bad UF2 block magic (not a UF2 file?)";
            return false;
        }
    }
    uint32_t flags = le32(&d[8]), family = (flags & 0x2000) ? le32(&d[28]) : 0;
    info = wfmt(L"%u blocks, %u KB of data, family id 0x%08X%ls", (unsigned)(d.size() / 512), (unsigned)(d.size() / 512 * 256 / 1024),
                family, family == 0xE48BFF56u ? L" (RP2040)" : (family == 0xE48BFF59u || family == 0xE48BFF57u) ? L" (RP2350)" : L"");
    return true;
}

static bool sleepW(DWORD ms) {            // false if the app is closing
    for (DWORD t = 0; t < ms; t += 50) {
        if (!g_workRun) return false;
        Sleep(50);
    }
    return g_workRun;
}

static std::vector<std::wstring> waitBootDrives(DWORD ms) {
    for (DWORD t = 0; t <= ms; t += 100) {
        std::vector<std::wstring> d = FindBootDrives();
        if (!d.empty()) { sleepW(500); return FindBootDrives(); }     // let Windows finish mounting
        if (!sleepW(100)) break;
    }
    return std::vector<std::wstring>();
}

static bool writeUf2(const std::wstring& drive, const std::vector<uint8_t>& d) {
    HANDLE h = CreateFileW((drive + L"firmware.uf2").c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    size_t off = 0;
    bool ok = true;
    while (off < d.size()) {
        DWORD w = 0;
        DWORD chunk = (DWORD)std::min<size_t>(d.size() - off, 64 * 1024);
        if (!WriteFile(h, d.data() + off, chunk, &w, nullptr) || w == 0) { ok = false; break; }
        off += w;
    }
    CloseHandle(h);                       // the Pico reboots as soon as the last block lands; errors here are normal
    return ok;
}

static bool programDrive(const std::wstring& drive, const std::vector<uint8_t>& uf2, const std::wstring& tag) {
    postLog(tag + L": copying firmware to " + drive + L" ...");
    bool wrote = writeUf2(drive, uf2);
    for (int i = 0; i < 150 && g_workRun; ++i) {            // the drive disappears when the Pico reboots
        std::vector<std::wstring> d = FindBootDrives();
        if (std::find(d.begin(), d.end(), drive) == d.end()) {
            postLog(tag + L": done - the Pico rebooted into the new firmware" +
                    (wrote ? L"." : L" (Windows reported a copy error while the drive vanished; that is normal)."));
            return true;
        }
        Sleep(100);
    }
    postLog(tag + L": the drive is still there after 15 s - the Pico did not accept the file (wrong chip family or damaged UF2).");
    return false;
}

static void sendBoot(const std::wstring& port) {
    Link l;
    if (!l.open(port)) return;
    Io io;
    std::atomic<bool> yes{true};
    writeAll(l.h, "BOOT\n", 5, io, yes);
    Sleep(200);
    l.close();
}

static void touch1200(const std::wstring& port) {         // Arduino-Pico: 1200 baud open/close = reboot to BOOTSEL
    Link l;
    if (!l.open(port, 1200)) return;
    Sleep(400);
    l.close();
}

static void uploadThread() {
    std::wstring dir = exeDir();
    std::wstring path = findUf2(dir);
    if (path.empty()) {
        postLog(L"No .uf2 file found next to the exe. Put your compiled firmware (.uf2) in: " + dir);
        PostMessageW(g_hwnd, WM_APP_BUSYDONE, 0, 0);
        return;
    }
    std::vector<uint8_t> data;
    std::wstring info;
    if (!readFileAll(path, data)) { postLog(L"Cannot read " + path); PostMessageW(g_hwnd, WM_APP_BUSYDONE, 0, 0); return; }
    if (!checkUf2(data, info)) {
        postLog(L"Invalid UF2 file " + path + L": " + info);
        PostMessageW(g_hwnd, WM_APP_BUSYDONE, 0, 0);
        return;
    }
    postLog(L"Using " + path + L" (" + info + L")");

    std::vector<std::wstring> ports = FindPicoPorts();
    std::vector<std::wstring> drives = FindBootDrives();
    if (ports.empty() && drives.empty()) {
        postLog(L"No Pico found. Plug one in, or hold BOOTSEL while connecting it.");
        PostMessageW(g_hwnd, WM_APP_BUSYDONE, 0, 0);
        return;
    }
    int total = 0, good = 0;
    for (const auto& dr : drives) {                         // boards already in BOOTSEL
        if (!g_workRun) break;
        ++total;
        if (programDrive(dr, data, L"BOOTSEL drive " + dr)) ++good;
    }
    for (const auto& port : ports) {
        if (!g_workRun) break;
        ++total;
        postLog(port + L": asking the Pico to reboot into BOOTSEL ...");
        sendBoot(port);
        std::vector<std::wstring> d = waitBootDrives(4000);
        if (d.empty() && g_workRun) {
            postLog(port + L": no reaction, trying the 1200-baud reset ...");
            touch1200(port);
            d = waitBootDrives(8000);
        }
        if (d.empty()) {
            postLog(port + L": could not enter BOOTSEL. Hold the BOOTSEL button while plugging the board in, then press Upload UF2 again.");
            continue;
        }
        if (programDrive(d[0], data, port)) ++good;
    }
    postLog(wfmt(L"Upload finished: %d of %d board(s) programmed.", good, total));
    PostMessageW(g_hwnd, WM_APP_BUSYDONE, 1, 0);
}

static void startUpload() {
    if (g_streaming || g_busy) return;
    logLine(L"--- Upload UF2 ---");
    g_busy = true;
    g_workRun = true;
    refreshUi();
    g_work = std::thread(uploadThread);
}

// --------------------------------------------------------------- window proc
static void layout(int w, int h) {
    if (w < 100 || h < 100) return;                     // minimised
    MoveWindow(g_lblPort, 12, 12, w - 24, 20, TRUE);
    MoveWindow(g_lblStats, 12, 82, w - 24, 112, TRUE);
    MoveWindow(g_log, 12, 200, w - 24, std::max(60, h - 212), TRUE);
}

static LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
    switch (m) {
    case WM_CREATE: {
        g_hwnd = h;
        HFONT f = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
        g_lblPort  = CreateWindowW(L"STATIC", L"", WS_CHILD | WS_VISIBLE, 12, 12, 760, 20, h, (HMENU)ID_PORT, nullptr, nullptr);
        g_btnStart = CreateWindowW(L"BUTTON", L"Start", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 12, 40, 90, 32, h, (HMENU)ID_START, nullptr, nullptr);
        g_btnStop  = CreateWindowW(L"BUTTON", L"Stop", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_DISABLED, 108, 40, 90, 32, h, (HMENU)ID_STOP, nullptr, nullptr);
        g_btnFlash = CreateWindowW(L"BUTTON", L"Flash test", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 214, 40, 110, 32, h, (HMENU)ID_FLASH, nullptr, nullptr);
        g_btnUf2   = CreateWindowW(L"BUTTON", L"Upload UF2", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON, 330, 40, 110, 32, h, (HMENU)ID_UF2, nullptr, nullptr);
        g_chkCpu   = CreateWindowW(L"BUTTON", L"Also run CPU/RAM test on core 1", WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, 460, 46, 300, 22, h, (HMENU)ID_CPU, nullptr, nullptr);
        g_lblStats = CreateWindowW(L"STATIC", L"", WS_CHILD | WS_VISIBLE, 12, 82, 760, 112, h, (HMENU)ID_STATS, nullptr, nullptr);
        g_log = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
                                WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE | ES_READONLY | ES_AUTOVSCROLL,
                                12, 200, 760, 300, h, (HMENU)ID_LOG, nullptr, nullptr);
        HWND all[] = {g_lblPort, g_btnStart, g_btnStop, g_btnFlash, g_btnUf2, g_chkCpu, g_lblStats, g_log};
        for (HWND c : all) SendMessageW(c, WM_SETFONT, (WPARAM)f, TRUE);
        SendMessageW(g_chkCpu, BM_SETCHECK, BST_CHECKED, 0);
        SendMessageW(g_log, EM_SETLIMITTEXT, 0x7FFFFFFF, 0);
        scanPorts();
        SetTimer(h, TIMER_STATS, 500, nullptr);
        return 0;
    }
    case WM_SIZE:
        layout(LOWORD(l), HIWORD(l));
        return 0;
    case WM_COMMAND:
        switch (LOWORD(w)) {
        case ID_START: startStreaming(); break;
        case ID_STOP:  stopStreaming();  break;
        case ID_FLASH: startFlashTest(); break;
        case ID_UF2:   startUpload();    break;
        }
        return 0;
    case WM_TIMER:
        if (w == TIMER_STATS) updateStats();
        else if (w == TIMER_RESCAN) {
            KillTimer(h, TIMER_RESCAN);
            if (!g_streaming && !g_busy) {
                scanPorts();
                logLine(g_ports.empty() ? L"No Pico COM port visible yet (the new firmware may not have USB serial)."
                                        : L"Pico visible again: " + g_ports[0] + (g_ports.size() > 1 ? L" ..." : L""));
            }
        }
        return 0;
    case WM_DEVICECHANGE:
        if (!g_streaming && !g_busy) scanPorts();
        return TRUE;
    case WM_APP_LOG: {
        std::unique_ptr<std::wstring> s((std::wstring*)l);
        logLine(*s);
        return 0;
    }
    case WM_APP_BUSYDONE:
        if (g_work.joinable()) g_work.join();
        g_workRun = false;
        g_busy = false;
        refreshUi();
        logLine(w ? L"--- Upload finished ---" : L"--- Flash test finished ---");
        if (w) SetTimer(h, TIMER_RESCAN, 3000, nullptr);
        else scanPorts();
        return 0;
    case WM_DESTROY:
        KillTimer(h, TIMER_STATS);
        g_workRun = false;
        if (g_work.joinable()) g_work.join();
        stopStreaming();
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(h, m, w, l);
}

int WINAPI wWinMain(HINSTANCE hi, HINSTANCE, PWSTR, int show) {
    SetErrorMode(SEM_FAILCRITICALERRORS);       // no "insert disk" pop-ups while looking for the BOOTSEL drive
    initCrc();
    INITCOMMONCONTROLSEX icc;
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_STANDARD_CLASSES;
    InitCommonControlsEx(&icc);
    WNDCLASSW wc;
    ZeroMemory(&wc, sizeof(wc));
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hi;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    wc.lpszClassName = L"PicoTesterWnd";
    RegisterClassW(&wc);
    HWND h = CreateWindowW(wc.lpszClassName, L"RP Pico Tester", WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, CW_USEDEFAULT, 900, 640,
                           nullptr, nullptr, hi, nullptr);
    ShowWindow(h, show);
    MSG msg;
    while (GetMessageW(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
    }
    return 0;
}

// Fallback entry point: if the project uses the Console subsystem the linker wants main().
int main() {
    return wWinMain(GetModuleHandleW(nullptr), nullptr, GetCommandLineW(), SW_SHOWDEFAULT);
}
