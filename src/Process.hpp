#pragma once

// A minimal fork/exec wrapper with optional pipes on stdin and stdout.
//
// No shell is involved anywhere: the argument vector is passed straight to
// execvp, so paths with spaces, quotes, or a leading dash need no escaping and
// there is no injection surface. `describe()` renders a shell-ish line purely
// for --dump-commands and error messages.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace hxc {

    class CSubprocess {
      public:
        struct SOptions {
            bool pipeStdin  = false;
            bool pipeStdout = false;
            // When false the child's stderr is this process's stderr, which is what
            // every streaming ffmpeg wants: its diagnostics interleave with ours and
            // nothing can deadlock on an unread pipe.
            bool quietStderr = false;
        };

        ~CSubprocess();
        CSubprocess(const CSubprocess&)            = delete;
        CSubprocess& operator=(const CSubprocess&) = delete;

        static std::unique_ptr<CSubprocess> spawn(const std::vector<std::string>& argv, const SOptions& options, std::string& error);

        bool                                writeAll(const void* data, size_t bytes);
        // Reads exactly `bytes`, looping over short reads. False means the child
        // closed the pipe first; `eof` distinguishes a clean end from an error.
        bool                                readExact(void* data, size_t bytes, bool& eof);
        size_t                              readSome(void* data, size_t bytes);
        void                                closeStdin();

        // Reaps the child. Returns the exit status, or -signal for a signalled
        // death. Safe to call more than once.
        int                                 wait();
        // Ends the child without waiting for it. Used when one worker of a
        // segmented render has failed and the rest are now doing work nobody
        // will read. A no-op once the child has been reaped.
        void                                terminate();

        const std::vector<std::string>&     argv() const {
            return m_argv;
        }
        std::string describe() const;

      private:
        CSubprocess() = default;

        std::vector<std::string> m_argv;
        int                      m_stdin   = -1;
        int                      m_stdout  = -1;
        int                      m_pid     = -1;
        bool                     m_reaped  = false;
        int                      m_status  = 0;
    };

    // Runs a command to completion, capturing stdout. Used for the one-shot
    // ffprobe calls, where the output is bounded and buffering is free.
    bool runCapture(const std::vector<std::string>& argv, std::string& stdoutText, std::string& error);

    std::string describeArgv(const std::vector<std::string>& argv);

    // Cores, not hardware threads. The distinction matters for the segmented
    // render's default job count: each worker runs a decoder that will happily
    // saturate every sibling thread it is given, so counting SMT siblings as
    // independent workers oversubscribes by exactly two.
    int         physicalCoreCount();

    // This executable, as `execvp` would need to find it again. Empty when the
    // path cannot be recovered, which is what makes --worker-binary exist.
    std::string executablePath();

    // Set by --dump-commands: every spawn logs its argument vector.
    void setCommandTracing(bool enabled);
    bool commandTracing();

}
