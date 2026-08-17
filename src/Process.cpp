#include "Process.hpp"
#include "Log.hpp"

#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

namespace hxc {

    namespace {
        bool g_traceCommands = false;

        void closeIfOpen(int& fd) {
            if (fd >= 0) {
                ::close(fd);
                fd = -1;
            }
        }
    }

    void setCommandTracing(bool enabled) {
        g_traceCommands = enabled;
    }

    bool commandTracing() {
        return g_traceCommands;
    }

    std::string describeArgv(const std::vector<std::string>& argv) {
        std::string out;
        for (const auto& ARG : argv) {
            if (!out.empty())
                out += ' ';
            const bool NEEDS_QUOTES = ARG.empty() || ARG.find_first_of(" \t\"'\\$`") != std::string::npos;
            if (!NEEDS_QUOTES) {
                out += ARG;
                continue;
            }
            out += '\'';
            for (char c : ARG) {
                if (c == '\'')
                    out += "'\\''";
                else
                    out += c;
            }
            out += '\'';
        }
        return out;
    }

    std::string CSubprocess::describe() const {
        return describeArgv(m_argv);
    }

    CSubprocess::~CSubprocess() {
        // A child still running at destruction is one being abandoned mid-stream -
        // a decoder the composite stopped reading, or a pipeline being torn down
        // after an error. Kill it outright before touching the pipes.
        //
        // The alternative, closing our end and letting the child discover a broken
        // pipe, is both noisy and unreliable: it only works if *nobody else* holds a
        // duplicate of that pipe. Since the pipes are opened O_CLOEXEC no sibling
        // can hold one any more, but a SIGKILL is decisive regardless and costs
        // nothing at teardown.
        if (m_pid > 0 && !m_reaped)
            ::kill(m_pid, SIGKILL);
        closeIfOpen(m_stdin);
        closeIfOpen(m_stdout);
        if (m_pid > 0 && !m_reaped)
            wait();
    }

    std::unique_ptr<CSubprocess> CSubprocess::spawn(const std::vector<std::string>& argv, const SOptions& options, std::string& error) {
        error.clear();
        if (argv.empty()) {
            error = "cannot spawn an empty command";
            return nullptr;
        }

        if (g_traceCommands)
            HXC_INFO("+ {}", describeArgv(argv));

        // Every pipe is close-on-exec so a later child cannot inherit an earlier
        // one's pipes. Without this two sibling ffmpeg processes hold duplicates of
        // each other's fds, so closing our end never reaches the child as EOF or
        // EPIPE and the pipeline deadlocks. dup2() in the child clears the flag on
        // the descriptor it installs, so stdin/stdout still cross the exec.
        static const bool IGNORE_SIGPIPE = [] {
            // A child that exits early must surface as EPIPE from write(), not as
            // the death of this process.
            ::signal(SIGPIPE, SIG_IGN);
            return true;
        }();
        (void)IGNORE_SIGPIPE;

        int inPipe[2]  = {-1, -1};
        int outPipe[2] = {-1, -1};
        if (options.pipeStdin && ::pipe2(inPipe, O_CLOEXEC) != 0) {
            error = std::string("pipe() for stdin failed: ") + std::strerror(errno);
            return nullptr;
        }
        if (options.pipeStdout && ::pipe2(outPipe, O_CLOEXEC) != 0) {
            error = std::string("pipe() for stdout failed: ") + std::strerror(errno);
            closeIfOpen(inPipe[0]);
            closeIfOpen(inPipe[1]);
            return nullptr;
        }

        const pid_t PID = ::fork();
        if (PID < 0) {
            error = std::string("fork() failed: ") + std::strerror(errno);
            closeIfOpen(inPipe[0]);
            closeIfOpen(inPipe[1]);
            closeIfOpen(outPipe[0]);
            closeIfOpen(outPipe[1]);
            return nullptr;
        }

        if (PID == 0) {
            // Child. Only async-signal-safe work between fork and exec.
            if (options.pipeStdin) {
                ::dup2(inPipe[0], STDIN_FILENO);
                ::close(inPipe[0]);
                ::close(inPipe[1]);
            }
            if (options.pipeStdout) {
                ::dup2(outPipe[1], STDOUT_FILENO);
                ::close(outPipe[0]);
                ::close(outPipe[1]);
            }
            if (options.quietStderr) {
                const int NULL_FD = ::open("/dev/null", O_WRONLY);
                if (NULL_FD >= 0) {
                    ::dup2(NULL_FD, STDERR_FILENO);
                    ::close(NULL_FD);
                }
            }
            // The parent ignores SIGPIPE; the child must not inherit that, or a
            // decoder whose consumer went away would spin instead of ending.
            ::signal(SIGPIPE, SIG_DFL);

            std::vector<char*> raw;
            raw.reserve(argv.size() + 1);
            for (const auto& ARG : argv)
                raw.push_back(const_cast<char*>(ARG.c_str()));
            raw.push_back(nullptr);
            ::execvp(raw[0], raw.data());
            ::_exit(127);
        }

        auto process     = std::unique_ptr<CSubprocess>(new CSubprocess());
        process->m_argv  = argv;
        process->m_pid   = PID;
        if (options.pipeStdin) {
            ::close(inPipe[0]);
            process->m_stdin = inPipe[1];
        }
        if (options.pipeStdout) {
            ::close(outPipe[1]);
            process->m_stdout = outPipe[0];
        }
        return process;
    }

    bool CSubprocess::writeAll(const void* data, size_t bytes) {
        if (m_stdin < 0)
            return false;

        const auto* cursor    = static_cast<const uint8_t*>(data);
        size_t      remaining = bytes;
        while (remaining > 0) {
            const ssize_t WROTE = ::write(m_stdin, cursor, remaining);
            if (WROTE < 0) {
                if (errno == EINTR)
                    continue;
                HXC_DEBUG("write to child failed: {}", std::strerror(errno));
                return false;
            }
            cursor += WROTE;
            remaining -= static_cast<size_t>(WROTE);
        }
        return true;
    }

    size_t CSubprocess::readSome(void* data, size_t bytes) {
        if (m_stdout < 0)
            return 0;
        for (;;) {
            const ssize_t READ = ::read(m_stdout, data, bytes);
            if (READ < 0) {
                if (errno == EINTR)
                    continue;
                return 0;
            }
            return static_cast<size_t>(READ);
        }
    }

    bool CSubprocess::readExact(void* data, size_t bytes, bool& eof) {
        eof              = false;
        auto*  cursor    = static_cast<uint8_t*>(data);
        size_t remaining = bytes;
        while (remaining > 0) {
            const size_t READ = readSome(cursor, remaining);
            if (READ == 0) {
                eof = remaining == bytes;
                return false;
            }
            cursor += READ;
            remaining -= READ;
        }
        return true;
    }

    void CSubprocess::closeStdin() {
        closeIfOpen(m_stdin);
    }

    int CSubprocess::wait() {
        if (m_reaped)
            return m_status;
        if (m_pid <= 0)
            return -1;

        closeIfOpen(m_stdin);
        int status = 0;
        while (::waitpid(m_pid, &status, 0) < 0) {
            if (errno != EINTR)
                break;
        }
        m_reaped = true;
        if (WIFEXITED(status))
            m_status = WEXITSTATUS(status);
        else if (WIFSIGNALED(status))
            m_status = -WTERMSIG(status);
        else
            m_status = -1;
        return m_status;
    }

    bool runCapture(const std::vector<std::string>& argv, std::string& stdoutText, std::string& error) {
        stdoutText.clear();

        CSubprocess::SOptions options;
        options.pipeStdout = true;
        auto process       = CSubprocess::spawn(argv, options, error);
        if (!process)
            return false;

        char buffer[64 * 1024];
        for (;;) {
            const size_t READ = process->readSome(buffer, sizeof(buffer));
            if (READ == 0)
                break;
            stdoutText.append(buffer, READ);
        }

        const int STATUS = process->wait();
        if (STATUS != 0) {
            error = std::format("`{}` exited with status {}", describeArgv(argv), STATUS);
            return false;
        }
        return true;
    }

}
