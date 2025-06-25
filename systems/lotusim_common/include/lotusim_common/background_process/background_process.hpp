#ifndef __BACKGROUND_PROCESS_HPP
#define __BACKGROUND_PROCESS_HPP

#ifdef _WIN32
    #include <windows.h>
#else if __linux__
    #include <signal.h>
    #include <unistd.h>
#endif

#include <boost/asio/read.hpp>
#include <boost/asio/readable_pipe.hpp>
#include <boost/process/v2.hpp>
#include <boost/system/error_code.hpp>
#include <iostream>

namespace proc = boost::process::v2;
namespace asio = boost::asio;

namespace lotusim::common {
/**
 * @brief Create a background process
 * Testing in progress
 *
 * @return int PID
 */
int create_process(const std::string& cmd)
{
    try {
        asio::io_context ctx;
        asio::readable_pipe p{ctx};

        const auto exe = proc::environment::find_executable("gcc");

        proc::process c{
            ctx,
            exe,
            {"--version"},
            proc::process_stdio{nullptr, p}};

        std::string line;
        boost::system::error_code ec;

        auto sz = asio::read(p, asio::dynamic_buffer(line), ec);
        if (ec) {
            throw std::runtime_error(ec_.message());
        }

        std::cout << "Gcc version: '" << line << "'" << std::endl;

        c.wait();

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}

bool killProcess(int pid)
{
#ifdef _WIN32
    // Open the process with necessary permissions
    HANDLE hProcess = OpenProcess(PROCESS_TERMINATE, FALSE, pid);
    if (hProcess == NULL) {
        std::cerr << "Failed to open process. Error: " << GetLastError()
                  << std::endl;
        return false;
    }
    // Terminate the process
    if (!TerminateProcess(hProcess, 0)) {
        std::cerr << "Failed to terminate process. Error: " << GetLastError()
                  << std::endl;
        return false;
    } else {
        std::cout << "Process " << pid << " terminated successfully."
                  << std::endl;
    }
    CloseHandle(hProcess);
#else if __linux__
    // Send SIGKILL to the process
    if (kill(pid, SIGKILL) == 0) {
        std::cout << "Process " << pid << " terminated successfully."
                  << std::endl;

    } else {
        perror("Failed to terminate process");
        return false;
    }
#endif
    return true;
}

}  // namespace lotusim::common

#endif