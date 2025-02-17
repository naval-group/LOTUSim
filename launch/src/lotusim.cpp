#include <unistd.h>

#include <CLI/CLI.hpp>
#include <cstdlib>
#include <filesystem>
#include <iostream>

// Default values
std::string LOTUSIM_WS =
    std::getenv("LOTUSIM_WS") ? std::getenv("LOTUSIM_WS") : "/lotusim_ws";
std::string LOTUSIM_PATH = LOTUSIM_WS + "/install/share/lotusim";
std::string ASSETS_PATH = LOTUSIM_PATH + "/assets";
std::string MODEL_PATH = ASSETS_PATH + "/models";
std::string WORLD_PATH = ASSETS_PATH + "/worlds";
std::string DEFAULT_WORLD = "mas.world";

// Flags
bool DEBUG = false;
bool GUI = false;

void validate_setup()
{
    if (!std::filesystem::exists("/opt/ros")) {
        std::cerr
            << "\033[0;31mError:\033[0m ROS2 is not installed or setup.bash is missing.\n";
        exit(1);
    }
    if (!std::filesystem::exists(LOTUSIM_WS)) {
        std::cerr
            << "\033[0;31mError:\033[0m Lotusim workspace does not exist at "
            << LOTUSIM_WS << "\n";
        exit(1);
    }
    if (!std::filesystem::exists(LOTUSIM_PATH)) {
        std::cerr << "\033[0;31mError:\033[0m Lotusim repo does not exist at "
                  << LOTUSIM_PATH << "\n";
        exit(1);
    }
}

void run_simulation(const std::string& world_name)
{
    validate_setup();

    // Set the environment variables directly in the program
    setenv(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        (LOTUSIM_WS + "/install/lib").c_str(),
        1);
    setenv("GZ_GUI_PLUGIN_PATH", (LOTUSIM_WS + "/install/lib").c_str(), 1);
    setenv("GZ_SIM_RESOURCE_PATH", MODEL_PATH.c_str(), 1);

    std::string world_file =
        WORLD_PATH + "/" + (world_name.empty() ? DEFAULT_WORLD : world_name);
    std::cout << "Running the simulation: \033[0;32m" << world_file
              << "\033[0m\n";
    std::cout << "GZ_SIM_SYSTEM_PLUGIN_PATH: \033[0;32m"
              << getenv("GZ_SIM_SYSTEM_PLUGIN_PATH") << "\033[0m\n";
    std::cout << "GZ_SIM_RESOURCE_PATH: \033[0;32m"
              << getenv("GZ_SIM_RESOURCE_PATH") << "\033[0m\n";

    std::string command = "bash -c '"
                          "source /opt/ros/humble/setup.bash && "
                          "source " +
                          LOTUSIM_WS +
                          "/install/setup.bash && "
                          "gz sim " +
                          std::string(DEBUG ? "-v4 " : "") +
                          (GUI ? "-s " : "") + "-r " + world_file + "'";

    // Run the simulation using popen to capture the output and execute the
    // command
    FILE* fp = popen(command.c_str(), "r");
    if (fp == nullptr) {
        std::cerr << "Failed to execute command\n";
        exit(1);
    }

    // Read and output the result of the command (optional)
    char buffer[128];
    while (fgets(buffer, sizeof(buffer), fp) != nullptr) {
        std::cout << buffer;
    }

    fclose(fp);
}

int main(int argc, char** argv)
{
    CLI::App app{"Lotusim Simulation Controller"};

    // Commands
    auto run_cmd = app.add_subcommand("run", "Run the simulation");
    std::string world_name;
    run_cmd->add_option("world_name", world_name, "Optional world name")
        ->default_val(DEFAULT_WORLD);
    run_cmd->callback([&]() { run_simulation(world_name); });

    app.callback([&]() {
        std::cerr << "Unknown command or invalid arguments.\n\n";
        std::cerr << app.help() << "\n";
        app.exit(CLI::RuntimeError());  // Exit with an error code
    });
    // Parse arguments
    try {
        CLI11_PARSE(app, argc, argv);
    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    }

    return 0;
}
