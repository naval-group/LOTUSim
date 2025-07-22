// This script is to binarize the launch script. We will use normal bashscript
// during development

#include <unistd.h>

#include <CLI/CLI.hpp>
#include <cstdlib>
#include <filesystem>
#include <iostream>

// Default values
std::string LOTUSIM_WS =
    std::getenv("LOTUSIM_WS") ? std::getenv("LOTUSIM_WS") : "/lotusim_ws";
std::string LOTUSIM_PATH = LOTUSIM_WS + "/src/lotusim";
std::string ASSETS_PATH = LOTUSIM_PATH + "/assets";
std::string MODEL_PATH = ASSETS_PATH + "/models";
std::string WORLD_PATH = ASSETS_PATH + "/worlds";
std::string DEFAULT_WORLD = "mas.world";

// Flags
bool DEBUG = false;
bool GUI = false;

void validate_setup()
{
    if (!std::filesystem::exists("/opt/ros/humble/setup.bash")) {
        std::cerr
            << "\033[0;31mError:\033[0m ROS2 Humble is not installed or setup.bash is missing.\n";
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

void build()
{
    chdir(LOTUSIM_WS.c_str());

    std::string cmd = "bash -c 'source /opt/ros/humble/setup.bash && "
                      "source " +
                      LOTUSIM_WS +
                      "/install/setup.bash && "
                      "colcon build --merge-install";

    if (DEBUG) {
        std::cout
            << "Debug mode enabled: Printing debug logs. Enabling LOTUSIM_TIDY\n";
        cmd += " --cmake-args -DLOTUSIM_TIDY=ON -DDEBUG=ON --symlink-install";
    }

    cmd += "'";

    if (system(cmd.c_str()) != 0) {
        std::cerr << "\033[0;31mError:\033[0m Build failed.\n";
        exit(1);
    }
}

void clean()
{
    system(("rm -rf " + LOTUSIM_WS + "/log " + LOTUSIM_WS + "/install " +
            LOTUSIM_WS + "/build")
               .c_str());
}

void run_simulation(const std::string& world_name)
{
    validate_setup();

    std::string GZ_SIM_SYSTEM_PLUGIN_PATH = LOTUSIM_WS + "/install/lib";
    std::string GZ_GUI_PLUGIN_PATH = LOTUSIM_WS + "/install/lib";
    std::string GZ_SIM_RESOURCE_PATH = MODEL_PATH;

    setenv(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        (LOTUSIM_WS + "/install/lib").c_str(),
        1);
    setenv("GZ_GUI_PLUGIN_PATH", (LOTUSIM_WS + "/install/lib").c_str(), 1);
    setenv("GZ_SIM_RESOURCE_PATH", MODEL_PATH.c_str(), 1);

    std::cout << "GZ_SIM_SYSTEM_PLUGIN_PATH: \033[0;32m"
              << GZ_SIM_SYSTEM_PLUGIN_PATH << "\033[0m\n";
    std::cout << "GZ_SIM_RESOURCE_PATH: \033[0;32m" << GZ_SIM_RESOURCE_PATH
              << "\033[0m\n";

    std::string world_file =
        WORLD_PATH + "/" + (world_name.empty() ? DEFAULT_WORLD : world_name);
    std::string cmd = "bash -c 'source /opt/ros/humble/setup.bash && "
                      "source " +
                      LOTUSIM_WS +
                      "/install/setup.bash && "
                      "gz sim " +
                      (DEBUG ? "-v4 " : "") + (GUI ? "" : "-s ") + "-r " +
                      world_file + "'";
    system(cmd.c_str());
}

int main(int argc, char** argv)
{
    CLI::App app{"Lotusim Simulation Controller"};

    // Global options
    app.add_option("--ws-path", LOTUSIM_WS, "Path to the Lotusim workspace");
    app.add_option(
        "--assets-path",
        ASSETS_PATH,
        "Path to the assets directory");
    app.add_flag("--debug", DEBUG, "Enable debug mode");
    app.add_flag("--gui", GUI, "Run gz sim GUI");

    // Commands
    auto install_cmd =
        app.add_subcommand("install", "Install dependencies")->callback([]() {
            std::cout << "Installing dependencies...\n";
            std::string cmd =
                "sudo -E " + LOTUSIM_PATH + "/launch/install_dep.sh";
            system(cmd.c_str());
            build();
        });

    auto clean_cmd =
        app.add_subcommand("clean", "Clean the project")->callback([]() {
            clean();
        });

    auto build_cmd =
        app.add_subcommand("build", "Build the project")->callback([]() {
            validate_setup();
            build();
        });

    auto clean_build_cmd =
        app.add_subcommand("clean_build", "Clean and build the project")
            ->callback([]() {
                clean();
                build();
            });

    auto doc_cmd =
        app.add_subcommand("doc", "Generate project documentation")->callback([]() {
            std::string doc_path = LOTUSIM_PATH + "/docs";
            chdir(doc_path.c_str());
            if (system("doxygen Doxyfile") != 0) {
                std::cerr
                    << "\033[0;31mError:\033[0m Documentation generation failed.\n";
                exit(1);
            }
            std::cout << "Docs generated at " << doc_path << "\n";
        });

    auto run_cmd = app.add_subcommand("run", "Run the simulation");
    std::string world_name;
    run_cmd->add_option("world_name", world_name, "Optional world name")
        ->default_val(DEFAULT_WORLD);
    run_cmd->callback([&]() { run_simulation(world_name); });

    // Parse arguments
    CLI11_PARSE(app, argc, argv);

    // Show help if no subcommand is provided
    if (app.get_subcommands().empty()) {
        std::cout << app.help() << std::endl;
        return 0;
    }

    return 0;
}
