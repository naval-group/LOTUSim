#include "world_plugins/WaveParamPlugin.h"

namespace liquidai {
namespace gazebo {

WaveParamPlugin::WaveParamPlugin() : ParamNode()
{
    m_gz_node = std::make_shared<gz::transport::Node>();

    char const *const *argv{};
    rclcpp::init(0, argv);
    m_nh = std::make_shared<rclcpp::Node>("WaveParamPlugin");
}

void WaveParamPlugin::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &ecm,
    gz::sim::EventManager &eventMgr)
{
    auto sdf = const_cast<sdf::Element *>(_sdf.get());

    // Setting up gazebo wave topic
    std::string wave_param_topic_name = "wave_param";
    if (sdf->HasElement("topic"))
        wave_param_topic_name = sdf->Get<std::string>("topic");
    m_param_pub = m_gz_node->Advertise<gz_liquidai_msgs::msgs::WaveParam>(
        wave_param_topic_name);
    m_gz_node->Advertise(
        "get_" + wave_param_topic_name,
        &WaveParamPlugin::getParamCB,
        this);

    // Default gazebo Wave Param
    WaveParameters default_param;
    default_param.amplitude = 1.0;
    default_param.period = 1.0;
    default_param.direction[0] = 1;
    default_param.direction[1] = 0;

    // Reading the param file and setting the param
    std::string file_path = "";
    if (sdf->HasElement("param_file"))
        file_path = sdf->Get<std::string>("param_file");
    FileFormat file_type = FileFormat::UNKNOWN;
    if (sdf->HasElement("file_type")) {
        std::string file_type_name = sdf->Get<std::string>("file_type");
        for (auto &&c : file_type_name) {
            c = std::tolower(c);
        }
        switch (file_type_mapping[file_type_name]) {
            case (FileFormat::YAML): {
                file_type = FileFormat::YAML;
            }
        }
    }
    if (!loadParamFromFile(file_path, file_type, default_param)) {
        if (sdf->HasElement("wave_param")) {
            auto wave_para_ptr = sdf->GetElement("wave_param");
            if (wave_para_ptr->HasElement("amplitude")) {
                default_param.amplitude =
                    wave_para_ptr->Get<double>("amplitude");
            } else {
                logError("wave amplitude not found. Using default 1.0");
            }
            if (wave_para_ptr->HasElement("period")) {
                default_param.amplitude = wave_para_ptr->Get<double>("period");
            } else {
                logError("wave period not found. Using default 1.0");
            }
            if (wave_para_ptr->HasElement("heading")) {
                auto heading_ptr = wave_para_ptr->GetElement("heading");
                if (heading_ptr->HasElement("x"))
                    default_param.direction[0] = heading_ptr->Get<double>("x");
                if (heading_ptr->HasElement("y"))
                    default_param.direction[1] = heading_ptr->Get<double>("y");
            } else {
                logError("wave heading not found. Using default 1.0, 0");
            }
        } else {
            logError("WaveParamPlugin::Configure: No param file and default "
                     "param found. Setting default param");
        }
    }

    // Setting up ROS wave topic
    rclcpp::QoS qos(1);
    qos.transient_local();
    m_wave_param_pub = m_nh->create_publisher<liquidai_msgs::msg::WaveParam>(
        "wave_param",
        qos);

    // Setting up ROS dynamic param only if file is not used
    if (!usingFile()) {
        addParameter(
            "amplitude",
            rclcpp::ParameterValue(default_param.amplitude),
            "This holds the amplitude of wave");
        addParameter(
            "period",
            rclcpp::ParameterValue(default_param.period),
            "This holds the period of wave");
        addParameter(
            "direction_x",
            rclcpp::ParameterValue(default_param.direction[0]),
            "This holds the x axis direction of wave");
        addParameter(
            "direction_y",
            rclcpp::ParameterValue(default_param.direction[1]),
            "This holds the y axis direction of wave");
        m_dyn_params_handler = m_nh->add_on_set_parameters_callback(std::bind(
            &WaveParamPlugin::dynamicParametersCallback,
            this,
            std::placeholders::_1));
    }
    setParamMsg(getParam());
}

void WaveParamPlugin::PreUpdate(
    const gz::sim::UpdateInfo &info,
    gz::sim::EntityComponentManager &ecm)
{
    if (step(info.simTime)) {
        auto it = m_param_map.begin();
        std::advance(it, m_current_index);
        setParamMsg(it->second);
    }
}

void WaveParamPlugin::logError(std::string err_statement)
{
    gzerr << err_statement << "\n";
}

bool WaveParamPlugin::getParamCB(gz_liquidai_msgs::msgs::WaveParam &msg)
{
    msg = m_param_msg;
}

void WaveParamPlugin::setParamMsg(const WaveParameters &msg)
{
    // Set the gazebo msg and pub
    m_param_msg.set_amplitude(msg.amplitude);
    m_param_msg.set_period(msg.period);
    gz::msgs::Vector2d v_msg;
    v_msg.set_x(msg.direction[0]);
    v_msg.set_y(msg.direction[1]);
    m_param_msg.mutable_direction()->CopyFrom(v_msg);
    m_param_pub.Publish(m_param_msg);

    // Set the ROS msg and pub
    liquidai_msgs::msg::WaveParam ros_msg;
    ros_msg.amplitude = msg.amplitude;
    ros_msg.period = msg.period;
    ros_msg.direction[0] = msg.direction[0];
    ros_msg.direction[1] = msg.direction[1];
    m_wave_param_pub->publish(ros_msg);
}

void WaveParamPlugin::addParameter(
    const std::string &name,
    const rclcpp::ParameterValue &default_value,
    const std::string &description,
    bool read_only)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.name = name;
    descriptor.description = description;
    descriptor.read_only = read_only;
    m_nh->declare_parameter(descriptor.name, default_value, descriptor);
}

rcl_interfaces::msg::SetParametersResult
WaveParamPlugin::dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters)
{
    WaveParameters new_param = getParam();
    for (auto parameter : parameters) {
        const auto &param_type = parameter.get_type();
        const auto &param_name = parameter.get_name();

        if (param_type ==
            rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
            if (param_name == "amplitude") {
                double new_amplitude = parameter.as_double();
                if (new_amplitude <= 0) {
                    RCLCPP_WARN(
                        m_nh->get_logger(),
                        "You've set amplitude less or equal to 0, this isn't "
                        "allowed, changes will be ignored.");
                } else {
                    new_param.amplitude = new_amplitude;
                }
            }
            if (param_name == "period") {
                double new_period = parameter.as_double();
                if (new_period <= 0) {
                    RCLCPP_WARN(
                        m_nh->get_logger(),
                        "You've set period less or equal to 0, this isn't "
                        "allowed, changes will be ignored.");
                } else {
                    new_param.period = new_period;
                }
            }
            if (param_name == "direction_x") {
                new_param.direction[0] = parameter.as_double();
            }
            if (param_name == "direction_y") {
                new_param.direction[1] = parameter.as_double();
            }
        }
    }
    setParam(new_param), setParamMsg(new_param);
}

}  // namespace gazebo
}  // namespace liquidai

GZ_ADD_PLUGIN(
    liquidai::gazebo::WaveParamPlugin,
    gz::sim::System,
    liquidai::gazebo::WaveParamPlugin::ISystemPreUpdate,
    liquidai::gazebo::WaveParamPlugin::ISystemConfigure)