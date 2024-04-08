#ifndef __INTERFACE_UTILS_HPP__
#define __INTERFACE_UTILS_HPP__

#include "liquidai_msgs/msg/ais.hpp"
#include "liquidai_msgs/msg/ais_array.hpp"

#include <curl/curl.h>

enum class PublisherMethod { HTTP, HTTPS };

// Forward decleration
void pubAISHTTP(
    const std::string &ip_address, const liquidai_msgs::msg::AIS &msg);
void pubAISArrayHTTP(
    const std::string &ip_address, const liquidai_msgs::msg::AISArray &msg);
void getAISArrayHTTP(
    const std::string &ip_address, liquidai_msgs::msg::AISArray &msg);

/**
 * @brief Factory function for publishing
 *
 */
void pubAIS(
    const std::string &ip_address,
    const liquidai_msgs::msg::AIS &msg,
    PublisherMethod method)
{
    switch (method) {
    case PublisherMethod::HTTP: {
        pubAISHTTP(ip_address, msg);
        break;
    }
    case PublisherMethod::HTTPS: {
    }
    default: {
        throw std::runtime_error("pubAIS: HTTPS Not implemented");
    }
    }
}

/**
 * @brief Factory function for publishing array
 *
 */
void pubAISArray(
    const std::string &ip_address,
    const liquidai_msgs::msg::AISArray &msg,
    PublisherMethod method)
{
    switch (method) {
    case PublisherMethod::HTTP: {
        pubAISArrayHTTP(ip_address, msg);
        break;
    }
    case PublisherMethod::HTTPS: {
    }
    default: {
        throw std::runtime_error("pubAISArray: HTTPS Not implemented");
    }
    }
};

/**
 * @brief Factory function for getting AIS
 *
 */
void getAIS(
    const std::string &ip_address,
    liquidai_msgs::msg::AISArray &msg,
    PublisherMethod method)
{
    switch (method) {
    case PublisherMethod::HTTP: {
        getAISArrayHTTP(ip_address, msg);
        break;
    }
    case PublisherMethod::HTTPS: {
    }
    default: {
        throw std::runtime_error("getAIS: Not implemented");
    }
    }
};

/**
 * @brief Publish AIS info through HTTP
 *
 */
void pubAISHTTP(
    const std::string &ip_address, const liquidai_msgs::msg::AIS &msg)
{
    std::string _url = ip_address;
    _url.append("?");
    _url.append({"wType=AIS"});
    _url.append({"&wSession=null"});
    _url.append({"&wName=" + msg.name});
    _url.append({"&wLat=" + std::to_string(msg.latitude)});
    _url.append({"&wLon=" + std::to_string(msg.longitude)});
    _url.append({"&wSOG=" + std::to_string(msg.sog)});
    _url.append({"&wCOG=" + std::to_string(msg.cog)});
    _url.append({"&wHeading=" + std::to_string(msg.true_heading)});
    _url.append({"&wMMSI=" + msg.user_id});

    CURL *curl;
    CURLcode res;
    curl = curl_easy_init();
    if (!curl) {
        return;
    }
    curl_easy_setopt(curl, CURLOPT_URL, _url.c_str());

    res = curl_easy_perform(curl);
    if (res != CURLE_OK)
        fprintf(
            stderr,
            "curl_easy_perform() failed: %s\n",
            curl_easy_strerror(res));

    curl_easy_cleanup(curl);
}

/**
 * @brief Publish AISArray info through HTTP
 *
 */
void pubAISArrayHTTP(
    const std::string &ip_address, const liquidai_msgs::msg::AISArray &msg)
{
    // Not implemented
}

/**
 * @brief Get AISArray info through HTTP
 *
 */
void getAISArrayHTTP(
    const std::string &ip_address, liquidai_msgs::msg::AISArray &msg)
{
}

#endif