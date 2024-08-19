#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__

namespace liquidai {

/**
 * @brief Wave parameters. TBC.
 *
 */
struct WaveParameters {
    float amplitude = 1.0;
    float period = 10.0;
    float direction[2] = {1.0, 0.0};
};

struct WindParameters {
    float direction[2] = {1.0, 0.0};
    float velocity = 1.0;
};

}  // namespace liquidai
#endif