#pragma once


namespace rtt {

    class WorldConfig {

    private:
        // The amount of cameras connected to the vision.
        unsigned int _num_cams;

    public:
        /**
         * Creates a default world configuration.
         */
        WorldConfig();

        unsigned int num_cams() { return _num_cams; };

        void set_num_cams(unsigned int amount) { _num_cams = amount; };

    };

}
