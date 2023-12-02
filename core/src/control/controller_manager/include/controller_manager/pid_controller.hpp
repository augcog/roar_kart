#ifndef ROAR__CONTROLLER_MANAGER__PID_CONTROLLER_HPP_
#define ROAR__CONTROLLER_MANAGER__PID_CONTROLLER_HPP_

#include <array>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace roar
{
    namespace control
    {
        struct PidCoefficients
        {
            double k_p = 1.0;
            double k_i = 0.0;
            double k_d = 0.0;
            double min_cmd = 0.0;
            double max_cmd = 1.0;
            double min_i = 0.0;
            double max_i = 1.0;
        };

        class PidController
        {
        public:
            explicit PidController(std::string const &name, PidCoefficients const &coefficients);

            PidController();

            bool try_update_param(std::string const &name, rclcpp::Parameter const &param);

            void reset_integral_error(double integral_error);

            double integral_error();

            double update(double new_error, double actual_dt);

            const PidCoefficients &params() const;

        private:
            std::string name_;
            PidCoefficients coefficients_;

            double integral_error_{};
            double last_error_{};
            double error_{};
        };
    } // namespace control

} // namespace race

#endif // ROAR__CONTROLLER_MANAGER__PID_CONTROLLER_HPP_
