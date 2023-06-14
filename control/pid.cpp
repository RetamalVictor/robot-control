#include <pybind11/pybind11.h>
#include <cmath>
#include <vector>

# define M_PI           3.14159265358979323846  /* pi */
namespace py = pybind11;

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
        : Kp(kp), Ki(ki), Kd(kd), prev_error(0), integral(0) {}

    double update(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;
        return output;
    }

private:
    double Kp, Ki, Kd;
    double prev_error;
    double integral;
};

PYBIND11_MODULE(pid_controller, m) {
    py::class_<PIDController>(m, "PIDController")
        .def(py::init<double, double, double>())
        .def("update", &PIDController::update);
}
