#include "safety_monitor.hpp"

#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    const arl::RoverPose pose{100.0, 50.0, 90.0};
    const arl::SafetyConfig config{0.60, 40.0, 1.50, 1.0, 5.0};
    const double speedKph = 36.0;

const std::vector<arl::Detection> detections{
    {"wall",     10.0,  0.5, 0.90},
    {"box",      18.0, -1.5, 0.80},
    {"person",    5.0,  2.0, 0.95},
    {"bad_conf",  8.0,  0.0, 0.40},
    {"too_far",  50.0,  0.0, 0.99},
    {"behind",   -5.0,  0.5, 0.90},
};

    const auto obstacles = arl::processDetections(detections, pose, config);
    const auto nearest = arl::findNearestObstacle(obstacles);
    const double stoppingDistance = arl::calculateStoppingDistance(speedKph, config);
    const bool emergencyBrake = arl::shouldEmergencyBrake(obstacles, speedKph, config);

    std::cout << std::fixed << std::setprecision(1);
    std::cout << "ARL Rover Safety Monitor\n";
    std::cout << "Valid detections: " << obstacles.size() << '\n';

    if (nearest) {
        std::cout << "Nearest obstacle: " << nearest->id << '\n';
        std::cout << "World position: (" << nearest->worldX << ", "
                  << nearest->worldY << ")\n";
    } else {
        std::cout << "Nearest obstacle: none\n";
    }

    std::cout << "Stopping distance: " << stoppingDistance << " m\n";
    std::cout << "Emergency brake: " << (emergencyBrake ? "ENGAGE" : "clear") << '\n';

    return 0;
}
