
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose3d.h>

namespace CameraConstants {

constexpr double kYawP = 0.1;
constexpr double kYawI = 0.0;
constexpr double kYawD = 0.0;

constexpr double kPitchP = 0.3;
constexpr double kPitchI = 0.0;
constexpr double kPitchD = 0.0;

constexpr int counterMax = 250;
constexpr int yawCounterMax = 50;

// Min and Max standard deviations for the apriltag detetion 
constexpr double kMinStandardDeviation = 0.2;
constexpr double kMaxStandardDeviation = 3.0;

// Max speed allowed for adding vidion measurments to the robot pose esitmator
constexpr double kMaxEstimationSpeed = 0.25; // mps

/**
 * @param distance The raw distance from the apriltag
 * 
 * @return The standard deviation value for the distance
*/
double GetStandardDeviationFromDistance(double distance);

// Pose3d/transformation2d of the camera relative to the robot
// X if forward, Y is Left, Z is up 
namespace FrontCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)0.250, (units::meter_t)-0.185, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)std::numbers::pi / 12, (units::radian_t)0.0};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace FrontCamera

namespace BackLeftCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)0.4125, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)std::numbers::pi * -0.1116883853, (units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 1.25};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace BackLeftCamera

namespace BackRightCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)-0.4125, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 0.1116883853, (units::radian_t)std::numbers::pi * 0.75};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace BackRightCamera

namespace OakDLiteCamera {
    const frc::Translation3d kTranlation3d{(units::meter_t)-0.250, (units::meter_t)-0.08, (units::meter_t)0.2286};
    const frc::Rotation3d kRotation3d{(units::radian_t)0.0, (units::radian_t)0.0, (units::radian_t)std::numbers::pi};
    const frc::Pose3d kPose3d{kTranlation3d, kRotation3d};
} // namespace OakDLiteCamera

} // namespace CameraConstants
