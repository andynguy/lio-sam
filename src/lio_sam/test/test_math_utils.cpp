// Unit tests for lio_sam_utils (MathUtils.h).
//
// Build & run:
//   colcon build --packages-select lio_sam
//   ./build/lio_sam/test_math_utils
//   ./build/lio_sam/test_math_utils --gtest_filter='Integrate.*'   (run one group)
//
// Pattern for every test:  Arrange (build inputs) -> Act (call the function) -> Assert (check)
//
// Each unfinished test calls GTEST_SKIP(), so it shows up as SKIPPED instead of passing
// silently. Delete the GTEST_SKIP() line once you write the body.

#include <gtest/gtest.h>

#include <cmath>

#include <Eigen/Dense>

#include "lio_sam/MathUtils.h"

namespace
{

// ---------------------------------------------------------------------------
// Shared constants & helpers
// ---------------------------------------------------------------------------
constexpr double kTol = 1e-9;
const Eigen::Vector3d kGravity(0.0, 0.0, -9.81);

// Why not Eigen's isApprox()? It uses a RELATIVE tolerance, so comparing against
// a zero vector always fails. A norm of the difference works for every case.
void expectVecNear(const Eigen::Vector3d& actual, const Eigen::Vector3d& expected,
                   double tol = kTol)
{
    EXPECT_LT((actual - expected).norm(), tol)
        << "\n  actual:   " << actual.transpose()
        << "\n  expected: " << expected.transpose();
}

void expectMatNear(const Eigen::Matrix3d& actual, const Eigen::Matrix3d& expected,
                   double tol = kTol)
{
    EXPECT_LT((actual - expected).norm(), tol)
        << "\n  actual:\n" << actual
        << "\n  expected:\n" << expected;
}

// Eigen does NOT zero-initialize, so always build states/biases through these.
utils::NavState makeState(const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity(),
                          const Eigen::Vector3d& p = Eigen::Vector3d::Zero(),
                          const Eigen::Vector3d& v = Eigen::Vector3d::Zero())
{
    utils::NavState s;
    s.R = R;
    s.p = p;
    s.v = v;
    return s;
}

utils::ImuBias zeroBias()
{
    utils::ImuBias b;
    b.ba = Eigen::Vector3d::Zero();
    b.bg = Eigen::Vector3d::Zero();
    return b;
}

// Feed the same IMU reading `n` times, like n consecutive samples from a 100 Hz IMU.
void runSteps(utils::NavState& state, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro,
              double dt, int n, const utils::ImuBias& bias = zeroBias())
{
    for (int i = 0; i < n; ++i)
        utils::integrate(state, acc, gyro, dt, bias, kGravity);
}

}  // namespace

// ===========================================================================
// skew
// ===========================================================================

// #8  skew(a) * b must equal the cross product a x b.
TEST(Skew, MatchesCrossProduct)
{
    GTEST_SKIP() << "TODO";
    // Arrange: a = (1, 2, 3), b = (-4, 5, 0.5)
    // Act:     Eigen::Vector3d result = utils::skew(a) * b;
    // Assert:  expectVecNear(result, a.cross(b));
}

// ===========================================================================
// expSO3
// ===========================================================================

// #7  WORKED EXAMPLE: a 90 deg turn about z takes the x-axis onto the y-axis.
TEST(ExpSO3, QuarterTurnAboutZ)
{
    // Arrange
    const Eigen::Vector3d phi(0.0, 0.0, M_PI / 2.0);

    // Act
    const Eigen::Matrix3d R = utils::expSO3(phi);

    // Assert
    expectVecNear(R * Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY());
}

// #6  Exp(0) must be the identity, and must not be NaN (small-angle branch).
TEST(ExpSO3, ZeroIsIdentity)
{
    Eigen::Matrix3d R = utils::expSO3(Eigen::Vector3d::Zero());
    expectMatNear(R, Eigen::Matrix3d::Identity());
    EXPECT_FALSE(R.hasNaN());
}

// #5  Compare against Eigen's own implementation (a trusted reference).
TEST(ExpSO3, MatchesEigenAngleAxis)
{
    GTEST_SKIP() << "TODO";
    // Arrange: phi = (0.3, -1.2, 2.0)
    // Expected: Eigen::AngleAxisd(phi.norm(), phi.normalized()).toRotationMatrix()
    // Assert:  expectMatNear(utils::expSO3(phi), expected);
    // Bonus:   also check R.transpose() * R == I  and  R.determinant() == 1
}

// ===========================================================================
// integrate
// ===========================================================================

// #3  WORKED EXAMPLE: a level IMU sitting still reads +9.81 on z and must not move.
//     If p.z ends up at +4.9 you forgot "+ gravity"; at -4.9 the sign is flipped.
TEST(Integrate, StationaryLevelDoesNotMove)
{
    // Arrange
    utils::NavState state = makeState();                  // R = I, p = 0, v = 0
    const Eigen::Vector3d acc(0.0, 0.0, 9.81);            // what a still IMU measures
    const Eigen::Vector3d gyro = Eigen::Vector3d::Zero();

    // Act: 1 second of 100 Hz samples
    runSteps(state, acc, gyro, 0.01, 100);

    // Assert
    expectVecNear(state.p, Eigen::Vector3d::Zero());
    expectVecNear(state.v, Eigen::Vector3d::Zero());
    expectMatNear(state.R, Eigen::Matrix3d::Identity());
}

// #1  MOST IMPORTANT. A *tilted* IMU sitting still must not move either.
//     With R = I, R and R^T are identical, so the level test can't catch a transpose bug.
TEST(Integrate, StationaryTiltedDoesNotMove)
{
    GTEST_SKIP() << "TODO";
    // Arrange: R = utils::expSO3({0.3, 0.2, 1.0});   state = makeState(R);
    //          acc = R.transpose() * Eigen::Vector3d(0, 0, 9.81);   (gravity seen in body frame)
    //          gyro = 0
    // Act:     runSteps(state, acc, gyro, 0.01, 100);
    // Assert:  p == 0, v == 0
}

// #2  MOST IMPORTANT. Gyro is measured in the BODY frame -> R_new = R * Exp(w dt), not Exp(w dt) * R.
//     Starting from a rolled orientation makes the two orders give different answers.
TEST(Integrate, GyroRotatesInBodyFrame)
{
    GTEST_SKIP() << "TODO";
    // Arrange: R0 = utils::expSO3({M_PI / 2, 0, 0});   state = makeState(R0);
    //          gyro = (0, 0, 0.5);  acc = R0.transpose() * Eigen::Vector3d(0, 0, 9.81)
    // Act:     one step, dt = 1.0
    // Assert:  expectMatNear(state.R, R0 * utils::expSO3({0, 0, 0.5}));
}

// #4  Constant 1 m/s^2 forward for 1 s: v = 1 m/s, p = 0.5 m  (x = 1/2 a t^2).
TEST(Integrate, ConstantAccelerationKinematics)
{
    GTEST_SKIP() << "TODO";
    // Arrange: R = I, acc = (1, 0, 9.81), gyro = 0
    // Act:     runSteps(state, acc, gyro, 0.01, 100);
    // Assert:  v == (1, 0, 0),  p == (0.5, 0, 0)
}

// #9  The accelerometer bias must be subtracted before integrating.
TEST(Integrate, AccelBiasIsRemoved)
{
    GTEST_SKIP() << "TODO";
    // Arrange: acc = (0.1, 0, 9.81);  bias = zeroBias();  bias.ba = (0.1, 0, 0)
    // Act:     runSteps(state, acc, gyro, 0.01, 100, bias);
    // Assert:  p == 0, v == 0
}

// #10 Spinning at pi/2 rad/s for 1 s (100 samples) gives a 90 deg yaw.
TEST(Integrate, PureYawQuarterTurn)
{
    GTEST_SKIP() << "TODO";
    // Arrange: gyro = (0, 0, M_PI / 2);  acc = (0, 0, 9.81)
    // Act:     runSteps(state, acc, gyro, 0.01, 100);
    // Assert:  expectMatNear(state.R, utils::expSO3({0, 0, M_PI / 2}));
}
