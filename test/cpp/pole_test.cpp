#include <gtest/gtest.h>

#include "pole_estimator.hpp"

#include <limits>

TEST (pole, estimator)
{
  const pole_camera camera{};

  // The image center looks along the camera axis: back along -x, 10 deg up.
  const auto center = camera.ray (960.0, 540.0);
  ASSERT_NEAR (center[0], -std::cos (0.1745), 1e-6) << "center ray x";
  ASSERT_NEAR (center[1], 0.0, 1e-6) << "center ray y";
  ASSERT_NEAR (center[2], std::sin (0.1745), 1e-6) << "center ray up";
  // Facing -x, the right of the image is +y; the top of the image is up.
  ASSERT_TRUE (camera.ray (1500.0, 540.0)[1] > 0.1) << "image right is +y";
  ASSERT_TRUE (camera.ray (960.0, 100.0)[2] > center[2]) << "image top is up";

  // pixel () undoes ray (), also with the pan-tilt mount turned.
  pole_camera turned = camera;
  turned.pose = pole_camera::oriented (camera.position (), 2.2, -0.6);
  const vec3 far = camera.position ();
  const vec3 r = turned.ray (300.0, 800.0);
  const auto px = turned.pixel (far + 20 * r);
  ASSERT_TRUE (px.has_value ()) << "pixel in front";
  ASSERT_NEAR ((*px)[0], 300.0, 1e-6) << "pixel u round trip";
  ASSERT_NEAR ((*px)[1], 800.0, 1e-6) << "pixel v round trip";
  ASSERT_TRUE (!turned.pixel (far - 20 * r)) << "behind the camera";
  ASSERT_TRUE (!camera.sees ({ 40.0, 0.0, 3.0 })) << "drone behind the pole";

  const vec3 truth{ 0.0, 8.0, 8.0 };
  const vec3 offset = truth - camera.position ();
  const double range = offset.norm ();
  const vec3 ray = offset.normalized ();

  // Start 3 m off: range and bearing together must pull the state to the
  // true point, each one only along its own direction.
  pole_estimator estimator{ {}, camera.position () };
  estimator.initialize (0.0, { 2.0, 6.0, 9.0 }, 5.0);
  for (int i = 1; i <= 20; ++i)
    {
      estimator.advance_to (i * 0.01);
      ASSERT_TRUE (estimator.add_range (range)) << "range accepted";
      ASSERT_TRUE (estimator.add_bearing (ray)) << "bearing accepted";
    }
  const auto p = estimator.position ();
  ASSERT_NEAR (p[0], truth[0], 0.1) << "pole fix x";
  ASSERT_NEAR (p[1], truth[1], 0.1) << "pole fix y";
  ASSERT_NEAR (p[2], truth[2], 0.1) << "pole fix z";

  // Range alone only pins the distance: the state ends up on the sphere.
  pole_estimator range_only{ {}, camera.position () };
  range_only.initialize (0.0, { 2.0, 6.0, 9.0 }, 5.0);
  for (int i = 1; i <= 20; ++i)
    {
      range_only.advance_to (i * 0.01);
      (void)range_only.add_range (range);
    }
  ASSERT_NEAR ((range_only.position () - camera.position ()).norm (), range,
               0.1)
      << "range alone reaches the sphere";

  // A wrong detection far from the converged state is gated out.
  ASSERT_TRUE (!estimator.add_bearing (camera.ray (200.0, 900.0)))
      << "false detection rejected";
  ASSERT_TRUE (!estimator.add_range (range + 5.0)) << "range outlier rejected";

  // Measurements that are not numbers are rejected and leave it healthy.
  const double nan = std::numeric_limits<double>::quiet_NaN ();
  ASSERT_TRUE (!estimator.add_range (nan)) << "nan range rejected";
  ASSERT_TRUE (!estimator.add_bearing ({ nan, 0.0, 1.0 }))
      << "nan ray rejected";
  ASSERT_TRUE (!estimator.add_gps ({ nan, 0.0, 0.0 }, 1.0, 1.0))
      << "nan gps rejected";
  ASSERT_TRUE (estimator.healthy ()) << "healthy after nan input";

  // A drone straight above the pole: the ray is vertical.
  const vec3 above = camera.position () + vec3{ 0.0, 0.0, 10.0 };
  pole_estimator overhead{ {}, camera.position () };
  overhead.initialize (0.0, { above[0] + 0.5, above[1], above[2] }, 2.0);
  for (int i = 1; i <= 20; ++i)
    {
      overhead.advance_to (i * 0.01);
      (void)overhead.add_bearing ({ 0.0, 0.0, 1.0 });
      (void)overhead.add_range (10.0);
    }
  ASSERT_TRUE (overhead.healthy ()) << "vertical ray keeps it finite";
  ASSERT_NEAR ((overhead.position () - above).norm (), 0.0, 0.1)
      << "vertical ray fix";

  // An estimate sitting on the anchor has no bearing scale; still finite.
  pole_estimator on_anchor{ {}, camera.position () };
  on_anchor.initialize (0.0, camera.position (), 1.0);
  (void)on_anchor.add_bearing (center);
  ASSERT_TRUE (on_anchor.healthy ()) << "estimate on the anchor stays finite";

  on_anchor.reset ();
  ASSERT_TRUE (!on_anchor.initialized ()) << "reset forgets the state";
}
