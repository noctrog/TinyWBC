#include <dart/dart.hpp>
#include <dart/dynamics/SmartPointer.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/ZeroDofJoint.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/math/Helpers.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <HyqWorldNode.hpp>

using namespace dart;

dynamics::SkeletonPtr
createFloor(void)
{
  const auto floor_shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(5.0, 5.0, 0.2));
  const auto floor_skeleton = dynamics::Skeleton::create();
  const auto floorAndWorld
    = floor_skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>();
  const auto floor_body = floorAndWorld.second;
  floor_body->createShapeNodeWith<
    dynamics::VisualAspect,
    dynamics::CollisionAspect,
    dynamics::DynamicsAspect>(floor_shape);
  Eigen::Isometry3d floor_tf(Eigen::Isometry3d::Identity());
  floor_tf.translation().z() = -0.8;
  floor_body->getParentJoint()->setTransformFromParentBodyNode(floor_tf);
  return floor_skeleton;
}

dynamics::SkeletonPtr
loadHyqRobot(void)
{
  dart::utils::DartLoader loader;

  // Load robot from URDF
  loader.addPackageDirectory("hyq_urdf", "/home/ramon/Documents/programming/tiny_wbc/examples/hyq/hyq_urdf");
  dynamics::SkeletonPtr robot
    = loader.parseSkeleton("/home/ramon/Documents/programming/tiny_wbc/examples/hyq/hyq_urdf/urdf/hyq.urdf");

  robot->setName("hyq");

  return robot;
}

int main()
{
  const auto floor_skeleton = createFloor();
  const auto robot = loadHyqRobot();

  // Create a world and add the rigid body
  auto world = simulation::World::create();
  world->addSkeleton(robot);
  world->addSkeleton(floor_skeleton);

  // Wrap a WorldNode around it
  ::osg::ref_ptr<HyqWorldNode> node
      = new HyqWorldNode(world, robot);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = gui::osg::Viewer();
  viewer.addWorldNode(node);

  viewer.addInstructionText("Press space to start free falling the box.\n");
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.57f, 3.14f, 1.64f),
      ::osg::Vec3(0.00f, 0.00f, 0.00f),
      ::osg::Vec3(-0.24f, -0.25f, 0.94f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
