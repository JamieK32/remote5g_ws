"""Save/restore one vehicle's world pose in Gazebo Classic (simulation only)."""
from copy import deepcopy
import math
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState


from remote5g_autonomy.errors import SimulationResetError


class GazeboStartPose:
    def __init__(self, node, entity='xtnetrc', timeout=5.0):
        self.node = node
        self.entity = entity
        self.timeout = timeout
        self.initial_pose = None
        self.get_client = node.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.set_client = node.create_client(SetEntityState, '/gazebo/set_entity_state')

    def _call(self, client, request, label):
        if not client.wait_for_service(timeout_sec=self.timeout):
            raise SimulationResetError(f'Gazebo {label} service unavailable; check simulation and ROS_DOMAIN_ID')
        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        if not future.done():
            future.cancel()
            raise SimulationResetError(f'Gazebo {label} request timed out')
        try:
            result = future.result()
        except Exception as exc:
            raise SimulationResetError(f'Gazebo {label} failed: {exc}') from exc
        if result is None or not result.success:
            raise SimulationResetError(f'Gazebo {label} failed for entity {self.entity!r}')
        return result

    def capture(self):
        # Check the return service before allowing any driving to begin.
        if not self.set_client.wait_for_service(timeout_sec=self.timeout):
            raise SimulationResetError('Gazebo set_entity_state unavailable; refusing to start return-enabled drive')
        response = self._call(self.get_client,
                              GetEntityState.Request(name=self.entity, reference_frame='world'),
                              'get_entity_state')
        pose = response.state.pose
        p, q = pose.position, pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not all(math.isfinite(v) for v in values) or abs(norm - 1.0) > 0.01:
            raise SimulationResetError('Gazebo returned an invalid initial pose')
        # Copy the full pose; odometry may use a different origin from world.
        self.initial_pose = deepcopy(pose)
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        self.node.get_logger().info(
            f'Initial world pose saved for {self.entity}: '
            f'x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}, yaw={math.degrees(yaw):.1f} deg')

    def restore(self):
        if self.initial_pose is None:
            raise SimulationResetError('No initial world pose recorded; vehicle was not reset')
        state = EntityState(name=self.entity, reference_frame='world')
        state.pose = deepcopy(self.initial_pose)
        # EntityState's default Twist is zero. Never restore the starting speed.
        self._call(self.set_client, SetEntityState.Request(state=state), 'set_entity_state')
        response = self._call(self.get_client,
                              GetEntityState.Request(name=self.entity, reference_frame='world'),
                              'verify initial pose')
        actual, expected = response.state.pose, self.initial_pose
        a, b = actual.position, expected.position
        distance = math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        aq, bq = actual.orientation, expected.orientation
        qa, qb = (aq.x, aq.y, aq.z, aq.w), (bq.x, bq.y, bq.z, bq.w)
        norm = math.sqrt(sum(v*v for v in qa) * sum(v*v for v in qb))
        dot = abs(sum(a*b for a, b in zip(qa, qb))) / norm if norm > 0 else float('nan')
        angle = 2 * math.acos(min(1.0, dot)) if math.isfinite(dot) else float('nan')
        if not math.isfinite(distance) or not math.isfinite(angle) or distance > 0.03 or angle > 0.05:
            raise SimulationResetError('Gazebo accepted reset but initial pose verification failed')
        self.node.get_logger().info(
            f'Returned {self.entity} to initial world pose; position error={distance:.4f} m')
