"""Path steering and follow/hold/finish policy; no ROS, image or GUI dependency."""
from dataclasses import dataclass, field
import math


def pure_pursuit_steering(target, wheelbase):
    x, y = target
    if not all(math.isfinite(v) for v in (x, y, wheelbase)) or x <= 0 or wheelbase <= 0:
        raise ValueError('Pure pursuit requires a finite forward target and positive wheelbase')
    return math.atan(wheelbase*2*y/(x*x+y*y))


def steering_for_observation(observation, options, wheelbase):
    if observation.error is None:
        return 0.0
    angle = (pure_pursuit_steering(observation.target, wheelbase)
             if observation.target is not None else -options.kp*observation.error)
    return max(-options.max_steer, min(options.max_steer, angle))


@dataclass
class Decision:
    publish: bool = False
    speed: float = 0.0
    steering: float = 0.0
    events: list = field(default_factory=list)


class FollowController:
    def __init__(self, options, now):
        self.options = options
        self.error = None
        self.steering = 0.0
        self.good_frames = 0
        self.image_at = self.odom_at = float('-inf')
        self.position = self.start_position = None
        self.started_at = self.lost_since = None
        self.wait_started = now
        self.done = False
        self.last_log = 0.0

    def observe(self, error, steering, now):
        if error is not None and not all(math.isfinite(v) for v in (error, steering)):
            error = None
        self.image_at, self.error = now, error
        self.good_frames = self.good_frames+1 if error is not None else 0
        if error is not None:
            self.steering = steering

    def invalidate_image(self):
        self.error, self.good_frames = None, 0

    def odometry(self, position, now):
        self.position, self.odom_at = position, now

    def arm_after_snapshot(self, now):
        self.good_frames, self.wait_started = 0, now

    def step(self, now):
        o = self.options
        if not o.drive or self.done:
            return Decision()
        result = Decision(publish=True)
        image_ok = now-self.image_at < o.image_timeout
        odom_ok = self.position is not None and now-self.odom_at < o.odom_timeout
        detected = self.error is not None and self.good_frames >= o.required_good_frames
        if not image_ok or not odom_ok or not detected:
            if o.continuous:
                if self.lost_since is None:
                    result.events.append(('warning', 'Stopped: waiting for valid lane, camera and odom; '
                                          'will resume when detection recovers'))
                    self.lost_since = now
                if not image_ok or not odom_ok:
                    self.good_frames = 0
                return result
            reason = None
            if self.started_at is None:
                if now-self.wait_started > o.acquisition_timeout:
                    reason = f'Failed to acquire lane/odom within {o.acquisition_timeout:g} seconds'
            else:
                if self.lost_since is None:
                    self.lost_since = now
                if now-self.lost_since >= o.loss_timeout:
                    reason = 'Lane or camera/odom lost: safe stop'
            if reason:
                self.done = True
                result.events.append(('info', reason))
            return result
        recovered = self.lost_since is not None
        self.lost_since = None
        if self.started_at is None:
            self.started_at, self.start_position = now, self.position
            result.events.append(('info', f'Lane acquired; beginning drive at {o.speed:.2f} m/s'))
        elif o.continuous and recovered:
            result.events.append(('info', 'Lane and sensors recovered; resuming drive'))
        distance = math.hypot(self.position[0]-self.start_position[0], self.position[1]-self.start_position[1])
        reason = None
        if not o.continuous and distance >= o.max_distance:
            reason = f'Distance limit reached: {distance:.2f} m'
        elif not o.continuous and now-self.started_at >= o.seconds:
            reason = 'Time limit reached'
        if reason:
            self.done = True
            result.events.append(('info', reason))
            return result
        result.speed, result.steering = o.speed, self.steering
        if now-self.last_log > 1.0:
            result.events.append(('info', f'distance={distance:.2f}m, error={self.error:+.3f}, '
                                  f'steer={math.degrees(self.steering):+.2f}deg'))
            self.last_log = now
        return result
